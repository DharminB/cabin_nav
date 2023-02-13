#include <chrono>

#include <geometry_common/Utils.h>
#include <yaml_common/Parser2.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/structs/context_data.h>
#include <cabin_nav/mpc/optimiser.h>
#include <cabin_nav/mpc/model.h>
#include <cabin_nav/input/velocity_input_data.h>
#include <cabin_nav/input/localisation_input_data.h>
#include <cabin_nav/input/laser_input_data.h>
#include <cabin_nav/output/cmd_vel_output_data.h>
#include <cabin_nav/output/visualization_marker_output_data.h>
#include <cabin_nav/action/goto_action.h>
#include <cabin_nav/behavior/corridor_behavior.h>

using kelo::geometry_common::Point2D;
using kelo::geometry_common::Vector2D;
using kelo::geometry_common::Circle;
using kelo::geometry_common::Pose2D;
using kelo::geometry_common::Velocity2D;
using kelo::geometry_common::Polygon2D;
using kelo::geometry_common::PointCloud2D;
using kelo::geometry_common::LineSegment2D;
using kelo::geometry_common::TransformMatrix2D;
using GCUtils = kelo::geometry_common::Utils;
using Parser = kelo::yaml_common::Parser2;

namespace cabin {

bool CorridorBehavior::configure(const YAML::Node& config)
{
    if ( !parseControlHorizon(config) ||
         !parseAccLimits(config) ||
         !parseVelLimits(config) ||
         !parseAccLimitWeights(config) ||
         !parseVelLimitWeights(config) ||
         !parseGoalStateWeights(config) ||
         !parseFootprint(config) ||
         !parseRequiredInputs(config) ||
         !parseOutputs(config) )
    {
        return false;
    }
    const YAML::Node& control_config = config["control"];
    weight_ideal_path_perp_dist_ = Parser::get<float>(
            control_config["weights"], "ideal_path_perp_dist", 0.0f);
    weight_left_wall_repulsion_ = Parser::get<float>(
            control_config["weights"], "left_wall_repulsion", 0.0f);
    weight_right_wall_repulsion_ = Parser::get<float>(
            control_config["weights"], "right_wall_repulsion", 0.0f);
    weight_remaining_laser_pts_repulsion_ = Parser::get<float>(
            control_config["weights"], "remaining_laser_pts_repulsion", 0.0f);
    weight_linear_vel_ellipse_ = Parser::get<float>(
            control_config["weights"], "linear_vel_ellipse", 0.0f);
    inflation_dist_ = Parser::get<float>(
            control_config, "inflation_dist", 0.2f);

    if ( is_footprint_box_ )
    {
        /* treat robot footprint as a line segment */
        const Point2D box_center = box_footprint_.center();
        if ( box_footprint_.sizeX() >= box_footprint_.sizeY() )
        {
            robot_line_ = LineSegment2D(box_footprint_.max_x, box_center.y,
                                        box_footprint_.min_x, box_center.y);
            wall_inflation_dist_ = (box_footprint_.sizeY()/2) + inflation_dist_;
        }
        else
        {
            robot_line_ = LineSegment2D(box_center.x, box_footprint_.max_y,
                                        box_center.x, box_footprint_.min_y);
            wall_inflation_dist_ = (box_footprint_.sizeX()/2) + inflation_dist_;
        }
    }

    if ( Parser::hasKey(config, "perception") )
    {
        lane_percentage_ = Parser::get<float>(
                config["perception"], "lane_percentage", 0.5f);
        ideal_path_length_ = Parser::get<float>(
                config["perception"], "ideal_path_length", 5.0f);
        line_fitting_ransac_delta_ = Parser::get<float>(
                config["perception"], "line_fitting_ransac_delta", 0.1f);
    }

    if ( Parser::hasKey(config, "transition_condition") )
    {
        if ( Parser::hasKey(config["transition_condition"], "pre") )
        {
            goal_tolerance_heading_pre_ = Parser::get<float>(
                    config["transition_condition"]["pre"],
                    "goal_tolerance_heading", 0.8f);
        }
        if ( Parser::hasKey(config["transition_condition"], "post") )
        {
            goal_tolerance_heading_post_ = Parser::get<float>(
                    config["transition_condition"]["post"],
                    "goal_tolerance_heading", 0.8f);
            goal_tolerance_linear_ = Parser::get<float>(
                    config["transition_condition"]["post"],
                    "goal_tolerance_linear", 3.0f);
        }
    }

    if ( Parser::hasKey(config, "recovery") )
    {
        retry_threshold_ = Parser::get<size_t>(config["recovery"], "retry_threshold", 0);
        parseFailureToRecoveryMap(config);
    }

    cf_ = std::bind(&CorridorBehavior::calcCost, this, std::placeholders::_1);

    if ( Parser::hasKey(control_config, "initial_guess") )
    {
        initial_guess_utils_.configure(control_config["initial_guess"],
                max_vel_, min_vel_, max_acc_, sample_times_);
        initial_guess_utils_.setCostFunction(cf_);
        use_initial_guess_utils_ = true;
    }
    else
    {
        use_initial_guess_utils_ = false;
    }
    goal_.vel.x = max_vel_.x;
    u_ = std::vector<float>(3 * sample_times_.size(), 0.0f);
    return true;
}

void CorridorBehavior::reset()
{
    left_wall_ = LineSegment2D();
    right_wall_ = LineSegment2D();
    ideal_path_ = LineSegment2D();
    laser_pts_.clear();
    remaining_pts_.clear();
    std::fill(u_.begin(), u_.end(), 0.0f);
    initial_guess_trajectory_.clear();
    initial_guess_utils_.reset();
    retry_counter_ = 0;
    current_.vel = Velocity2D();
}

void CorridorBehavior::runOnce(const ContextData& context_data, BehaviorFeedback& fb)
{
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

    VisualizationMarkerOutputData::addMarker(fb.output_data_map,
            outputs_map_.at("markers"), behavior_name_marker_);
    VisualizationMarkerOutputData::addMarkers(fb.output_data_map,
            outputs_map_.at("markers"), context_data.getPlanMarkers());
    VisualizationMarkerOutputData::addMarker(fb.output_data_map,
            outputs_map_.at("markers"), footprint_marker_);

    context_data_ = &context_data;

    if ( !VelocityInputData::getVelocity(context_data.input_data_map,
             required_inputs_map_.at("velocity"), current_.vel) ||
         !LaserInputData::getLaserPts(context_data.input_data_map,
             required_inputs_map_.at("laser"), laser_pts_) )
    {
        std::cout << Print::Err << Print::Time() << "[CorridorBehavior] "
                  << "Could not get velocity and/or laser."
                  << Print::End << std::endl;
        CmdVelOutputData::setTrajectory(
                fb.output_data_map, calcRampedTrajectory(current_.vel));
        fb.success = false;
        return;
    }

    if ( !perceive(fb) )
    {
        return;
    }

    calcInitialU(u_, fb);

    std::chrono::duration<float> init_u_time_duration = std::chrono::steady_clock::now() - start_time;
    float remaining_time = sample_times_.front() - init_u_time_duration.count();

    float time_threshold = remaining_time * 0.8f;

    Optimiser::conjugateGradientDescent(time_threshold, cf_, u_);
    Trajectory traj = Model::calcTrajectory(current_, u_, sample_times_);
    CmdVelOutputData::setTrajectory(fb.output_data_map, traj);
    std::chrono::duration<float> time_taken = std::chrono::steady_clock::now() - start_time;
    // std::cout << time_taken.count() * 1000.0f << " ms" << std::endl;

    /* visualize features using markers */
    VisualizationMarkerOutputData::addMarker(fb.output_data_map,
            outputs_map_.at("markers"), left_wall_.asMarker(
                "robot", 1.0f, 0.0, 0.0f, 0.8f, 0.05f));
    VisualizationMarkerOutputData::addMarker(fb.output_data_map,
            outputs_map_.at("markers"), right_wall_.asMarker(
                "robot", 1.0f, 0.0, 0.0f, 0.8f, 0.05f));
    VisualizationMarkerOutputData::addMarker(fb.output_data_map,
            outputs_map_.at("markers"), ideal_path_.asMarker(
                "robot", 0.0f, 0.33f, 0.0f, 1.0f, 0.05f));

    retry_counter_ = 0;
    fb.success = true;
}

void CorridorBehavior::calcInitialU(std::vector<float>& u, BehaviorFeedback& fb)
{
    if ( use_initial_guess_utils_ )
    {
        initial_guess_utils_.calcInitialGuessIntermittently(
                current_, u, initial_guess_trajectory_);
        addInitialGuessTrajectoryMarkers(
                initial_guess_trajectory_, fb.output_data_map);
    }
    else
    {
        /* apply no control */
        std::fill(u.begin(), u.end(), 0.0f);
    }
}

float CorridorBehavior::calcCost(const std::vector<float>& u)
{
    // generate trajectory using model
    Trajectory trajectory = Model::calcTrajectory(
            current_, u, sample_times_);

    float cost = 0.0f;

    /* acc limit soft constraints */
    cost += calcAccLimitCost(u);

    for ( const TrajectoryPoint& tp : trajectory )
    {
        /* CONSTRAINTS */
        cost += weight_left_wall_repulsion_ * calcWallCost(left_wall_, tp);
        cost += weight_right_wall_repulsion_ * calcWallCost(right_wall_, tp);
        cost += weight_remaining_laser_pts_repulsion_ *
                calcLaserPtsCost(remaining_pts_, tp.pos, inflation_dist_);

        /* vel limit soft constraints */
        cost += calcVelLimitCost(tp.vel);

        float linear_vel_constraint = GCUtils::clip(
                std::pow(tp.vel.x/max_vel_.x, 2) + std::pow(tp.vel.y/(2*max_vel_.y), 2),
                2.0f, 1.0f) - 1.0f;
        cost += weight_linear_vel_ellipse_ * std::pow(linear_vel_constraint, 2);

        cost += weight_ideal_path_perp_dist_ * ideal_path_.squaredMinDistTo(tp.pos.position());
        cost += calcGoalCost(tp, goal_);
    }

    return cost;
}

float CorridorBehavior::calcWallCost(
        const LineSegment2D& wall,
        const TrajectoryPoint& tp)
{
    return ( is_footprint_box_ )
           ? calcWallCostBoxFootprint(wall, tp)
           : calcWallCostCircleFootprint(wall, tp);
}

float CorridorBehavior::calcWallCostBoxFootprint(
        const LineSegment2D& wall,
        const TrajectoryPoint& tp)
{
    const LineSegment2D transformed_robot_line = TransformMatrix2D(tp.pos) * robot_line_;

    float cost = 0.0f;
    const Point2D robot_pt;
    const bool robot_determinant_sign = std::signbit(Utils::calcDeterminant(
                wall, robot_pt));

    /* calc cost for start of line segment */
    const float start_dist = wall.minDistTo(transformed_robot_line.start);
    const bool start_determinant_sign = std::signbit(Utils::calcDeterminant(
                wall, transformed_robot_line.start));
    if ( start_determinant_sign != robot_determinant_sign )
    {
        cost += std::pow((wall_inflation_dist_ + start_dist) / inflation_dist_, 2);
    }
    else if ( start_dist < wall_inflation_dist_ )
    {
        cost += std::pow((wall_inflation_dist_ - start_dist) / inflation_dist_, 2);
    }

    /* calc cost for end of line segment */
    const float end_dist = wall.minDistTo(transformed_robot_line.end);
    const bool end_determinant_sign = std::signbit(Utils::calcDeterminant(
                wall, transformed_robot_line.end));
    if ( end_determinant_sign != robot_determinant_sign )
    {
        cost += std::pow((wall_inflation_dist_ + end_dist) / inflation_dist_, 2);
    }
    else if ( end_dist < wall_inflation_dist_ )
    {
        cost += std::pow((wall_inflation_dist_ - end_dist) / inflation_dist_, 2);
    }
    return cost;
}

float CorridorBehavior::calcWallCostCircleFootprint(
        const LineSegment2D& wall,
        const TrajectoryPoint& tp)
{
    Circle transformed_circle = TransformMatrix2D(tp.pos) * circle_footprint_;

    float cost = 0.0f;
    Point2D robot_pt;
    int robot_determinant_sign = ( Utils::calcDeterminant(wall, robot_pt) >= 0 ) ? 1 : -1;
    float safety_radius = transformed_circle.r + inflation_dist_;
    float dist = wall.minDistTo(transformed_circle);
    int determinant_sign = ( Utils::calcDeterminant(wall, transformed_circle) >= 0 ) ? 1 : -1;
    if ( determinant_sign != robot_determinant_sign )
    {
        cost += std::pow(safety_radius/inflation_dist_, 2);
        cost += std::pow(dist, 2);
    }
    else if ( dist < safety_radius )
    {
        cost += std::pow((safety_radius-dist)/(inflation_dist_), 2);
    }
    return cost;
}

bool CorridorBehavior::preConditionSatisfied(
        const ContextData& context_data, Action::Ptr action) const
{
    if ( !context_data.isPlanValid() ||
         context_data.plan[context_data.plan_index]->getType() != "goto" )
    {
        return false;
    }

    TransformMatrix2D loc_tf;
    if ( !LocalisationInputData::getLocalisationTF(context_data.input_data_map,
                required_inputs_map_.at("localisation"), loc_tf) )
    {
        std::cout << Print::Err << Print::Time() << "[CorridorBehavior] "
                  << "Could not get localisation tf"
                  << Print::End << std::endl;
        return false;
    }

    GoToAction::Ptr goto_action = std::static_pointer_cast<GoToAction>(action);
    Point2D intermediate_goal_pt;

    const std::vector<Area::ConstPtr>& goto_plan = goto_action->getGoToPlan();
    size_t goto_plan_index = goto_action->getGoToPlanIndex();
    const Area::ConstPtr& corridor_area = goto_plan[goto_plan_index];
    const Pose2D robot_pose = loc_tf.asPose2D();
    const Point2D robot_pt = robot_pose.position();

    if ( goto_plan.size() - goto_plan_index == 1 ) // last area
    {
        intermediate_goal_pt = goto_action->getGoal().position();
    }
    else
    {
        const Area::ConstPtr& next_area = goto_plan[goto_plan_index + 1];
        corridor_area->getPolygon().calcClosestIntersectionPointWith(
                LineSegment2D(corridor_area->getCenter(), next_area->getCenter()),
                intermediate_goal_pt);
    }

    const Polygon2D& corridor_polygon = corridor_area->getPolygon();
    LineSegment2D corridor_mid_line, projected_line_segment;
    GCUtils::fitLineRegression(corridor_polygon.vertices, corridor_mid_line);
    projected_line_segment.start = GCUtils::calcProjectedPointOnLine(
            corridor_mid_line.start, corridor_mid_line.end, robot_pt, false);
    projected_line_segment.end = GCUtils::calcProjectedPointOnLine(
            corridor_mid_line.start, corridor_mid_line.end, intermediate_goal_pt, false);

    float dist_to_goal = projected_line_segment.length();
    float angle_to_goal = projected_line_segment.angle();
    float angle_diff = GCUtils::calcShortestAngle(
            angle_to_goal, robot_pose.theta);

    bool inside_polygon = corridor_polygon.containsPoint(robot_pt);

    if ( !inside_polygon )
    {
        LineSegment2D l(robot_pt, corridor_area->getCenter());
        Point2D intersection_pt;
        corridor_polygon.calcClosestIntersectionPointWith(l, intersection_pt);
        TransformMatrix2D tf(1.0f, 0.0f, 0.0f);
        Pose2D intersection_pose(intersection_pt, l.angle());
        goto_action->setIntermediateGoal(TrajectoryPoint(
                    (TransformMatrix2D(intersection_pose) * tf).asPose2D()));
        goto_action->enableIsDuringTransition();
        return false;
    }
    else if ( std::fabs(angle_diff) > goal_tolerance_heading_pre_ &&
              dist_to_goal > goal_tolerance_linear_ )
    {
        /* approx corridor width */
        float sum_dist = 0.0f;
        for ( size_t i = 0; i < corridor_polygon.size(); i++ )
        {
            sum_dist += corridor_mid_line.minDistTo(corridor_polygon[i]);
        }
        float corridor_width = 2.0f * (sum_dist / corridor_polygon.size());

        float shift_dist = (0.5f-lane_percentage_) * corridor_width;
        Vector2D mid_unit_vec = projected_line_segment.unitVector();
        Vector2D mid_perp_unit_vec(-mid_unit_vec.y, mid_unit_vec.x);
        LineSegment2D shifted_line(
                projected_line_segment.start + (mid_perp_unit_vec * shift_dist),
                projected_line_segment.end + (mid_perp_unit_vec * shift_dist));
        Point2D projected_robot_pt = shifted_line.closestPointTo(robot_pt);
        float perp_dist = projected_robot_pt.distTo(robot_pt);
        Point2D goal_pt = projected_robot_pt + (mid_unit_vec * perp_dist);
        goto_action->setIntermediateGoal(TrajectoryPoint(Pose2D(goal_pt, angle_to_goal)));
        goto_action->enableIsDuringTransition();
        return false;
    }
    else
    {
        return true;
    }
}

bool CorridorBehavior::postConditionSatisfied(const ContextData& context_data) const
{
    if ( !context_data.isPlanValid() ||
         context_data.plan[context_data.plan_index]->getType() != "goto" )
    {
        return true;
    }

    TransformMatrix2D loc_tf;
    if ( !LocalisationInputData::getLocalisationTF(context_data.input_data_map,
                required_inputs_map_.at("localisation"), loc_tf) )
    {
        std::cout << Print::Err << Print::Time() << "[CorridorBehavior] "
                  << "Could not get localisation tf"
                  << Print::End << std::endl;
        return false;
    }

    const GoToAction::Ptr goto_action = std::static_pointer_cast<GoToAction>(
             context_data.plan[context_data.plan_index]);
    Point2D intermediate_goal_pt;

    const std::vector<Area::ConstPtr>& goto_plan = goto_action->getGoToPlan();
    size_t goto_plan_index = goto_action->getGoToPlanIndex();
    const Area::ConstPtr& corridor_area = goto_plan[goto_plan_index];
    const Pose2D robot_pose = loc_tf.asPose2D();
    const Point2D robot_pt = robot_pose.position();

    if ( goto_plan.size() - goto_plan_index == 1 ) // last area
    {
        intermediate_goal_pt = goto_action->getGoal().position();
    }
    else
    {
        const Area::ConstPtr& next_area = goto_plan[goto_plan_index + 1];
        corridor_area->getPolygon().calcClosestIntersectionPointWith(
                LineSegment2D(corridor_area->getCenter(), next_area->getCenter()),
                intermediate_goal_pt);
    }

    LineSegment2D corridor_mid_line, projected_line_segment;
    GCUtils::fitLineRegression(corridor_area->getPolygon().vertices, corridor_mid_line);
    projected_line_segment.start = GCUtils::calcProjectedPointOnLine(
            corridor_mid_line.start, corridor_mid_line.end, robot_pt, false);
    projected_line_segment.end = GCUtils::calcProjectedPointOnLine(
            corridor_mid_line.start, corridor_mid_line.end, intermediate_goal_pt, false);

    float dist_to_goal = projected_line_segment.length();
    float angle_to_goal = projected_line_segment.angle();
    float angle_diff = GCUtils::calcShortestAngle(
            angle_to_goal, robot_pose.theta);

    bool inside_polygon = corridor_area->getPolygon().containsPoint(robot_pt);

    if ( dist_to_goal < goal_tolerance_linear_ ||
         std::fabs(angle_diff) > goal_tolerance_heading_post_ ||
         !inside_polygon)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool CorridorBehavior::findWalls(
        LineSegment2D& left_wall,
        LineSegment2D& right_wall,
        PointCloud2D& remaining_pts)
{
    remaining_pts.clear();
    PointCloud2D left_pts, right_pts;
    left_pts.reserve(laser_pts_.size()/2);
    right_pts.reserve(laser_pts_.size()/2);
    for ( const Point2D& pt : laser_pts_ )
    {
        // if ( laser_pts_[i].x < 0 )
        // {
        //     remaining_pts_.push_back(laser_pts_[i]);
        // }
        if ( pt.y > 0 )
        {
            left_pts.push_back(pt);
        }
        if ( pt.y < 0 )
        {
            right_pts.push_back(pt);
        }
    }

    if ( left_pts.size() < 2 || right_pts.size() < 2 )
    {
        left_wall = LineSegment2D();
        right_wall = LineSegment2D();
        return false;
    }

    GCUtils::fitLineSegmentRANSAC(left_pts, left_wall, line_fitting_ransac_delta_);
    GCUtils::fitLineSegmentRANSAC(right_pts, right_wall, line_fitting_ransac_delta_);

    float delta_sq = std::pow(line_fitting_ransac_delta_, 2);
    for ( const Point2D& pt : left_pts )
    {
        if ( left_wall.squaredMinDistTo(pt) > delta_sq )
        {
            remaining_pts.push_back(pt);
        }
    }
    for ( const Point2D& pt : right_pts )
    {
        if ( right_wall.squaredMinDistTo(pt) > delta_sq )
        {
            remaining_pts.push_back(pt);
        }
    }

    return true;
}

void CorridorBehavior::failWithRetry(
        BehaviorFeedback& fb,
        const std::string& failure_code)
{
    CmdVelOutputData::setTrajectory(
            fb.output_data_map, calcRampedTrajectory(current_.vel));
    retry_counter_ ++;
    if ( retry_counter_ > retry_threshold_ )
    {
        fb.success = false;
        fb.failure_code = failure_code;
    }
    else
    {
        fb.success = true;
    }
}

bool CorridorBehavior::perceive(BehaviorFeedback& fb)
{
    if ( !findWalls(left_wall_, right_wall_, remaining_pts_) )
    {
        std::cerr << Print::Warn << Print::Time() << "[CorridorBehavior] "
                  << "Could not find walls"
                  << Print::End << std::endl;
        failWithRetry(fb, "perception_failure");
        return false;
    }

    if ( std::fabs(GCUtils::calcShortestAngle(left_wall_.angle(), right_wall_.angle())) > 1.0f )
    {
        std::cerr << Print::Warn << Print::Time() << "[CorridorBehavior] "
                  << "Left and right wall do not have similar slope"
                  << Print::End << std::endl;
        failWithRetry(fb, "perception_failure");
        return false;
    }

    if ( left_wall_.start.y < 0 || right_wall_.start.y > 0 )
    {
        std::cerr << Print::Warn << Print::Time() << "[CorridorBehavior] "
                  << "Either of the wall is not on the correct side"
                  << Print::End << std::endl;
        failWithRetry(fb, "perception_failure");
        return false;
    }

    /* calculate ideal path based on left and right walls */
    float ideal_path_m = (left_wall_.slope() + right_wall_.slope()) / 2;
    float ideal_path_c = GCUtils::applyLinearInterpolation(
            left_wall_.constant(), right_wall_.constant(), lane_percentage_);
    ideal_path_.start.x = 0.0f;
    ideal_path_.start.y = (ideal_path_m * ideal_path_.start.x) + ideal_path_c;
    ideal_path_.end.x = ideal_path_length_;
    ideal_path_.end.y = (ideal_path_m * ideal_path_.end.x) + ideal_path_c;
    goal_.pos.theta = ideal_path_.angle();
    return true;
}

} // namespace cabin
