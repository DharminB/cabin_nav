#include <chrono>

#include <geometry_common/Utils.h>
#include <yaml_common/Parser2.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/structs/context_data.h>
#include <cabin_nav/mpc/optimiser.h>
#include <cabin_nav/mpc/model.h>
#include <cabin_nav/utils/geometric_planner.h>
#include <cabin_nav/action/goto_action.h>
#include <cabin_nav/behavior/junction_behavior.h>

using kelo::geometry_common::Point2D;
using kelo::geometry_common::Vector2D;
using kelo::geometry_common::PointVec2D;
using kelo::geometry_common::Pose2D;
using kelo::geometry_common::Path;
using kelo::geometry_common::Velocity2D;
using kelo::geometry_common::LineSegment2D;
using kelo::geometry_common::TransformMatrix2D;
using GCUtils = kelo::geometry_common::Utils;
using Parser = kelo::yaml_common::Parser2;

namespace cabin {

bool JunctionBehavior::configure(const YAML::Node& config)
{
    if ( !parseControlHorizon(config) ||
         !parseAccLimits(config) ||
         !parseVelLimits(config) ||
         !parseAccLimitWeights(config) ||
         !parseVelLimitWeights(config) ||
         !parseGoalStateWeights(config) ||
         !parseFootprint(config) ||
         !parseRequiredInputs(config) )
    {
        return false;
    }
    const YAML::Node& control_config = config["control"];
    is_unicycle_ = Parser::get<bool>(
            control_config, "is_unicycle", false);
    weight_laser_pts_repulsion_ = Parser::get<float>(
            control_config["weights"], "laser_pts_repulsion", 0.0f);
    weight_goal_state_intercept_angle_ = Parser::get<float>(
            control_config["weights"], "goal_state_intercept_angle", 0.0f);

    ideal_path_step_size_ = Parser::get<float>(
            control_config, "ideal_path_step_size", 0.1f);
    lane_percentage_ = Parser::get<float>(
            control_config, "lane_percentage", 0.6f);
    pre_spline_control_pt_dist_ = Parser::get<float>(
            control_config, "pre_spline_control_pt_dist", 1.0f);
    post_spline_control_pt_dist_ = Parser::get<float>(
            control_config, "post_spline_control_pt_dist", 1.0f);
    inflation_dist_ = Parser::get<float>(
            control_config, "inflation_dist", 0.2f);

    if ( Parser::hasKey(config, "transition_condition") )
    {
        if ( Parser::hasKey(config["transition_condition"], "pre") )
        {
            goal_tolerance_linear_pre_ = Parser::get<float>(
                    config["transition_condition"]["pre"],
                    "goal_tolerance_linear", 1.0f);
            goal_tolerance_angular_pre_ = Parser::get<float>(
                    config["transition_condition"]["pre"],
                    "goal_tolerance_angular", 0.8f);
        }
        if ( Parser::hasKey(config["transition_condition"], "post") )
        {
            goal_tolerance_linear_post_ = Parser::get<float>(
                    config["transition_condition"]["post"],
                    "goal_tolerance_linear", 1.0f);
            goal_tolerance_angular_post_ = Parser::get<float>(
                    config["transition_condition"]["post"],
                    "goal_tolerance_angular", 0.8f);
        }
    }

    parseFailureToRecoveryMap(config);

    if ( is_unicycle_ )
    {
        max_vel_.y = 0.0f;
        min_vel_.y = 0.0f;
    }

    cf_ = std::bind(&JunctionBehavior::calcCost, this, std::placeholders::_1);

    if ( Parser::hasKey(control_config, "initial_guess") )
    {
        initial_guess_utils_.configure(control_config["initial_guess"],
                max_vel_, min_vel_, max_acc_, sample_times_, is_unicycle_);
        initial_guess_utils_.setCostFunction(cf_);
        use_initial_guess_utils_ = true;
    }
    else
    {
        use_initial_guess_utils_ = false;
    }

    goal_.vel.x = max_vel_.x;

    u_ = ( is_unicycle_ )
         ? std::vector<float>(2 * sample_times_.size(), 0.0f)
         : std::vector<float>(3 * sample_times_.size(), 0.0f);

    float control_horizon_time = 0.0f;
    for ( float sample_time : sample_times_ )
    {
        control_horizon_time += sample_time;
    }
    goal_dist_from_robot_ = max_vel_.x * control_horizon_time;
    return true;
}

void JunctionBehavior::reset()
{
    ideal_path_.clear();
    std::fill(u_.begin(), u_.end(), 0.0f);
    initial_guess_trajectory_.clear();
    initial_guess_utils_.reset();
    current_.vel = Velocity2D();
    goal_ = TrajectoryPoint();
}

void JunctionBehavior::runOnce(const ContextData& context_data, BehaviorFeedback& fb)
{
    fb.output_data->markers.push_back(behavior_name_marker_);
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

    context_data_ = &context_data;
    current_.vel = context_data_->input_data->current_vel;

    ideal_path_ = calcIdealPath(context_data);
    goal_.pos = calcGoal();

    calcInitialU(u_, fb);

    std::chrono::duration<float> init_u_time_duration = std::chrono::steady_clock::now() - start_time;
    float remaining_time = sample_times_.front() - init_u_time_duration.count();

    float time_threshold = remaining_time * 0.8f;

    Optimiser::conjugateGradientDescent(time_threshold, cf_, u_);
    fb.output_data->trajectory = Model::calcTrajectory(
            current_, u_, sample_times_, is_unicycle_);
    std::chrono::duration<float> time_taken = std::chrono::steady_clock::now() - start_time;
    // std::cout << time_taken.count() * 1000.0f << " ms" << std::endl;

    /* visualize features using markers */
    fb.output_data->markers.push_back(GCUtils::convertGeometricPathToMarker(
                ideal_path_, "robot", 0.0f, 0.33f, 0.0f, 1.0f, 0.02f));
    fb.output_data->markers.push_back(goal_.pos.asMarker("robot", 0.0f, 1.0f, 0.0f));
    std::vector<visualization_msgs::Marker> plan_markers = context_data.getPlanMarkers();
    fb.output_data->markers.insert(fb.output_data->markers.end(),
            plan_markers.begin(), plan_markers.end());
    fb.output_data->markers.push_back(footprint_marker_);

    fb.success = true;
}

void JunctionBehavior::calcInitialU(std::vector<float>& u, BehaviorFeedback& fb)
{
    if ( use_initial_guess_utils_ )
    {
        initial_guess_utils_.calcInitialGuessIntermittently(
                current_, u, initial_guess_trajectory_);
        addInitialGuessTrajectoryMarkers(
                initial_guess_trajectory_, fb.output_data->markers);
    }
    else
    {
        /* apply no control */
        std::fill(u.begin(), u.end(), 0.0f);
    }
}

float JunctionBehavior::calcCost(const std::vector<float>& u)
{
    // generate trajectory using model
    Trajectory trajectory = Model::calcTrajectory(
            current_, u, sample_times_, is_unicycle_);

    float cost = 0.0f;

    /* acc limit soft constraints */
    cost += calcAccLimitCost(u, is_unicycle_);

    for ( const TrajectoryPoint& tp : trajectory )
    {
        /* CONSTRAINTS */
        cost += weight_laser_pts_repulsion_ *
                calcLaserPtsCost(
                        context_data_->input_data->laser_pts, tp.pos, inflation_dist_);

        /* vel limit soft constraints */
        cost += calcVelLimitCost(tp.vel);
    }
    /* GOAL */
    cost += calcGoalCost(trajectory.back(), goal_);

    float angle_to_goal = (goal_.pos.position() - trajectory.back().pos.position()).angle();
    float angular_dist = GCUtils::calcShortestAngle(angle_to_goal, goal_.pos.theta);
    cost += weight_goal_state_intercept_angle_ * std::pow(angular_dist, 2);

    return cost;
}

bool JunctionBehavior::preConditionSatisfied(
        const ContextData& context_data, Action::Ptr action) const
{
    if ( !context_data.isPlanValid() ||
         context_data.plan[context_data.plan_index]->getType() != "goto" )
    {
        return false;
    }

    GoToAction::Ptr goto_action = std::static_pointer_cast<GoToAction>(action);

    size_t goto_plan_index = goto_action->getGoToPlanIndex();
    const std::vector<Area::ConstPtr>& goto_plan = goto_action->getGoToPlan();

    /* if junction is beginning or end of plan */
    if ( goto_plan_index == 0 ||
         goto_plan.size() - goto_plan_index == 1 )
    {
        return true;
    }
    Path ideal_path = calcIdealPath(context_data);
    Pose2D ideal_path_start = context_data.input_data->localisation_tf * ideal_path.front();

    const Pose2D robot_pose = context_data.input_data->localisation_tf.asPose2D();

    if ( !Utils::isWithinTolerance(robot_pose, ideal_path_start,
                goal_tolerance_linear_pre_, goal_tolerance_angular_pre_) ||
         context_data.input_data->current_vel.x > max_vel_.x )
    {
        goto_action->setIntermediateGoal(
                TrajectoryPoint(ideal_path_start, Velocity2D(max_vel_.x)));
        goto_action->enableIsDuringTransition();
        return false;
    }
    else
    {
        return true;
    }
}

bool JunctionBehavior::postConditionSatisfied(const ContextData& context_data) const
{
    if ( !context_data.isPlanValid() ||
         context_data.plan[context_data.plan_index]->getType() != "goto" )
    {
        return false;
    }

    const GoToAction::Ptr goto_action = std::static_pointer_cast<GoToAction>(
            context_data.plan[context_data.plan_index]);

    size_t goto_plan_index = goto_action->getGoToPlanIndex();
    const std::vector<Area::ConstPtr>& goto_plan = goto_action->getGoToPlan();

    /* if junction is beginning or end of plan */
    if ( goto_plan_index == 0 ||
         goto_plan.size() - goto_plan_index == 1 )
    {
        return true;
    }

    Path ideal_path = calcIdealPath(context_data);
    Pose2D ideal_path_end = context_data.input_data->localisation_tf * ideal_path.back();

    const Pose2D robot_pose = context_data.input_data->localisation_tf.asPose2D();

    if ( !Utils::isWithinTolerance(robot_pose, ideal_path_end,
                goal_tolerance_linear_post_, goal_tolerance_angular_post_) )
    {
        return false;
    }
    else
    {
        return true;
    }
}

Pose2D JunctionBehavior::calcGoal()
{
    /* calculate goal_ from ideal_path_ */
    float min_dist = std::numeric_limits<float>::max();
    size_t closest_pose_index = 0;
    Pose2D current;
    for ( size_t i = 0; i < ideal_path_.size(); i++ )
    {
        float dist = current.distTo(ideal_path_[i]);
        if ( dist < min_dist )
        {
            min_dist = dist;
            closest_pose_index = i;
        }
    }
    float dist_offset = std::max(goal_dist_from_robot_ - min_dist, 0.0f);
    size_t index_offset = std::floor(dist_offset / ideal_path_step_size_);
    size_t goal_index = std::min(closest_pose_index + index_offset, ideal_path_.size()-1);
    return ideal_path_[goal_index];
}

Path JunctionBehavior::calcIdealPath(const ContextData& context_data) const
{
    Path ideal_path;
    const GoToAction::Ptr goto_action = std::static_pointer_cast<GoToAction>(
            context_data.plan[context_data.plan_index]);

    size_t goto_plan_index = goto_action->getGoToPlanIndex();
    const std::vector<Area::ConstPtr>& goto_plan = goto_action->getGoToPlan();
    const Area::ConstPtr& current_area = goto_plan[goto_plan_index];
    const Area::ConstPtr& next_area = goto_plan[goto_plan_index+1];
    const Area::ConstPtr& prev_area = goto_plan[goto_plan_index-1];

    /* calculate goal in local frame */
    TransformMatrix2D robot_to_global_tf = context_data.input_data->localisation_tf.calcInverse();

    PointVec2D control_points(3);
    LineSegment2D l(current_area->getCenter(), prev_area->getCenter());
    LineSegment2D entry_side = Utils::calcIntersectingSide(current_area->getPolygon(), l);
    LineSegment2D l2(current_area->getCenter(), next_area->getCenter());
    LineSegment2D exit_side = Utils::calcIntersectingSide(current_area->getPolygon(), l2);
    control_points[0] = entry_side.center();
    control_points[1] = current_area->getCenter();
    control_points[2] = exit_side.center();

    robot_to_global_tf.transform(control_points);

    /* straight line before start of junction */
    Vector2D diff_1_norm = (control_points[0] - control_points[1]).asNormalised();
    Point2D pre_spline_control_point = control_points[0] + (diff_1_norm * pre_spline_control_pt_dist_);
    PointVec2D pre_spline_points = GeometricPlanner::calcStraightLinePath(
            pre_spline_control_point, control_points[0], ideal_path_step_size_/2);

    /* spline curve in junction */
    PointVec2D spline_points = GCUtils::calcSplineCurvePoints(control_points, 100);

    /* straight line after end of junction */
    Vector2D diff_2_norm = (control_points[2] - control_points[1]).asNormalised();
    Point2D post_spline_control_point = control_points[2] + (diff_2_norm * post_spline_control_pt_dist_);
    PointVec2D post_spline_points = GeometricPlanner::calcStraightLinePath(
            control_points[2], post_spline_control_point, ideal_path_step_size_/2);

    /* merge all points into a single container */
    PointVec2D curve_points;
    curve_points.reserve((pre_spline_points.size()-1) +
                         spline_points.size() +
                         (post_spline_points.size()-1));
    curve_points.insert(curve_points.end(), pre_spline_points.begin(),
                        pre_spline_points.end()-1);
    curve_points.insert(curve_points.end(), spline_points.begin(),
                        spline_points.end());
    curve_points.insert(curve_points.end(), post_spline_points.begin()+1,
                        post_spline_points.end());

    /* shift curve points to right (for right hand drive) */
    Pose2D pose;
    TransformMatrix2D tf_mat;
    float entry_side_offset = entry_side.length() * (0.5f - lane_percentage_);
    float exit_side_offset = exit_side.length() * (0.5f - lane_percentage_);
    for ( size_t i = 0; i+1 < curve_points.size(); i++ )
    {
        float offset;
        if ( i < pre_spline_points.size() )
        {
            offset = entry_side_offset;
        }
        else if ( i > pre_spline_points.size()+spline_points.size() )
        {
            offset = exit_side_offset;
        }
        else
        {
            float t = (static_cast<float>(i) - pre_spline_points.size())/spline_points.size();
            offset = ((1-t) * entry_side_offset) + (t * exit_side_offset);
        }
        tf_mat.updateY(offset);
        Pose2D pose(curve_points[i], (curve_points[i+1] - curve_points[i]).angle());
        Pose2D transformed_pose(TransformMatrix2D(pose) * tf_mat);
        if ( ideal_path.size() == 0 ||
             transformed_pose.distTo(ideal_path.back()) > ideal_path_step_size_ )
        {
            ideal_path.push_back(transformed_pose);
        }
    }
    return ideal_path;
}

} // namespace cabin
