#include <chrono>

#include <geometry_common/Utils.h>
#include <yaml_common/Parser2.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/structs/context_data.h>
#include <cabin_nav/mpc/optimiser.h>
#include <cabin_nav/mpc/model.h>
#include <cabin_nav/utils/geometric_planner.h>
#include <cabin_nav/action/goto_action.h>
#include <cabin_nav/behavior/open_area_behavior.h>

using kelo::geometry_common::Point2D;
using kelo::geometry_common::PointVec2D;
using kelo::geometry_common::Pose2D;
using kelo::geometry_common::Path;
using kelo::geometry_common::Velocity2D;
using kelo::geometry_common::LineSegment2D;
using kelo::geometry_common::TransformMatrix2D;
using GCUtils = kelo::geometry_common::Utils;
using Parser = kelo::yaml_common::Parser2;

namespace cabin {

bool OpenAreaBehavior::configure(const YAML::Node& config)
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
    weight_laser_pts_repulsion_ = Parser::get<float>(
            control_config["weights"], "laser_pts_repulsion", 0.0f);
    weight_linear_vel_ellipse_ = Parser::get<float>(
            control_config["weights"], "linear_vel_ellipse", 0.0f);
    min_inflation_dist_ = Parser::get<float>(
            control_config, "min_inflation_dist", 0.1f);
    max_inflation_dist_ = Parser::get<float>(
            control_config, "max_inflation_dist", 10.0f);
    robot_to_goal_dist_ = Parser::get<float>(
            control_config, "robot_to_goal_dist", 5.0f);

    if ( Parser::hasKey(config, "transition_condition") )
    {
        if ( Parser::hasKey(config["transition_condition"], "pre") )
        {
            goal_tolerance_linear_pre_ = Parser::get<float>(
                    config["transition_condition"]["pre"],
                    "goal_tolerance_linear", 1.0f);
        }
        if ( Parser::hasKey(config["transition_condition"], "post") )
        {
            goal_tolerance_linear_post_open_area_ = Parser::get<float>(
                    config["transition_condition"]["post"],
                    "goal_tolerance_linear_open_area", 1.0f);
            goal_tolerance_linear_post_diff_area_ = Parser::get<float>(
                    config["transition_condition"]["post"],
                    "goal_tolerance_linear_diff_area", 2.0f);
            goal_tolerance_linear_post_final_goal_ = Parser::get<float>(
                    config["transition_condition"]["post"],
                    "goal_tolerance_linear_final_goal", 3.0f);
        }
    }

    parseFailureToRecoveryMap(config);

    cf_ = std::bind(&OpenAreaBehavior::calcCost, this, std::placeholders::_1);

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

    u_ = std::vector<float>(3 * sample_times_.size(), 0.0f);
    return true;
}

void OpenAreaBehavior::reset()
{
    geometric_plan_.clear();
    std::fill(u_.begin(), u_.end(), 0.0f);
    initial_guess_trajectory_.clear();
    initial_guess_utils_.reset();
    goal_ = TrajectoryPoint();
    current_.vel = Velocity2D();
}

void OpenAreaBehavior::runOnce(const ContextData& context_data, BehaviorFeedback& fb)
{
    fb.output_data->markers.push_back(behavior_name_marker_);
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

    context_data_ = &context_data;
    current_.vel = context_data_->input_data->current_vel;

    calcSpline();
    goal_.pos = calcGoal();

    calcInitialU(u_, fb);

    std::chrono::duration<float> init_u_time_duration = std::chrono::steady_clock::now() - start_time;
    float remaining_time = sample_times_.front() - init_u_time_duration.count();

    float time_threshold = remaining_time * 0.8f;

    Optimiser::conjugateGradientDescent(time_threshold, cf_, u_);
    fb.output_data->trajectory = Model::calcTrajectory(current_, u_, sample_times_);

    std::chrono::duration<float> time_taken = std::chrono::steady_clock::now() - start_time;
    // std::cout << time_taken.count() * 1000.0f << " ms" << std::endl;

    /* visualize features using markers */
    fb.output_data->markers.push_back(goal_.pos.asMarker("robot", 0.0f, 1.0f, 0.0f));
    fb.output_data->markers.push_back(GCUtils::convertGeometricPathToMarker(
                geometric_plan_, "robot", 0.0f, 0.33f, 0.0f, 1.0f, 0.02f));
    std::vector<visualization_msgs::Marker> plan_markers = context_data.getPlanMarkers();
    fb.output_data->markers.insert(fb.output_data->markers.end(),
            plan_markers.begin(), plan_markers.end());
    fb.output_data->markers.push_back(footprint_marker_);

    fb.success = true;
}

void OpenAreaBehavior::calcInitialU(std::vector<float>& u, BehaviorFeedback& fb)
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

float OpenAreaBehavior::calcCost(const std::vector<float>& u)
{
    // generate trajectory using model
    Trajectory trajectory = Model::calcTrajectory(current_, u, sample_times_);

    float cost = 0.0f;

    /* acc limit soft constraints */
    cost += calcAccLimitCost(u);

    for ( const TrajectoryPoint& tp : trajectory )
    {
        /* CONSTRAINTS */
        cost += weight_laser_pts_repulsion_ *
                calcLaserPtsCostVelBased(
                        context_data_->input_data->laser_pts, tp,
                        min_inflation_dist_, max_inflation_dist_);

        /* vel limit soft constraints */
        cost += calcVelLimitCost(tp.vel);

        float linear_vel_constraint = GCUtils::clip(
                std::pow(tp.vel.x/max_vel_.x, 2) + std::pow(tp.vel.y/(2*max_vel_.y), 2),
                2.0f,
                1.0f) - 1.0f;
        cost += weight_linear_vel_ellipse_ * std::pow(linear_vel_constraint, 2);
    }

    /* GOAL */
    cost += calcGoalCost(trajectory.back(), goal_);

   return cost;
}

bool OpenAreaBehavior::preConditionSatisfied(
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
    const Area::ConstPtr& current_area = goto_plan[goto_plan_index];
    const Point2D robot_pt = context_data.input_data->localisation_tf.asPose2D().position();

    bool inside_polygon = current_area->getPolygon().containsPoint(robot_pt);
    if ( !inside_polygon )
    {
        Point2D boundary_pt;
        current_area->getPolygon().calcClosestIntersectionPointWith(
                LineSegment2D(current_area->getCenter(), robot_pt),
                boundary_pt);
        float dist_to_boundary = robot_pt.distTo(boundary_pt);
        if ( dist_to_boundary > goal_tolerance_linear_pre_ )
        {
            float theta = (current_area->getCenter() - boundary_pt).angle();
            goto_action->setIntermediateGoal(TrajectoryPoint(Pose2D(boundary_pt, theta)));
            goto_action->enableIsDuringTransition();
            return false;
        }
    }

    return true;
}

bool OpenAreaBehavior::postConditionSatisfied(const ContextData& context_data) const
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
    const Area::ConstPtr& current_area = goto_plan[goto_plan_index];
    const Point2D robot_pt = context_data.input_data->localisation_tf.asPose2D().position();

    float dist_to_goal = goto_action->getGoal().distTo(robot_pt);
    if ( dist_to_goal < goal_tolerance_linear_post_final_goal_ )
    {
        return true;
    }

    if ( goto_plan.size() - goto_plan_index == 1 ) // last area
    {
        return false;
    }
    else
    {
        const Area::ConstPtr& next_area = goto_plan[goto_plan_index+1];
        if ( next_area->getType() != "open_area" )
        {
            Point2D intermediate_goal_pt;
            current_area->getPolygon().calcClosestIntersectionPointWith(
                    LineSegment2D(current_area->getCenter(), next_area->getCenter()),
                    intermediate_goal_pt);
            float dist_to_boundary = robot_pt.distTo(intermediate_goal_pt);
            return ( dist_to_boundary < goal_tolerance_linear_post_diff_area_ );
        }
        else
        {
            bool inside_polygon = current_area->getPolygon().containsPoint(robot_pt);
            if ( inside_polygon )
            {
                return false;
            }
            Point2D boundary_pt;
            current_area->getPolygon().calcClosestIntersectionPointWith(
                    LineSegment2D(current_area->getCenter(), robot_pt),
                    boundary_pt);
            float dist_to_boundary = robot_pt.distTo(boundary_pt);
            float angle_to_next_area = GCUtils::calcAngleBetweenPoints(
                    next_area->getCenter(), current_area->getCenter(), robot_pt);
            return ( dist_to_boundary > goal_tolerance_linear_post_open_area_ &&
                     std::fabs(angle_to_next_area) < M_PI/2 );
        }
    }
}

Pose2D OpenAreaBehavior::calcGoal()
{
    Point2D robot_pt;

    if ( geometric_plan_.empty() )
    {
        std::cerr << Print::Warn << Print::Time() << "[OpenAreaBehavior] "
                  << "geometric_plan is empty"
                  << Print::End << std::endl;
        return context_data_->input_data->localisation_tf.asPose2D();
    }

    /* calculate goal_ from geometric_plan_ */
    float min_dist = std::numeric_limits<float>::max();
    size_t closest_index = 0;
    for ( size_t i = 0; i < geometric_plan_.size(); i++ )
    {
        float dist = geometric_plan_[i].distTo(robot_pt);
        if ( dist < min_dist )
        {
            min_dist = dist;
            closest_index = i;
        }
    }
    // prune path from beginning to robot
    geometric_plan_.erase(geometric_plan_.begin(), geometric_plan_.begin()+closest_index);

    float dist_offset = std::max(robot_to_goal_dist_ - min_dist, 0.0f);
    int goal_index = GeometricPlanner::calcIndexAtDist(
                geometric_plan_, dist_offset);

    return geometric_plan_[goal_index];
}

void OpenAreaBehavior::calcSpline()
{
    Path geometric_plan;

    const GoToAction::Ptr goto_action = std::static_pointer_cast<GoToAction>(
            context_data_->plan[context_data_->plan_index]);

    size_t goto_plan_index = goto_action->getGoToPlanIndex();
    const std::vector<Area::ConstPtr>& goto_plan = goto_action->getGoToPlan();
    const Point2D robot_pt = context_data_->input_data->localisation_tf.asPose2D().position();

    TransformMatrix2D robot_to_global_tf = context_data_->input_data->localisation_tf.calcInverse();

    PointVec2D control_pts;
    for ( size_t i = goto_plan_index;
          i < goto_plan.size() &&
          goto_plan[i]->getType() == "open_area";
          i++ )
    {
        Point2D intermediate_goal_pt;
        if ( goto_plan.size() - i == 1 ) // last area
        {
            intermediate_goal_pt = goto_action->getGoal().position();
        }
        else if ( goto_plan[i+1]->getType() != "open_area" ) // next area is not an open_area
        {
            const Area::ConstPtr& current_area = goto_plan[i];
            const Area::ConstPtr& next_area = goto_plan[i+1];
            current_area->getPolygon().calcClosestIntersectionPointWith(
                    LineSegment2D(current_area->getCenter(), next_area->getCenter()),
                    intermediate_goal_pt);
        }
        else
        {
            intermediate_goal_pt = goto_plan[i]->getCenter();
        }
        control_pts.push_back(robot_to_global_tf * intermediate_goal_pt);
        if ( control_pts.size() >= 4 )
        {
            break;
        }
    }

    if ( control_pts.size() == 1 )
    {
        control_pts.insert(control_pts.begin(), Point2D());
    }

    PointVec2D geometric_plan_pts = GCUtils::calcSplineCurvePoints(control_pts, 100);
    geometric_plan_.clear();
    geometric_plan_.reserve(geometric_plan_pts.size()-1);
    for ( size_t i = 0; i+1 < geometric_plan_pts.size(); i++ )
    {
        float theta = (geometric_plan_pts[i+1] - geometric_plan_pts[i]).angle();
        geometric_plan_.push_back(Pose2D(geometric_plan_pts[i], theta));
    }
}

} // namespace cabin
