#include <chrono>

#include <geometry_common/Utils.h>
#include <yaml_common/Parser2.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/mpc/optimiser.h>
#include <cabin_nav/mpc/model.h>
#include <cabin_nav/input/velocity_input_data.h>
#include <cabin_nav/input/localisation_input_data.h>
#include <cabin_nav/input/laser_input_data.h>
#include <cabin_nav/output/cmd_vel_output_data.h>
#include <cabin_nav/output/visualization_marker_output_data.h>
#include <cabin_nav/structs/context_data.h>
#include <cabin_nav/action/goto_action.h>
#include <cabin_nav/behavior/recovery_ptp_behavior.h>

using kelo::geometry_common::Path;
using kelo::geometry_common::Point2D;
using kelo::geometry_common::Pose2D;
using kelo::geometry_common::LineSegment2D;
using kelo::geometry_common::TransformMatrix2D;
using GCUtils = kelo::geometry_common::Utils;
using Parser = kelo::yaml_common::Parser2;

namespace cabin {

bool RecoveryPTPBehavior::configure(const YAML::Node& config)
{
    if ( !PTPBehavior::configure(config) )
    {
        return false;
    }

    cf_ = std::bind(&RecoveryPTPBehavior::calcCost, this, std::placeholders::_1);

    max_goal_dist_ = Parser::get<float>(config["control"], "max_goal_dist", 2.0f);

    if ( Parser::hasKey(config, "transition_condition") )
    {
        if ( Parser::hasKey(config["transition_condition"], "post") )
        {
            goal_tolerance_angular_post_ = Parser::get<float>(
                    config["transition_condition"]["post"],
                    "goal_tolerance_angular", 0.5f);
            goal_tolerance_linear_post_ = Parser::get<float>(
                    config["transition_condition"]["post"],
                    "goal_tolerance_linear", 0.5f);
        }
    }

    recovery_time_threshold_ = Parser::get<float>(
            config["control"], "recovery_time_threshold", 5.0f);
    return true;
}

void RecoveryPTPBehavior::reset()
{
    PTPBehavior::reset();
    global_goal_ = TrajectoryPoint();
    is_global_goal_valid_ = false;
}

void RecoveryPTPBehavior::runOnce(const ContextData& context_data, BehaviorFeedback& fb)
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
         !LocalisationInputData::getLocalisationTF(context_data.input_data_map,
             required_inputs_map_.at("localisation"), localisation_tf_) ||
         !LaserInputData::getLaserPts(context_data.input_data_map,
             required_inputs_map_.at("velocity"), laser_pts_) )
    {
        std::cout << Print::Err << Print::Time() << "[PTPBehavior] "
                  << "Could not get velocity and/or localization tf."
                  << Print::End << std::endl;
        CmdVelOutputData::setTrajectory(
                fb.output_data_map, calcRampedTrajectory(current_.vel));
        fb.success = false;
        return;
    }

    goal_ = calcGoal(context_data, fb);

    std::chrono::duration<float> time_spent_in_recovery = std::chrono::steady_clock::now() - recovery_start_time_;
    if ( time_spent_in_recovery.count() > recovery_time_threshold_ )
    {
        std::cout << Print::Err << Print::Time() << "[RecoveryPTPBehavior] "
                  << "Timeout while trying to reach goal"
                  << Print::End << std::endl;
        CmdVelOutputData::setTrajectory(
                fb.output_data_map, calcRampedTrajectory(current_.vel));
        fb.success = false;
        return;
    }

    std::vector<visualization_msgs::Marker> markers;
    planGeometricPath(markers);
    VisualizationMarkerOutputData::addMarkers(fb.output_data_map,
            outputs_map_.at("markers"), markers);
    calcInitialU(u_, fb);

    std::chrono::duration<float> init_u_time_duration = std::chrono::steady_clock::now() - start_time;
    float remaining_time = sample_times_.front() - init_u_time_duration.count();

    float time_threshold = remaining_time * 0.8f;

    Optimiser::conjugateGradientDescent(time_threshold, cf_, u_);
    Trajectory traj = Model::calcTrajectory(current_, u_, sample_times_);
    CmdVelOutputData::setTrajectory(fb.output_data_map, traj);

    std::chrono::duration<float> time_taken = std::chrono::steady_clock::now() - start_time;
    // std::cout << time_taken.count() * 1000.0f << " ms" << std::endl;

    VisualizationMarkerOutputData::addMarker(fb.output_data_map,
            outputs_map_.at("markers"), goal_.pos.asMarker("robot", 0.0f, 1.0f, 0.0f));

    fb.success = true;
}

bool RecoveryPTPBehavior::preConditionSatisfied(
        const ContextData& context_data, Action::Ptr action) const
{
    return  ( context_data.isPlanValid() &&
              context_data.plan[context_data.plan_index]->getType() == "goto" &&
              std::static_pointer_cast<GoToAction>(
                  context_data.plan[context_data.plan_index])->isDuringTransition() );
}

bool RecoveryPTPBehavior::postConditionSatisfied(const ContextData& context_data) const
{
    if ( !is_global_goal_valid_ ) // should never be here
    {
        std::cerr << Print::Err << Print::Time() << "[RecoveryPTPBehavior][post condition] "
                  << "global_goal_ is not valid"
                  << Print::End << std::endl;
        return true;
    }

    TransformMatrix2D loc_tf;
    if ( !LocalisationInputData::getLocalisationTF(context_data.input_data_map,
                required_inputs_map_.at("localisation"), loc_tf) )
    {
        std::cout << Print::Err << Print::Time() << "[JunctionBehavior] "
                  << "Could not get localisation tf"
                  << Print::End << std::endl;
        return false;
    }

    const Pose2D robot_pose = loc_tf.asPose2D();
    return ( Utils::isWithinTolerance(
                robot_pose, global_goal_.pos,
                goal_tolerance_linear_post_, goal_tolerance_angular_post_) );
}

TrajectoryPoint RecoveryPTPBehavior::calcGoal(const ContextData& context_data, BehaviorFeedback& fb)
{
    if ( !is_global_goal_valid_ )
    {
        recovery_start_time_ = std::chrono::steady_clock::now();

        const GoToAction::Ptr goto_action = std::static_pointer_cast<GoToAction>(
                context_data.plan[context_data.plan_index]);
        Point2D intermediate_goal_pt;

        const std::vector<Area::ConstPtr>& goto_plan = goto_action->getGoToPlan();
        size_t goto_plan_index = goto_action->getGoToPlanIndex();
        const Point2D robot_pt = localisation_tf_.asPose2D().position();

        if ( goto_plan.size() - goto_plan_index == 1 ) // last area
        {
            intermediate_goal_pt = goto_action->getGoal().position();
        }
        else
        {
            const Area::ConstPtr& current_area = goto_plan[goto_plan_index];
            const Area::ConstPtr& next_area = goto_plan[goto_plan_index + 1];
            current_area->getPolygon().calcClosestIntersectionPointWith(
                    LineSegment2D(current_area->getCenter(), next_area->getCenter()),
                    intermediate_goal_pt);
        }

        float angle = (intermediate_goal_pt - robot_pt).angle();
        global_goal_.pos = Pose2D(
                robot_pt + Point2D::initFromRadialCoord(max_goal_dist_, angle),
                angle);
        is_global_goal_valid_ = true;
    }

    /* calculate goal in local frame */
    TransformMatrix2D tf = localisation_tf_.calcInverse();
    return tf * global_goal_;
}

} // namespace cabin
