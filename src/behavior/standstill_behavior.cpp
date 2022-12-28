#include <geometry_common/Utils.h>
#include <yaml_common/Parser2.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/structs/context_data.h>
#include <cabin_nav/action/goto_action.h>
#include <cabin_nav/behavior/standstill_behavior.h>

using kelo::geometry_common::Pose2D;
using GCUtils = kelo::geometry_common::Utils;
using Parser = kelo::yaml_common::Parser2;

namespace cabin {

bool StandstillBehavior::configure(const YAML::Node& config)
{
    return ( parseControlHorizon(config) &&
             parseAccLimits(config) &&
             parseRequiredInputs(config) &&
             Parser::hasKey(config, "transition_condition") &&
             Parser::hasKey(config["transition_condition"], "pre") &&
             Parser::read<float>(
                 config["transition_condition"]["pre"],
                 "goal_tolerance_angular", goal_tolerance_linear_) &&
             Parser::read<float>(
                 config["transition_condition"]["pre"],
                 "goal_tolerance_angular", goal_tolerance_angular_) );
}

void StandstillBehavior::reset()
{
}

void StandstillBehavior::runOnce(const ContextData& context_data, BehaviorFeedback& fb)
{
    fb.output_data->markers.push_back(behavior_name_marker_);
    fb.output_data->trajectory = calcRampedTrajectory(
            context_data.input_data->current_vel);

    std::vector<visualization_msgs::Marker> plan_markers = context_data.getPlanMarkers();
    fb.output_data->markers.insert(fb.output_data->markers.end(),
            plan_markers.begin(), plan_markers.end());

    fb.success = true;
}

void StandstillBehavior::calcInitialU(std::vector<float>& u, BehaviorFeedback& fb)
{
}

float StandstillBehavior::calcCost(const std::vector<float>& u)
{
    return 0.0f;
}

bool StandstillBehavior::preConditionSatisfied(
        const ContextData& context_data, Action::Ptr action) const
{
    if ( context_data.is_paused || !context_data.isPlanValid() ||
         context_data.plan[context_data.plan_index]->getType() != "goto" )
    {
        return true;
    }

    GoToAction::Ptr goto_action = std::dynamic_pointer_cast<GoToAction>(action);

    if ( goto_action == nullptr )
    {
        return true;
    }

    const Pose2D robot_pose = context_data.input_data->localisation_tf.asPose2D();

    const Pose2D& goal = goto_action->getGoal();

    if ( Utils::isWithinTolerance(robot_pose, goal,
                goal_tolerance_linear_, goal_tolerance_angular_) )
    {
        return true;
    }
    else
    {
        goto_action->setIntermediateGoal(TrajectoryPoint(goal));
        goto_action->enableIsDuringTransition();
        return false;
    }
}

bool StandstillBehavior::postConditionSatisfied(const ContextData& context_data) const
{
    return true;
}

} // namespace cabin
