#include <geometry_common/Utils.h>
#include <yaml_common/Parser2.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/structs/context_data.h>
#include <cabin_nav/action/goto_action.h>
#include <cabin_nav/behavior/joypad_behavior.h>

using kelo::geometry_common::Pose2D;
using kelo::geometry_common::Velocity2D;
using kelo::geometry_common::TransformMatrix2D;
using GCUtils = kelo::geometry_common::Utils;
using Parser = kelo::yaml_common::Parser2;

namespace cabin {

bool JoypadBehavior::configure(const YAML::Node& config)
{
    return ( parseControlHorizon(config) &&
             parseAccLimits(config) &&
             parseVelLimits(config) &&
             parseRequiredInputs(config) );
}

void JoypadBehavior::reset()
{
}

void JoypadBehavior::runOnce(const ContextData& context_data, BehaviorFeedback& fb)
{
    fb.output_data->markers.push_back(behavior_name_marker_);
    Velocity2D curr_vel =  context_data.input_data->current_vel;
    Velocity2D target_vel;

    if ( context_data.input_data->joypad.buttons[4] == 1 && // LB
         context_data.input_data->joypad.buttons[5] == 0 )  // RB
    {
        target_vel.x = context_data.input_data->joypad.axes[1];
        target_vel.y = context_data.input_data->joypad.axes[0];
        target_vel.theta = context_data.input_data->joypad.axes[2];
        target_vel.x = ( target_vel.x >= 0 )
                       ? std::fabs(target_vel.x) * max_vel_.x
                       : std::fabs(target_vel.x) * min_vel_.x;
        target_vel.y = ( target_vel.y >= 0 )
                       ? std::fabs(target_vel.y) * max_vel_.y
                       : std::fabs(target_vel.y) * min_vel_.y;
        target_vel.theta = ( target_vel.theta >= 0 )
                           ? std::fabs(target_vel.theta) * max_vel_.theta
                           : std::fabs(target_vel.theta) * min_vel_.theta;
    }

    fb.output_data->trajectory = calcRampedTrajectory(curr_vel, target_vel);

    std::vector<visualization_msgs::Marker> plan_markers = context_data.getPlanMarkers();
    fb.output_data->markers.insert(fb.output_data->markers.end(),
            plan_markers.begin(), plan_markers.end());
    fb.output_data->markers.push_back(footprint_marker_);

    fb.success = true;

}

void JoypadBehavior::calcInitialU(std::vector<float>& u, BehaviorFeedback& fb)
{
}

float JoypadBehavior::calcCost(const std::vector<float>& u)
{
    return 0.0f;
}

bool JoypadBehavior::preConditionSatisfied(
        const ContextData& context_data, Action::Ptr action) const
{
    return ( context_data.active_inputs.at("joypad") &&
             ( context_data.input_data->joypad.buttons[4] == 1 ||  // LB
               context_data.input_data->joypad.buttons[5] == 1) ); // RB
}

bool JoypadBehavior::postConditionSatisfied(const ContextData& context_data) const
{
    return ( context_data.active_inputs.at("joypad") &&
             context_data.input_data->joypad.buttons[1] == 1 && // A (green button)
             context_data.input_data->joypad.buttons[5] == 0 ); // RB
}

} // namespace cabin
