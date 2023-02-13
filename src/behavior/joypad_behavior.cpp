#include <geometry_common/Utils.h>
#include <yaml_common/Parser2.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/structs/context_data.h>
#include <cabin_nav/input/velocity_input_data.h>
#include <cabin_nav/input/joypad_input_data.h>
#include <cabin_nav/output/cmd_vel_output_data.h>
#include <cabin_nav/output/visualization_marker_output_data.h>
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
             parseRequiredInputs(config) &&
             parseOutputs(config) );
}

void JoypadBehavior::reset()
{
}

void JoypadBehavior::runOnce(const ContextData& context_data, BehaviorFeedback& fb)
{
    VisualizationMarkerOutputData::addMarker(fb.output_data_map,
            outputs_map_.at("markers"), behavior_name_marker_);
    VisualizationMarkerOutputData::addMarkers(fb.output_data_map,
            outputs_map_.at("markers"), context_data.getPlanMarkers());
    VisualizationMarkerOutputData::addMarker(fb.output_data_map,
            outputs_map_.at("markers"), footprint_marker_);

    Velocity2D curr_vel;
    Joypad joypad;
    if ( !VelocityInputData::getVelocity(context_data.input_data_map,
             required_inputs_map_.at("velocity"), curr_vel) ||
         !JoypadInputData::getJoypad(context_data.input_data_map,
             required_inputs_map_.at("joypad"), joypad) )
    {
        std::cout << Print::Err << Print::Time() << "[StandstillBehavior] "
                  << "Could not get velocity and/or joypad."
                  << Print::End << std::endl;
        CmdVelOutputData::setTrajectory(
                fb.output_data_map, calcRampedTrajectory(curr_vel));
        fb.success = false;
        return;
    }

    Velocity2D target_vel;
    if ( joypad.buttons[4] == 1 && // LB
         joypad.buttons[5] == 0 )  // RB
    {
        target_vel.x = joypad.axes[1];
        target_vel.y = joypad.axes[0];
        target_vel.theta = joypad.axes[2];
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
    CmdVelOutputData::setTrajectory(
            fb.output_data_map, calcRampedTrajectory(curr_vel, target_vel));
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
    bool is_lb_pressed = false, is_rb_pressed = false, is_yellow_button_pressed = false;
    return ( context_data.active_inputs.at("joypad") &&
             JoypadInputData::isLeftButtonPressed(context_data.input_data_map,
                 required_inputs_map_.at("joypad"), is_lb_pressed) &&
             JoypadInputData::isRightButtonPressed(context_data.input_data_map,
                 required_inputs_map_.at("joypad"), is_rb_pressed) &&
             JoypadInputData::isYellowButtonPressed(context_data.input_data_map,
                 required_inputs_map_.at("joypad"), is_yellow_button_pressed) &&
             ( is_lb_pressed || is_rb_pressed || is_yellow_button_pressed ) );
}

bool JoypadBehavior::postConditionSatisfied(const ContextData& context_data) const
{
    bool is_green_button_pressed = false, is_rb_pressed = false;
    return ( context_data.active_inputs.at("joypad") &&
             JoypadInputData::isGreenButtonPressed(context_data.input_data_map,
                 required_inputs_map_.at("joypad"), is_green_button_pressed) &&
             JoypadInputData::isRightButtonPressed(context_data.input_data_map,
                 required_inputs_map_.at("joypad"), is_rb_pressed) &&
             is_green_button_pressed &&
             !is_rb_pressed );
}

} // namespace cabin
