#include <geometry_common/Utils.h>
#include <yaml_common/Parser2.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/structs/context_data.h>
#include <cabin_nav/input/velocity_input_data.h>
#include <cabin_nav/output/cmd_vel_output_data.h>
#include <cabin_nav/output/visualization_marker_output_data.h>
#include <cabin_nav/action/goto_action.h>
#include <cabin_nav/behavior/standstill_behavior.h>

using kelo::geometry_common::Velocity2D;
using kelo::geometry_common::Pose2D;
using GCUtils = kelo::geometry_common::Utils;
using Parser = kelo::yaml_common::Parser2;

namespace cabin {

bool StandstillBehavior::configure(const YAML::Node& config)
{
    return ( parseControlHorizon(config) &&
             parseAccLimits(config) &&
             parseRequiredInputs(config) &&
             parseOutputs(config) );
}

void StandstillBehavior::reset()
{
}

void StandstillBehavior::runOnce(const ContextData& context_data, BehaviorFeedback& fb)
{
    VisualizationMarkerOutputData::addMarker(fb.output_data_map,
            outputs_map_.at("markers"), behavior_name_marker_);
    VisualizationMarkerOutputData::addMarkers(fb.output_data_map,
            outputs_map_.at("markers"), context_data.getPlanMarkers());

    Velocity2D current_vel;
    if ( !VelocityInputData::getVelocity(context_data.input_data_map,
             required_inputs_map_.at("velocity"), current_vel) )
    {
        std::cout << Print::Warn << Print::Time() << "[StandstillBehavior] "
                  << "Could not get velocity. Assuming zero velocity."
                  << Print::End << std::endl;
    }

    CmdVelOutputData::setTrajectory(
            fb.output_data_map, calcRampedTrajectory(current_vel));

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
    return true;
}

bool StandstillBehavior::postConditionSatisfied(const ContextData& context_data) const
{
    return true;
}

} // namespace cabin
