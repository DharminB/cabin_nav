#include <iostream>

#include <yaml_common/Parser2.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/behavior/behavior_factory.h>

/* goto action */
#include <cabin_nav/behavior/standstill_behavior.h>
#include <cabin_nav/behavior/ptp_behavior.h>
#include <cabin_nav/behavior/ptp_occ_grid_behavior.h>
#include <cabin_nav/behavior/corridor_behavior.h>
#include <cabin_nav/behavior/junction_behavior.h>
#include <cabin_nav/behavior/door_behavior.h>
#include <cabin_nav/behavior/open_area_behavior.h>
#include <cabin_nav/behavior/joypad_behavior.h>
#include <cabin_nav/behavior/recovery_ptp_behavior.h>

using Parser = kelo::yaml_common::Parser2;

namespace cabin {

bool BehaviorFactory::configure(const YAML::Node& config)
{
    if ( !config.IsMap() )
    {
        std::cout << Print::Err << Print::Time() << "[BehaviorFactory] "
                  << "config is not in correct format. Expecting a map."
                  << Print::End << std::endl;
        return false;
    }

    /* initialise behavior map */
    for ( YAML::const_iterator itr = config.begin(); itr != config.end(); itr ++ )
    {
        std::string behavior_name;
        if ( !Parser::read<std::string>(itr->first, behavior_name) )
        {
            std::cout << Print::Err << Print::Time() << "[BehaviorFactory] "
                      << "Could not read behavior name from " << itr->first
                      << Print::End << std::endl;
            return false;
        }

        if ( !Parser::has<std::string>(itr->second, "type") )
        {
            std::cout << Print::Err << Print::Time() << "[BehaviorFactory] "
                      << behavior_name << " behavior does not have type"
                      << Print::End << std::endl;
            return false;
        }

        behavior_config_map_[behavior_name] = YAML::Clone(itr->second);
    }

    return true;
}

Behavior::Ptr BehaviorFactory::createBehavior(const std::string& behavior_name)
{
    if ( behavior_config_map_.find(behavior_name) == behavior_config_map_.end() )
    {
        std::cout << Print::Err << Print::Time() << "[BehaviorFactory] "
                  << "config does not contain a behavior named " << behavior_name
                  << Print::End << std::endl;
        return nullptr;
    }

    const YAML::Node& config = behavior_config_map_.at(behavior_name);
    const std::string type = Parser::get<std::string>(config, "type", "");

    Behavior::Ptr behavior;
    if ( type == "standstill" )
    {
        behavior = std::make_shared<StandstillBehavior>();
    }
    else if ( type == "ptp" )
    {
        behavior = std::make_shared<PTPBehavior>();
    }
    else if ( type == "ptp_occ_grid" )
    {
        behavior = std::make_shared<PTPOccGridBehavior>();
    }
    else if ( type == "corridor" )
    {
        behavior = std::make_shared<CorridorBehavior>();
    }
    else if ( type == "junction" )
    {
        behavior = std::make_shared<JunctionBehavior>();
    }
    else if ( type == "door" )
    {
        behavior = std::make_shared<DoorBehavior>();
    }
    else if ( type == "open_area" )
    {
        behavior = std::make_shared<OpenAreaBehavior>();
    }
    else if ( type == "joypad" )
    {
        behavior = std::make_shared<JoypadBehavior>();
    }
    else if ( type == "recovery_ptp" )
    {
        behavior = std::make_shared<RecoveryPTPBehavior>();
    }
    else
    {
        std::cout << Print::Err << Print::Time() << "[BehaviorFactory] "
                  << "behavior of type " << type << " is not supported."
                  << Print::End << std::endl;
        return nullptr;
    }
    if ( !behavior->configure(config) )
    {
        std::cout << Print::Err << Print::Time() << "[BehaviorFactory] "
                  << "behavior of type " << type << " could not be configured."
                  << Print::End << std::endl;
        return nullptr;
    }

    behavior->setName(behavior_name);

    return behavior;
}

} // namespace cabin
