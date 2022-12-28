#include <iostream>

#include <yaml_common/Parser2.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/action/action_factory.h>

#include <cabin_nav/action/goto_action.h>

using Parser = kelo::yaml_common::Parser2;

namespace cabin {

bool ActionFactory::configure(const YAML::Node& config)
{
    if ( !config.IsMap() )
    {
        std::cout << Print::Err << Print::Time() << "[ActionFactory] "
                  << "config is not in correct format. Expecting a map."
                  << Print::End << std::endl;
        return false;
    }

    /* initialise action config map */
    for ( YAML::const_iterator itr = config.begin(); itr != config.end(); itr ++ )
    {
        std::string action_type;
        if ( !Parser::read<std::string>(itr->first, action_type) )
        {
            std::cout << Print::Err << Print::Time() << "[ActionFactory] "
                      << "Could not read action type from " << itr->first
                      << Print::End << std::endl;
            return false;
        }

        action_config_map_[action_type] = YAML::Clone(itr->second);
    }

    return true;
}

Action::Ptr ActionFactory::createAction(
        const YAML::Node& action_params,
        const ContextData& context_data) const
{
    std::string type;
    if ( !Parser::read<std::string>(action_params, "type", type) )
    {
        std::cout << Print::Err << Print::Time() << "[ActionFactory] "
                  << "config does not contain type"
                  << Print::End << std::endl;
        return nullptr;
    }

    YAML::Node config;
    if ( action_config_map_.find(type) == action_config_map_.end() )
    {
        std::cout << Print::Warn << Print::Time() << "[ActionFactory] "
                  << "action_config_map does not contain any action of "
                  << type << " type. Providing empty config."
                  << Print::End << std::endl;
        config = YAML::Node();
    }
    else
    {
        config = action_config_map_.at(type);
    }

    Action::Ptr action;
    if ( type == "goto" )
    {
        action = std::make_shared<GoToAction>();
    }
    else
    {
        std::cout << Print::Err << Print::Time() << "[ActionFactory] "
                  << "action of type " << type << " is not supported"
                  << Print::End << std::endl;
        return nullptr;
    }

    if ( !action->configure(config, action_params, context_data) )
    {
        std::cout << Print::Err << Print::Time() << "[ActionFactory] "
                  << "action of type " << type << " could not be configured "
                  << "with following params" << std::endl << action_params
                  << Print::End << std::endl;
        return nullptr;
    }

    return action;
}

} // namespace cabin
