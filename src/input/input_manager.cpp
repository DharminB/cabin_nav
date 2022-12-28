#include <yaml_common/Parser2.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/input/input_factory.h>
#include <cabin_nav/input/input_manager.h>

using Parser = kelo::yaml_common::Parser2;

namespace cabin {

bool InputManager::configure(const YAML::Node& config)
{
    if ( !config.IsMap() )
    {
        std::cerr << Print::Err << Print::Time() << "[InputManager] "
                  << "Input config file does not have a correct format. "
                  << "Expecting a map."
                  << Print::End << std::endl;
        return false;
    }

    for ( YAML::const_iterator it = config.begin(); it != config.end(); it ++ )
    {
        std::string input_name;
        if ( !Parser::read<std::string>(it->first, input_name) )
        {
            return false;
        }
        const YAML::Node& input_config = it->second;

        Input::Ptr input = InputFactory::createInput(input_name, input_config);
        if ( !input )
        {
            return false;
        }

        if ( inputs_.count(input_name) > 0 )
        {
            std::cerr << Print::Err << Print::Time() << "[InputManager] "
                      << "Input config file contains multiple inputs with same name"
                      << input_name
                      << Print::End << std::endl;
            return false;
        }
        // std::cout << *input << std::endl;
        inputs_[input_name] = input;

        /* activate always active input */
        if ( input->alwaysActive() )
        {
            input->activate();
        }
    }

    return true;
}

bool InputManager::getData(InputData::Ptr input_data)
{
    for ( auto inputs_itr = inputs_.begin(); inputs_itr != inputs_.end(); inputs_itr ++ )
    {
        if ( inputs_itr->second->isActive() )
        {
            if ( !inputs_itr->second->getData(input_data, inputs_itr->first) )
            {
                return false;
            }
        }
    }
    return true;
}

std::map<std::string, bool> InputManager::getActiveInputs()
{
    std::map<std::string, bool> active_inputs;
    for ( auto inputs_itr = inputs_.begin(); inputs_itr != inputs_.end(); inputs_itr ++ )
    {
        active_inputs[inputs_itr->first] = inputs_itr->second->isActive();
    }
    return active_inputs;
}

void InputManager::updateActiveInputs(const std::vector<std::string>& required_inputs)
{
    for ( auto inputs_itr = inputs_.begin(); inputs_itr != inputs_.end(); inputs_itr ++ )
    {
        bool is_required = std::find(required_inputs.begin(),
                                     required_inputs.end(),
                                     inputs_itr->first) != required_inputs.end();
        bool is_active = inputs_itr->second->isActive();
        if ( is_required && !is_active )
        {
            inputs_itr->second->activate();
        }

        else if ( !is_required && is_active )
        {
            inputs_itr->second->deactivate();
        }
    }
}

} // namespace cabin
