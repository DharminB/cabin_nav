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
                      << "Input config file contains multiple inputs with same name "
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

void InputManager::initializeInputData(
        InputData::Map& input_data,
        std::unordered_map<std::string, bool> active_inputs)
{
    for ( auto inputs_itr = inputs_.begin(); inputs_itr != inputs_.end(); inputs_itr ++ )
    {
        const std::string& input_name = inputs_itr->first;
        input_names_.push_back(input_name);
        input_data[input_name] = nullptr;
        active_inputs[input_name] = false;
    }
}

bool InputManager::getData(InputData::Map& input_data)
{
    for ( const std::string& input_name : input_names_ )
    {
        if ( inputs_[input_name]->isActive() )
        {
            if ( !inputs_[input_name]->getData(input_data[input_name]) )
            {
                return false;
            }
        }
    }
    return true;
}

void InputManager::getActiveInputs(
        std::unordered_map<std::string, bool>& active_inputs) const
{
    for ( const std::string& input_name : input_names_ )
    {
        active_inputs[input_name] = inputs_.at(input_name)->isActive();
    }
}

void InputManager::updateActiveInputs(const std::vector<std::string>& required_inputs)
{
    for ( const std::string& input_name : input_names_ )
    {
        bool is_required = std::find(required_inputs.begin(), required_inputs.end(),
                                     input_name) != required_inputs.end();
        bool is_active = inputs_.at(input_name)->isActive();
        if ( is_required && !is_active )
        {
            inputs_[input_name]->activate();
        }
        else if ( !is_required && is_active )
        {
            inputs_[input_name]->deactivate();
        }
    }
}

} // namespace cabin
