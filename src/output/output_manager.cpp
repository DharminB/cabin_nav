#include <yaml_common/Parser2.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/output/output_factory.h>
#include <cabin_nav/output/output_manager.h>

using Parser = kelo::yaml_common::Parser2;

namespace cabin {

bool OutputManager::configure(const YAML::Node& config)
{
    if ( !config.IsMap() )
    {
        std::cerr << Print::Err << Print::Time() << "[OutputManager] "
                  << "output config file does not have a correct format. Expecting a map."
                  << Print::End << std::endl;
        return false;
    }

    for ( YAML::const_iterator it = config.begin(); it != config.end(); it ++ )
    {
        std::string output_name;
        if ( !Parser::read<std::string>(it->first, output_name) )
        {
            return false;
        }

        Output::Ptr output = OutputFactory::createOutput(
                output_name, it->second);

        if ( !output )
        {
            return false;
        }

        output->start();

        if ( outputs_.find(output_name) != outputs_.end() )
        {
            std::cerr << Print::Err << Print::Time() << "[OutputManager] "
                      << "Output config file contains multiple outputs with same name"
                      << output_name
                      << Print::End << std::endl;
            return false;
        }
        // std::cout << *output << std::endl;
        outputs_[output_name] = output;
        output_names_.push_back(output_name);
    }

    return true;
}

void OutputManager::initializeOutputDataMap(OutputData::Map& output_data_map)
{
    for ( const std::string& output_name : output_names_ )
    {
        output_data_map[output_name] = nullptr;
        outputs_[output_name]->initializeOutputData(output_data_map[output_name]);
    }
}

bool OutputManager::setData(
        const OutputData::Map& output_data_map,
        const InputData::Map& input_data_map)
{
    for ( const std::string& output_name : output_names_ )
    {
        if ( !outputs_[output_name]->setData(output_data_map.at(output_name), input_data_map) )
        {
            return false;
        }
    }
    return true;
}

void OutputManager::stopAllOutputs()
{
    for ( const std::string& output_name : output_names_ )
    {
        outputs_[output_name]->stop();
    }
}

void OutputManager::resetOutputDataMap(OutputData::Map& output_data_map) const
{
    for ( const std::string& output_name : output_names_ )
    {
        output_data_map[output_name]->reset();
    }
}

} // namespace cabin
