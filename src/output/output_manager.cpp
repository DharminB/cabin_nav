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

        if ( outputs_.count(output_name) > 0 )
        {
            std::cerr << Print::Err << Print::Time() << "[OutputManager] "
                      << "Output config file contains multiple outputs with same name"
                      << output_name
                      << Print::End << std::endl;
            return false;
        }
        // std::cout << *output << std::endl;
        outputs_[output_name] = output;
    }

    return true;
}

bool OutputManager::setData(
        OutputData::Ptr& output_data, const InputData::Ptr& input_data)
{
    for ( auto output_itr = outputs_.begin(); output_itr != outputs_.end(); output_itr ++ )
    {
        if ( !output_itr->second->setData(output_data, input_data, output_itr->first) )
        {
            return false;
        }
    }
    return true;
}

void OutputManager::stopAllOutputs()
{
    for ( auto itr = outputs_.begin(); itr != outputs_.end(); itr ++ )
    {
        itr->second->stop();
    }
}


} // namespace cabin
