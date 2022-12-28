#include <cabin_nav/utils/print.h>
#include <cabin_nav/output/cmd_vel_output.h>
#include <cabin_nav/output/visualization_marker_output.h>

#include <cabin_nav/output/output_factory.h>

namespace cabin {

Output::Ptr OutputFactory::createOutput(
        const std::string& output_name, const YAML::Node& output_config)
{
    if ( !output_config.IsMap() )
    {
        std::cerr << Print::Err << Print::Time() << "[OutputFactory] "
                  << "Output " << output_name << " is not a map"
                  << Print::End << std::endl;
        return nullptr;
    }

    if ( !output_config["type"] )
    {
        std::cerr << Print::Err << Print::Time() << "[OutputFactory] "
                  << "Output " << output_name << " does not have a type param"
                  << Print::End << std::endl;
        return nullptr;
    }

    std::string output_type = output_config["type"].as<std::string>();

    Output::Ptr output;
    if ( output_type == "CMD_VEL" )
    {
        output = std::make_shared<CmdVelOutput>();
    }
    else if ( output_type == "VISUALIZATION_MARKER" )
    {
        output = std::make_shared<VisualizationMarkerOutput>();
    }
    else
    {
        std::cerr << Print::Err << Print::Time() << "[OutputFactory] "
                  << "Output " << output_name << " has unsupported type " << output_type
                  << Print::End << std::endl;
        return nullptr;
    }

    if ( !output->configure(output_config) )
    {
        std::cerr << Print::Err << Print::Time() << "[OutputFactory] "
                  << "Output " << output_name << " could not be configured"
                  << Print::End << std::endl;
        return nullptr;
    }

    return output;
}

} // namespace cabin
