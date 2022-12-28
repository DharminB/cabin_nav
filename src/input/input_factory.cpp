#include <cabin_nav/utils/print.h>
#include <cabin_nav/input/velocity_input.h>
#include <cabin_nav/input/localisation_input.h>
#include <cabin_nav/input/laser_input.h>
#include <cabin_nav/input/joypad_input.h>
#include <cabin_nav/input/pointcloud_input.h>
#include <cabin_nav/input/image_input.h>
#include <cabin_nav/input/occupancy_grid_map_input.h>
#include <cabin_nav/input/semantic_map_input.h>

#include <cabin_nav/input/input_factory.h>

namespace cabin {

Input::Ptr InputFactory::createInput(const std::string& input_name,
                                        const YAML::Node& input_config)
{
    if ( !input_config.IsMap() )
    {
        std::cerr << Print::Err << Print::Time() << "[InputFactory] "
                  << "Input " << input_name << " is not a map"
                  << Print::End << std::endl;
        return nullptr;
    }

    if ( !input_config["type"] )
    {
        std::cerr << Print::Err << Print::Time() << "[InputFactory] "
                  << "Input " << input_name << " does not have a type param"
                  << Print::End << std::endl;
        return nullptr;
    }

    std::string input_type = input_config["type"].as<std::string>();

    Input::Ptr input;
    if ( input_type == "VELOCITY" )
    {
        input = std::make_shared<VelocityInput>();
    }
    else if ( input_type == "LOCALISATION" )
    {
        input = std::make_shared<LocalisationInput>();
    }
    else if ( input_type == "LASER" )
    {
        input = std::make_shared<LaserInput>();
    }
    else if ( input_type == "JOYPAD" )
    {
        input = std::make_shared<JoypadInput>();
    }
    else if ( input_type == "IMG" )
    {
        input = std::make_shared<ImageInput>();
    }
    else if ( input_type == "POINTCLOUD" )
    {
        input = std::make_shared<PointCloudInput>();
    }
    else if ( input_type == "OCCUPANCY_GRID_MAP" )
    {
        input = std::make_shared<OccupancyGridMapInput>();
    }
    else if ( input_type == "SEMANTIC_MAP" )
    {
        input = std::make_shared<SemanticMapInput>();
    }
    else
    {
        std::cerr << Print::Err << Print::Time() << "[InputFactory] "
                  << "Input " << input_name << " has unsupported type " << input_type
                  << Print::End << std::endl;
        return nullptr;
    }

    if ( !input->configure(input_config) )
    {
        std::cerr << Print::Err << Print::Time() << "[InputFactory] "
                  << "Input " << input_name << " could not be configured"
                  << Print::End << std::endl;
        return nullptr;
    }

    return input;
}

} // namespace cabin
