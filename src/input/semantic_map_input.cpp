#include <ros/package.h>

#include <yaml_common/Parser2.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/input/semantic_map_input.h>

using kelo::geometry_common::TransformMatrix2D;
using Parser = kelo::yaml_common::Parser2;

namespace cabin {

bool SemanticMapInput::configure(const YAML::Node& config)
{
    std::string pkg_name, relative_file_path;
    if ( !Parser::read<std::string>(config, "pkg_name", pkg_name) ||
         !Parser::read<std::string>(config, "relative_file_path", relative_file_path) )
    {
        std::cout << Print::Err << Print::Time() << "[SemanticMapInput] "
                  << "Missing parameter/s pkg_name and/or relative_file_path"
                  << Print::End << std::endl;
        return false;
    }
    always_active_ = Parser::get<bool>(config, "always_active", true);

    const std::string pkg_path = ros::package::getPath(pkg_name);
    const std::string map_file_path = pkg_path + "/" + relative_file_path;

    semantic_map_ = std::make_shared<SemanticMap>();
    return semantic_map_->initialise(map_file_path);
}

bool SemanticMapInput::getData(
        InputData::Ptr& input_data,
        const std::string& input_name)
{
    if ( input_data->semantic_map == nullptr )  // for first time
    {
        input_data->semantic_map = semantic_map_;
    }

    return true;
}

void SemanticMapInput::activate()
{
    is_active_ = true;
}

void SemanticMapInput::deactivate()
{
    is_active_ = false;
}

} // namespace cabin
