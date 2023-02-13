#include <ros/package.h>

#include <yaml_common/Parser2.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/input/semantic_map_input.h>
#include <cabin_nav/input/semantic_map_input_data.h>

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

bool SemanticMapInput::getData(InputData::Ptr& input_data)
{
    if ( input_data == nullptr ) // for first time
    {
        input_data = std::make_shared<SemanticMapInputData>();
    }

    if ( input_data->getType() != getType() )
    {
        std::cout << Print::Err << Print::Time() << "[SemanticMapInput] "
                  << "input_data's type is not \"" << getType() << "\"."
                  << Print::End << std::endl;
        return false;
    }

    SemanticMapInputData::Ptr semantic_map_input_data =
        std::static_pointer_cast<SemanticMapInputData>(input_data);

    if ( semantic_map_input_data->semantic_map == nullptr )
    {
        semantic_map_input_data->semantic_map = semantic_map_;
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

std::ostream& SemanticMapInput::write(std::ostream& out) const
{
    out << "<Input type: " << getType() << ">";
    return out;
}

} // namespace cabin
