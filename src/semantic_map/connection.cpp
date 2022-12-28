#include <yaml_common/Parser2.h>

#include <cabin_nav/semantic_map/connection.h>

using Parser = kelo::yaml_common::Parser2;

namespace cabin {

bool Connection::initialise(const YAML::Node& connection_yaml)
{
    return ( Parser::read<std::string>(connection_yaml, "area_1", area_1) &&
             Parser::read<std::string>(connection_yaml, "area_2", area_2) );
}

std::ostream& operator << (std::ostream& out, const Connection& connection)
{
    out << "<Connection " << connection.area_1 << " <---> " << connection.area_2 << " >";
    return out;
}

YAML::Emitter& operator << (YAML::Emitter& out, const Connection& connection)
{
    out << YAML::Flow << YAML::BeginMap;
    out << YAML::Key << "area_1" << YAML::Value << connection.area_1;
    out << YAML::Key << "area_2" << YAML::Value << connection.area_2;
    out << YAML::EndMap;
    return out;
}

} // namespace cabin
