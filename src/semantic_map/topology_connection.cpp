#include <yaml_common/Parser2.h>

#include <cabin_nav/semantic_map/topology_connection.h>

using Parser = kelo::yaml_common::Parser2;

namespace cabin {

bool TopologyConnection::initialise(const YAML::Node& node_connection_yaml)
{
    return ( Parser::read<size_t>(node_connection_yaml, "node_1", node_1) &&
             Parser::read<size_t>(node_connection_yaml, "node_2", node_2) );
}

std::ostream& operator << (
        std::ostream& out,
        const TopologyConnection& connection)
{
    out << "<TopologyConnection " << connection.node_1
        << " <---> " << connection.node_2 << " >";
    return out;
}

YAML::Emitter& operator << (YAML::Emitter& out, const TopologyConnection& connection)
{
    out << YAML::Flow << YAML::BeginMap;
    out << YAML::Key << "node_1" << YAML::Value << connection.node_1;
    out << YAML::Key << "node_2" << YAML::Value << connection.node_2;
    out << YAML::EndMap;
    return out;
}

} // namespace cabin
