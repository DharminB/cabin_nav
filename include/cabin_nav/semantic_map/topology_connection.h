#ifndef CABIN_TOPOLOGY_CONNECTION_H
#define CABIN_TOPOLOGY_CONNECTION_H

#include <string>

#include <yaml-cpp/yaml.h>

namespace cabin {

class TopologyConnection
{
    public:
        size_t node_1;
        size_t node_2;

        bool initialise(const YAML::Node& node_connection_yaml);

        friend std::ostream& operator << (
                std::ostream& out,
                const TopologyConnection& connection);

        friend YAML::Emitter& operator << (
                YAML::Emitter& out,
                const TopologyConnection& connection);

};

} // namespace cabin

#endif // CABIN_TOPOLOGY_CONNECTION_H
