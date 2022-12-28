#ifndef CABIN_CONNECTION_H
#define CABIN_CONNECTION_H

#include <string>

#include <yaml-cpp/yaml.h>

namespace cabin {

class Connection
{
    public:

        std::string area_1;
        std::string area_2;

        bool initialise(const YAML::Node& connection_yaml);

        friend std::ostream& operator << (
                std::ostream& out,
                const Connection& connection);

        friend YAML::Emitter& operator << (
                YAML::Emitter& out,
                const Connection& connection);

};

} // namespace cabin

#endif // CABIN_CONNECTION_H
