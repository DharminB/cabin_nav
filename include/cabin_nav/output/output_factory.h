#pragma once

#include <yaml-cpp/yaml.h>

#include <cabin_nav/output/output.h>

namespace cabin {

class OutputFactory
{
    public:
        static Output::Ptr createOutput(
                const std::string& input_name, const YAML::Node& input_config);

};

} // namespace cabin
