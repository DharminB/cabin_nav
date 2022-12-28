#ifndef CABIN_OUTPUT_FACTORY_H
#define CABIN_OUTPUT_FACTORY_H

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

#endif // CABIN_OUTPUT_FACTORY_H
