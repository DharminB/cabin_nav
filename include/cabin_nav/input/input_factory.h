#ifndef CABIN_INPUT_FACTORY_H
#define CABIN_INPUT_FACTORY_H

#include <yaml-cpp/yaml.h>

#include <cabin_nav/input/input.h>

namespace cabin {

class InputFactory
{
    public:
        static Input::Ptr createInput(const std::string& input_name,
                                        const YAML::Node& input_config);

};

} // namespace cabin

#endif // CABIN_INPUT_FACTORY_H
