#pragma once

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
