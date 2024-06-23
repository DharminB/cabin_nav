#pragma once

#include <unordered_map>

#include <yaml-cpp/yaml.h>

#include <cabin_nav/behavior/behavior.h>

namespace cabin {

class BehaviorFactory
{
    public:
        BehaviorFactory() = default;

        virtual ~BehaviorFactory() = default;

        bool configure(const YAML::Node& config);

        Behavior::Ptr createBehavior(const std::string& behavior_name);

    private:

        std::unordered_map<std::string, YAML::Node> behavior_config_map_;

};

} // namespace cabin
