#pragma once

#include <yaml-cpp/yaml.h>

#include <cabin_nav/structs/context_data.h>

namespace cabin {

class TaskPlanner
{
    public:

        static bool plan(
                const YAML::Node& task_request,
                const ContextData& context_data,
                std::vector<YAML::Node>& actions_params);

    private:

        static bool planGoToTask(
                const YAML::Node& task_request,
                const ContextData& context_data,
                std::vector<YAML::Node>& actions_params);

        static bool planDisinfectTask(
                const YAML::Node& task_request,
                const ContextData& context_data,
                std::vector<YAML::Node>& actions_params);

        static bool planDockTask(
                const YAML::Node& task_request,
                const ContextData& context_data,
                std::vector<YAML::Node>& actions_params);

        static bool planUndockTask(
                const YAML::Node& task_request,
                const ContextData& context_data,
                std::vector<YAML::Node>& actions_params);

        static bool planTransportTask(
                const YAML::Node& task_request,
                const ContextData& context_data,
                std::vector<YAML::Node>& actions_params);

};

} // namespace cabin
