#ifndef CABIN_ACTION_FACTORY_H
#define CABIN_ACTION_FACTORY_H

#include <unordered_map>

#include <yaml-cpp/yaml.h>

#include <cabin_nav/action/action.h>

namespace cabin {

class ActionFactory
{
    public:

        ActionFactory() = default;

        virtual ~ActionFactory() = default;

        bool configure(const YAML::Node& config);

        Action::Ptr createAction(
                const YAML::Node& action_params,
                const ContextData& context_data) const;

    private:

        std::unordered_map<std::string, YAML::Node> action_config_map_;

};

} // namespace cabin

#endif // CABIN_ACTION_FACTORY_H
