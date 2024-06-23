#pragma once

#include <vector>
#include <string>
#include <iostream>

#include <cabin_nav/structs/behavior_feedback.h>
#include <cabin_nav/behavior/behavior.h>
#include <cabin_nav/structs/status.h>

namespace cabin {

/* forward declaration */
struct ContextData;

class Action
{
    public:

        using Ptr = std::shared_ptr<Action>;

        virtual ~Action() = default;

        virtual bool configure(
                const YAML::Node& config,
                const YAML::Node& params,
                const ContextData& context_data) = 0;

        virtual Status recommendNextBehavior(
                ContextData& context_data,
                const Behavior::Map& behavior_map,
                const std::string& current_behavior,
                const BehaviorFeedback& fb,
                std::vector<std::string>& required_inputs,
                std::string& next_behavior) = 0;

        const std::vector<std::string>& getAllRequiredBehaviorNames() const
        {
            return required_behavior_names_;
        }

        virtual std::vector<visualization_msgs::Marker> asMarkers() const = 0;

        virtual std::string getPlanAsString() const = 0;

        virtual std::ostream& write(std::ostream& out) const = 0;

        const std::string& getType() const
        {
            return type_;
        }

    protected:

        /**
         * @brief Should be filled in Action::configure() function
         */
        std::vector<std::string> required_behavior_names_;

        std::unordered_map<std::string, std::string> inputs_map_;

        Action(const std::string& type):
            type_(type) {};

        bool parseInputs(const YAML::Node& config);

    private:

        std::string type_;

        Action() = delete;

};

inline std::ostream& operator << (std::ostream& out, const Action& action)
{
    return action.write(out);
};

} // namespace cabin
