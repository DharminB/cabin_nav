#pragma once

#include <cabin_nav/behavior/behavior.h>

namespace cabin {

class JoypadBehavior : public Behavior
{
    public:

        JoypadBehavior():
            Behavior("joypad") {}

        virtual ~JoypadBehavior() = default;

        bool configure(const YAML::Node& params);

        void reset();

        void runOnce(const ContextData& context_data, BehaviorFeedback& fb);

        bool preConditionSatisfied(
                const ContextData& context_data,
                std::shared_ptr<Action> action) const;

        bool postConditionSatisfied(const ContextData& context_data) const;

    protected:

        float calcCost(const std::vector<float>& u);

        void calcInitialU(std::vector<float>& u, BehaviorFeedback& fb);

};

} // namespace cabin
