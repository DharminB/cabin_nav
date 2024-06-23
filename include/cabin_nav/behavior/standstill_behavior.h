#pragma once

#include <cabin_nav/behavior/behavior.h>

namespace cabin {

class StandstillBehavior : public Behavior
{
    public:

        StandstillBehavior():
            Behavior("standstill") {}

        virtual ~StandstillBehavior() = default;

        bool configure(const YAML::Node& params);

        void reset();

        void runOnce(const ContextData& context_data, BehaviorFeedback& fb);

        bool preConditionSatisfied(
                const ContextData& context_data,
                std::shared_ptr<Action> action) const;

        bool postConditionSatisfied(const ContextData& context_data) const;

    private:

        float calcCost(const std::vector<float>& u);

        void calcInitialU(std::vector<float>& u, BehaviorFeedback& fb);

};

} // namespace cabin
