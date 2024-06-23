#pragma once

#include <cabin_nav/behavior/ptp_behavior.h>

namespace cabin {

class RecoveryPTPBehavior : public PTPBehavior
{
    public:

        RecoveryPTPBehavior():
            PTPBehavior("recovery_ptp") {}

        virtual ~RecoveryPTPBehavior() = default;

        bool configure(const YAML::Node& config);

        void reset();

        void runOnce(const ContextData& context_data, BehaviorFeedback& fb);

        bool preConditionSatisfied(
                const ContextData& context_data,
                std::shared_ptr<Action> action) const;

        bool postConditionSatisfied(const ContextData& context_data) const;

    protected:

        TrajectoryPoint global_goal_;
        bool is_global_goal_valid_{false};
        float max_goal_dist_{2.0f};
        float goal_tolerance_linear_post_{0.5f};
        float goal_tolerance_angular_post_{0.5f};
        float recovery_time_threshold_{10.0f};
        std::chrono::steady_clock::time_point recovery_start_time_;

        TrajectoryPoint calcGoal(const ContextData& context_data, BehaviorFeedback& fb);

};

} // namespace cabin
