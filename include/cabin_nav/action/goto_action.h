#pragma once

#include <geometry_common/Pose2D.h>

#include <cabin_nav/action/action.h>
#include <cabin_nav/semantic_map/area.h>
#include <cabin_nav/semantic_map/connection.h>
#include <cabin_nav/semantic_map/semantic_map.h>

namespace cabin {

class GoToAction: public Action
{
    public:

        using Ptr = std::shared_ptr<GoToAction>;

        GoToAction():
            Action("goto") {}

        virtual ~GoToAction() = default;

        bool configure(
                const YAML::Node& config,
                const YAML::Node& params,
                const ContextData& context_data) override;

        Status recommendNextBehavior(
                ContextData& context_data,
                const Behavior::Map& behavior_map,
                const std::string& current_behavior,
                const BehaviorFeedback& fb,
                std::vector<std::string>& required_inputs,
                std::string& next_behavior) override;

        std::vector<visualization_msgs::Marker> asMarkers() const override;

        std::string getPlanAsString() const override;

        std::ostream& write(std::ostream& out) const override;

        void setGoal(const kelo::geometry_common::Pose2D& goal);

        kelo::geometry_common::Pose2D getGoal() const;

        TrajectoryPoint getIntermediateGoal() const;

        const std::vector<Area::ConstPtr>& getGoToPlan() const;

        size_t getGoToPlanIndex() const;

        bool isDuringTransition() const;

        bool isDuringRecovery() const;

        void setIntermediateGoal(const TrajectoryPoint& intermediate_goal);

        void enableIsDuringTransition();

    protected:

        kelo::geometry_common::Pose2D goal_;
        float goal_tolerance_linear_{0.2f};
        float goal_tolerance_angular_{0.2f};
        TrajectoryPoint intermediate_goal_;

        std::vector<Area::ConstPtr> goto_plan_;
        size_t goto_plan_index_{0};
        bool is_during_transition_{true};
        bool is_during_recovery_{false};
        std::vector<std::string> recovery_behavior_stack_;

        Status recursiveRecommendNextBehavior(
                const ContextData& context_data,
                const Behavior::Map& behavior_map,
                const std::string& current_behavior,
                const BehaviorFeedback& fb,
                std::string& next_behavior);

        std::string getBehaviorNameFromAreaType(const std::string& area_type);

        std::string getBehaviorNameBasedOnRequiredInputs(
                const std::string& preferred_behavior_name,
                const ContextData& context_data,
                const std::map<std::string, std::shared_ptr<Behavior> >& behavior_map,
                std::vector<std::string>& required_inputs);

        void plan(const ContextData& context_data,
                  const kelo::geometry_common::Pose2D& start);

        bool reachedGoal(const ContextData& context_data) const;

        bool fillRequiredBehaviorNames(
                const YAML::Node& config,
                const YAML::Node& params);

        static void addBehaviorName(
                const std::string& behavior_name,
                const std::set<std::string>& available_behaviors,
                std::set<std::string>& required_behaviors);

};

} // namespace cabin
