#ifndef CABIN_JUNCTION_behavior_H
#define CABIN_JUNCTION_behavior_H

#include <geometry_common/TransformMatrix2D.h>

#include <cabin_nav/behavior/behavior.h>
#include <cabin_nav/utils/initial_guess_utils.h>

namespace cabin {

class JunctionBehavior : public Behavior
{
    public:

        JunctionBehavior():
            Behavior("junction") {}

        virtual ~JunctionBehavior() = default;

        bool configure(const YAML::Node& params);

        void reset();

        void runOnce(const ContextData& context_data, BehaviorFeedback& fb);

        bool preConditionSatisfied(
                const ContextData& context_data, std::shared_ptr<Action> action) const;

        bool postConditionSatisfied(const ContextData& context_data) const;

    protected:

        const ContextData* context_data_{nullptr};
        TrajectoryPoint goal_;
        TrajectoryPoint current_;
        kelo::geometry_common::TransformMatrix2D localisation_tf_;
        std::vector<float> u_;
        kelo::geometry_common::Path ideal_path_;
        kelo::geometry_common::PointCloud2D laser_pts_;
        CostFunction cf_;

        bool is_unicycle_{false};
        float inflation_dist_{0.1f};
        float weight_laser_pts_repulsion_{0.0f};
        float weight_goal_state_intercept_angle_{0.0f};
        float ideal_path_step_size_{0.2f};
        float lane_percentage_{0.5f};
        float pre_spline_control_pt_dist_{1.0f};
        float post_spline_control_pt_dist_{1.0f};
        float goal_dist_from_robot_{2.0f};
        float goal_tolerance_linear_pre_{1.0f};
        float goal_tolerance_angular_pre_{0.8f};
        float goal_tolerance_linear_post_{1.0f};
        float goal_tolerance_angular_post_{0.8f};

        bool use_initial_guess_utils_{true};
        InitialGuessUtils initial_guess_utils_;
        Trajectory initial_guess_trajectory_;

        JunctionBehavior(const std::string& type):
            Behavior(type) {}

        float calcCost(const std::vector<float>& u);

        void calcInitialU(std::vector<float>& u, BehaviorFeedback& fb);

        kelo::geometry_common::Pose2D calcGoal();

        kelo::geometry_common::Path calcIdealPath(const ContextData& context_data) const;

};

} // namespace cabin

#endif // CABIN_JUNCTION_behavior_H
