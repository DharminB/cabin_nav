#pragma once

#include <geometry_common/TransformMatrix2D.h>

#include <cabin_nav/behavior/behavior.h>
#include <cabin_nav/utils/initial_guess_utils.h>

namespace cabin {

class OpenAreaBehavior : public Behavior
{
    public:

        OpenAreaBehavior():
            Behavior("open_area") {}

        virtual ~OpenAreaBehavior() = default;

        bool configure(const YAML::Node& params);

        void reset();

        void runOnce(const ContextData& context_data, BehaviorFeedback& fb);

        bool preConditionSatisfied(
                const ContextData& context_data, std::shared_ptr<Action> action) const;

        bool postConditionSatisfied(const ContextData& context_data) const;

    protected:

        const ContextData* context_data_{nullptr};
        std::vector<float> u_;
        TrajectoryPoint goal_;
        TrajectoryPoint current_;
        kelo::geometry_common::TransformMatrix2D localisation_tf_;
        kelo::geometry_common::Path geometric_plan_;
        kelo::geometry_common::PointCloud2D laser_pts_;
        CostFunction cf_;

        float min_inflation_dist_{0.1f};
        float max_inflation_dist_{10.0f};
        float weight_laser_pts_repulsion_{0.0f};
        float weight_linear_vel_ellipse_{0.0f};
        float goal_tolerance_linear_pre_{1.0f};
        float goal_tolerance_linear_post_open_area_{1.0f};
        float goal_tolerance_linear_post_diff_area_{2.0f};
        float goal_tolerance_linear_post_final_goal_{3.0f};
        float robot_to_goal_dist_{5.0f};

        bool use_initial_guess_utils_{true};
        InitialGuessUtils initial_guess_utils_;
        Trajectory initial_guess_trajectory_;

        OpenAreaBehavior(const std::string& type):
            Behavior(type) {}

        float calcCost(const std::vector<float>& u);

        void calcInitialU(std::vector<float>& u, BehaviorFeedback& fb);

        kelo::geometry_common::Pose2D calcGoal();

        void calcSpline();

};

} // namespace cabin
