#pragma once

#include <geometry_common/TransformMatrix2D.h>

#include <cabin_nav/behavior/behavior.h>
#include <cabin_nav/utils/initial_guess_utils.h>
#include <cabin_nav/utils/obstacle_tracker.h>
#include <cabin_nav/utils/geometric_planner.h>
#include <cabin_nav/structs/occupancy_grid.h>
#include <cabin_nav/structs/moving_circle.h>

namespace cabin {

class PTPBehavior : public Behavior
{
    public:

        PTPBehavior():
            Behavior("ptp") {}

        virtual ~PTPBehavior() = default;

        bool configure(const YAML::Node& params);

        void reset();

        void runOnce(const ContextData& context_data, BehaviorFeedback& fb);

        bool preConditionSatisfied(
                const ContextData& context_data,
                std::shared_ptr<Action> action) const;

        bool postConditionSatisfied(const ContextData& context_data) const;

    protected:

        const ContextData* context_data_{nullptr};
        TrajectoryPoint goal_;
        TrajectoryPoint current_;
        kelo::geometry_common::TransformMatrix2D localisation_tf_;
        bool is_unicycle_{false};
        std::vector<float> u_;
        kelo::geometry_common::PointCloud2D laser_pts_;
        CostFunction cf_;

        bool shorten_control_horizon_{true};
        size_t shortened_control_horizon_size_{1};
        float weight_laser_pts_repulsion_{0.0f};
        float min_inflation_dist_{0.1f};
        float max_inflation_dist_{10.0f};
        float robot_to_goal_dist_{3.0f};
        float reverse_motion_goal_threshold_linear_{0.2f};
        float reverse_motion_goal_threshold_heading_{M_PI/2};

        bool use_geometric_planner_{false};
        GeometricPlanner geometric_planner_;
        float geometric_planner_goal_tolerance_linear_{0.2f};
        float geometric_planner_goal_tolerance_angular_{1.0f};
        float geometric_planner_use_goal_theta_{false};

        kelo::geometry_common::XYTheta normal_state_pos_weights_;
        kelo::geometry_common::XYTheta normal_state_vel_weights_;
        float weight_goal_state_intercept_angle_{0.0f};

        bool use_initial_guess_utils_{true};
        InitialGuessUtils initial_guess_utils_;
        Trajectory initial_guess_trajectory_;

        bool use_dynamic_obstacles_{false};
        std::vector<float> initial_guess_sample_times_;
        std::vector<std::vector<kelo::geometry_common::Circle>> future_obstacles_;
        std::vector<std::vector<kelo::geometry_common::Circle>> future_initial_guess_obstacles_;
        kelo::geometry_common::PointCloud2D static_pts_;
        ObstacleTracker obstacle_tracker_;

        PTPBehavior(const std::string& type):
            Behavior(type) {}

        float calcCost(const std::vector<float>& u);

        void calcInitialU(std::vector<float>& u, BehaviorFeedback& fb);

        virtual void planGeometricPath(std::vector<visualization_msgs::Marker>& markers);

        bool parseNormalStateWeights(const YAML::Node& config);

        virtual TrajectoryPoint calcGoal(
                const ContextData& context_data, BehaviorFeedback& fb);

};

} // namespace cabin
