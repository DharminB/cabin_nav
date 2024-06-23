#pragma once

#include <geometry_common/LineSegment2D.h>

#include <cabin_nav/behavior/behavior.h>
#include <cabin_nav/utils/initial_guess_utils.h>

namespace cabin {

class CorridorBehavior : public Behavior
{
    public:

        CorridorBehavior():
            Behavior("corridor") {}

        virtual ~CorridorBehavior() = default;

        bool configure(const YAML::Node& config);

        void reset();

        void runOnce(const ContextData& context_data, BehaviorFeedback& fb);

        bool preConditionSatisfied(
                const ContextData& context_data,
                std::shared_ptr<Action> action) const;

        bool postConditionSatisfied(const ContextData& context_data) const;

    protected:

        const ContextData* context_data_{nullptr};
        kelo::geometry_common::LineSegment2D left_wall_, right_wall_, ideal_path_;
        kelo::geometry_common::PointCloud2D remaining_pts_;
        std::vector<float> u_;
        TrajectoryPoint goal_;
        TrajectoryPoint current_;
        kelo::geometry_common::PointCloud2D laser_pts_;
        CostFunction cf_;

        kelo::geometry_common::LineSegment2D robot_line_;
        float wall_inflation_dist_{0.0f};

        float inflation_dist_{0.1f};
        float weight_ideal_path_perp_dist_{0.0f};
        float weight_left_wall_repulsion_{0.0f};
        float weight_right_wall_repulsion_{0.0f};
        float weight_remaining_laser_pts_repulsion_{0.0f};
        float weight_linear_vel_ellipse_{0.0f};
        float lane_percentage_{0.5f};
        float ideal_path_length_{5.0f};
        float line_fitting_ransac_delta_{0.1f};
        float goal_tolerance_heading_pre_{0.8f};
        float goal_tolerance_heading_post_{0.8f};
        float goal_tolerance_linear_{3.0f};

        size_t retry_counter_{0}, retry_threshold_{0};

        bool use_initial_guess_utils_{true};
        InitialGuessUtils initial_guess_utils_;
        Trajectory initial_guess_trajectory_;

        CorridorBehavior(const std::string& type):
            Behavior(type) {}

        float calcCost(const std::vector<float>& u);

        void calcInitialU(std::vector<float>& u, BehaviorFeedback& fb);

        float calcWallCost(
                const kelo::geometry_common::LineSegment2D& wall,
                const TrajectoryPoint& s);

        float calcWallCostBoxFootprint(
                const kelo::geometry_common::LineSegment2D& wall,
                const TrajectoryPoint& s);

        float calcWallCostCircleFootprint(
                const kelo::geometry_common::LineSegment2D& wall,
                const TrajectoryPoint& s);

        bool findWalls(
                kelo::geometry_common::LineSegment2D& left_wall,
                kelo::geometry_common::LineSegment2D& right_wall,
                kelo::geometry_common::PointCloud2D& remaining_pts);

        void failWithRetry(
                BehaviorFeedback& fb,
                const std::string& failure_code="");

        bool perceive(BehaviorFeedback& fb);

};

} // namespace cabin
