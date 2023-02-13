#ifndef CABIN_DOOR_behavior_H
#define CABIN_DOOR_behavior_H

#include <deque>

#include <cabin_nav/behavior/behavior.h>
#include <cabin_nav/utils/initial_guess_utils.h>

namespace cabin {

class DoorBehavior : public Behavior
{
    public:

        DoorBehavior():
            Behavior("door") {}

        virtual ~DoorBehavior() = default;

        bool configure(const YAML::Node& params);

        void reset();

        void runOnce(const ContextData& context_data, BehaviorFeedback& fb);

        bool preConditionSatisfied(
                const ContextData& context_data, std::shared_ptr<Action> action) const;

        bool postConditionSatisfied(const ContextData& context_data) const;

    protected:

        const ContextData* context_data_{nullptr};
        std::vector<float> u_;
        kelo::geometry_common::LineSegment2D door_;
        kelo::geometry_common::LineSegment2D ideal_path_;
        TrajectoryPoint goal_;
        TrajectoryPoint current_;
        kelo::geometry_common::TransformMatrix2D localisation_tf_;
        kelo::geometry_common::PointCloud2D laser_pts_;
        CostFunction cf_;

        float inflation_dist_{0.05f};
        float weight_laser_pts_repulsion_{0.0f};
        float weight_ideal_path_perp_dist_{0.0f};
        float ideal_path_length_after_door_{2.0f};
        float pre_door_goal_dist_{1.5f};
        float pre_door_min_dist_{1.0f};
        float pre_door_max_dist_{2.5f};
        float post_door_min_dist_x_{0.4f};
        float goal_tolerance_heading_{1.0f};
        float goal_tolerance_angular_{0.8f};
        float avg_pts_dist_threshold_{0.1f};

        float door_mismatch_threshold_{0.3f};
        size_t retry_counter_{0}, retry_threshold_{0};

        std::deque<kelo::geometry_common::LineSegment2D> door_buffer_;
        size_t buffer_size_{5};

        bool use_initial_guess_utils_{true};
        InitialGuessUtils initial_guess_utils_;
        Trajectory initial_guess_trajectory_;

        DoorBehavior(const std::string& type):
            Behavior(type) {}

        float calcCost(const std::vector<float>& u);

        void calcInitialU(std::vector<float>& u, BehaviorFeedback& fb);

        bool updateDoorFrame();

        bool findDoorFrameMap();

        void calcAveragedDoorFrame();

        void failWithRetry(
                BehaviorFeedback& fb,
                const std::string& failure_code = "");

        // bool findDoorFramePerception(visualization_msgs::MarkerArray& marker_array);

};

} // namespace cabin

#endif // CABIN_DOOR_behavior_H
