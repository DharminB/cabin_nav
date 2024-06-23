#pragma once

#include <geometry_common/Pose2D.h>
#include <geometry_common/Circle.h>
#include <geometry_common/Point2D.h>

#include <cabin_nav/structs/moving_circle.h>

namespace cabin {

class ObstacleTracker
{
    public:

        ObstacleTracker() = default;

        virtual ~ObstacleTracker() = default;

        void reset();

        void configure(float sample_time,
                       float cluster_distance_threshold = 0.1f,
                       size_t min_cluster_size = 5,
                       float max_cluster_length = 0.5f,
                       float max_tracking_dist = 5.0f,
                       float ransac_circle_fitting_threshold = 0.02f,
                       size_t ransac_circle_fitting_itr_limit = 10,
                       float min_ransac_circle_fitting_score = 0.7f,
                       float min_circle_radius = 0.1f,
                       float max_circle_radius = 0.4f,
                       float max_obstacle_velocity = 2.0f);

        void calcObstacles(
                const kelo::geometry_common::Pose2D& current_pose,
                const kelo::geometry_common::PointCloud2D& current_laser_pts,
                std::vector<visualization_msgs::Marker>& markers,
                std::vector<MovingCircle>& moving_obstacles,
                kelo::geometry_common::PointCloud2D& remaining_pts);

        void calcFutureObstacles(
                const std::vector<float>& sample_times,
                const std::vector<MovingCircle>& obstacles,
                std::vector<std::vector<kelo::geometry_common::Circle>>& future_obstacles);

    private:
        std::vector<MovingCircle> prev_moving_obstacles_;
        kelo::geometry_common::Pose2D old_pose_;
        bool old_pose_valid_{false};

        float sample_time_{0.2f};

        float cluster_distance_threshold_;
        size_t min_cluster_size_;
        float max_cluster_length_;

        float max_tracking_dist_;

        float ransac_circle_fitting_threshold_;
        size_t ransac_circle_fitting_itr_limit_;
        float min_ransac_circle_fitting_score_;

        float min_circle_radius_;
        float max_circle_radius_;
        float max_obstacle_travel_dist_;
};

} // namespace cabin
