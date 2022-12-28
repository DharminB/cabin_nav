#include <geometry_common/Utils.h>
#include <geometry_common/TransformMatrix2D.h>

#include <cabin_nav/utils/obstacle_tracker.h>

using kelo::geometry_common::Circle;
using kelo::geometry_common::Pose2D;
using kelo::geometry_common::Point2D;
using kelo::geometry_common::PointCloud2D;
using kelo::geometry_common::TransformMatrix2D;
using GCUtils = kelo::geometry_common::Utils;

namespace cabin {

void ObstacleTracker::reset()
{
    old_pose_valid_ = false;
    old_pose_ = Pose2D();
    prev_moving_obstacles_.clear();
}

void ObstacleTracker::configure(
        float sample_time,
        float cluster_distance_threshold,
        size_t min_cluster_size,
        float max_cluster_length,
        float max_tracking_dist,
        float ransac_circle_fitting_threshold,
        size_t ransac_circle_fitting_itr_limit,
        float min_ransac_circle_fitting_score,
        float min_circle_radius,
        float max_circle_radius,
        float max_obstacle_velocity)
{
    sample_time_ = sample_time;
    cluster_distance_threshold_ = cluster_distance_threshold;
    min_cluster_size_ = min_cluster_size;
    max_cluster_length_ = max_cluster_length;
    max_tracking_dist_ = max_tracking_dist;
    min_ransac_circle_fitting_score_ = min_ransac_circle_fitting_score;
    min_circle_radius_ = min_circle_radius;
    max_circle_radius_ = max_circle_radius;
    max_obstacle_travel_dist_ = max_obstacle_velocity * sample_time_;
    ransac_circle_fitting_threshold_ = ransac_circle_fitting_threshold;
    ransac_circle_fitting_itr_limit_ = ransac_circle_fitting_itr_limit;
}

void ObstacleTracker::calcObstacles(
        const kelo::geometry_common::Pose2D& current_pose,
        const kelo::geometry_common::PointCloud2D& current_laser_pts,
        std::vector<visualization_msgs::Marker>& markers,
        std::vector<MovingCircle>& moving_obstacles,
        kelo::geometry_common::PointCloud2D& remaining_pts)
{
    Point2D robot_pt;
    remaining_pts.clear();
    remaining_pts.reserve(current_laser_pts.size()); // guess
    moving_obstacles.clear();
    moving_obstacles.reserve(prev_moving_obstacles_.size()); // guess

    std::vector<PointCloud2D> clusters = GCUtils::clusterOrderedPoints(
            current_laser_pts, cluster_distance_threshold_, 1);

    /* fit circles to clusters wherever possible */
    for ( const PointCloud2D& cluster : clusters )
    {
        if ( cluster.size() > min_cluster_size_ )
        {
            float cluster_length = cluster.front().distTo(cluster.back());
            Point2D centroid = GCUtils::calcMeanPoint(cluster);
            if ( cluster_length < max_cluster_length_ &&
                 robot_pt.distTo(centroid) < max_tracking_dist_ )
            {
                Circle circle;
                float score = GCUtils::fitCircleRANSAC(
                        cluster, circle, ransac_circle_fitting_threshold_,
                        ransac_circle_fitting_itr_limit_);
                if ( score > min_ransac_circle_fitting_score_ &&
                     circle.r > min_circle_radius_ &&
                     circle.r < max_circle_radius_ )
                {
                    moving_obstacles.push_back(MovingCircle(circle));
                    continue;
                }
            }
        }

        remaining_pts.insert(remaining_pts.end(), cluster.begin(), cluster.end());
    }

    if ( !prev_moving_obstacles_.empty() && !moving_obstacles.empty() && old_pose_valid_ )
    {
        TransformMatrix2D movement_tf = TransformMatrix2D(current_pose).calcInverse() *
                                        TransformMatrix2D(old_pose_);

        for ( Circle& obs : prev_moving_obstacles_ )
        {
            movement_tf.transform(obs);
            markers.push_back(obs.asMarker("", 1.0f, 0.0, 0.0f, 1.0f));
        }

        /* nearest neighbour calculation to assign velocities */
        for ( size_t i = 0; i < prev_moving_obstacles_.size(); i++ )
        {
            size_t nearest_index = 0;
            float min_dist = std::numeric_limits<float>::max();
            bool found = false;
            for ( size_t j = 0; j < moving_obstacles.size(); j++ )
            {
                /* if already has valid velocities */
                if ( std::fabs(moving_obstacles[j].vel.x) > 1e-3f ||
                     std::fabs(moving_obstacles[j].vel.y) > 1e-3f )
                {
                    continue;
                }

                float dist = prev_moving_obstacles_[i].distTo(moving_obstacles[j]);
                if ( dist < min_dist && dist < max_obstacle_travel_dist_ )
                {
                    min_dist = dist;
                    nearest_index = j;
                    found = true;
                }
            }

            if ( found )
            {
                moving_obstacles[nearest_index].vel.x =
                    (moving_obstacles[nearest_index].x - prev_moving_obstacles_[i].x) / sample_time_;
                moving_obstacles[nearest_index].vel.y =
                    (moving_obstacles[nearest_index].y - prev_moving_obstacles_[i].y) / sample_time_;
            }
        }
    }
    for ( size_t i = 0; i < moving_obstacles.size(); i++ )
    {
        markers.push_back(moving_obstacles[i].asMarker("", 0.33f, 0.66f, 0.5, 1.0f));
    }

    prev_moving_obstacles_ = moving_obstacles;
    old_pose_valid_ = true;
    old_pose_ = current_pose;
}

void ObstacleTracker::calcFutureObstacles(
        const std::vector<float>& sample_times,
        const std::vector<MovingCircle>& obstacles,
        std::vector<std::vector<Circle>>& future_obstacles)
{
    future_obstacles = std::vector<std::vector<Circle>>(sample_times.size());
    float sample_time = 0.0f;
    for ( size_t i = 0; i < sample_times.size(); i++ )
    {
        sample_time += sample_times[i];
        future_obstacles[i].reserve(obstacles.size());
        for ( const MovingCircle& mc : obstacles )
        {
            future_obstacles[i].push_back(
                    Circle(
                        mc.x + (mc.vel.x * sample_time),
                        mc.y + (mc.vel.y * sample_time),
                        mc.r));
        }
    }
}

} // namespace cabin
