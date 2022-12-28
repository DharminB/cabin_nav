#ifndef CABIN_GEOMETRIC_PLANNER_H
#define CABIN_GEOMETRIC_PLANNER_H

#include <yaml-cpp/yaml.h>

#include <geometry_common/Point2D.h>
#include <geometry_common/Circle.h>
#include <geometry_common/LineSegment2D.h>

#include <cabin_nav/structs/trajectory_point.h>
#include <cabin_nav/structs/occupancy_grid.h>
#include <cabin_nav/structs/neighbour_offset.h>

namespace cabin {

class GeometricPlanner
{
    public:
        GeometricPlanner() = default;

        virtual ~GeometricPlanner() = default;

        void configure(const YAML::Node& params);

        void reset();

        bool plan(
                const kelo::geometry_common::Pose2D& start,
                const kelo::geometry_common::Pose2D& goal,
                const kelo::geometry_common::PointCloud2D& laser_pts,
                kelo::geometry_common::Path& geometric_path,
                std::vector<visualization_msgs::Marker>& markers);

        void setFootprint(
                const kelo::geometry_common::Box2D& box_footprint,
                const kelo::geometry_common::Circle& circle_footprint,
                bool is_footprint_box);

        static kelo::geometry_common::Path calcArcPath(
                const kelo::geometry_common::Pose2D& start,
                float angular_dist,
                const kelo::geometry_common::Circle& circle,
                float angular_step_size = 0.25f);

        static kelo::geometry_common::PointVec2D calcStraightLinePath(
                const kelo::geometry_common::Point2D& start,
                const kelo::geometry_common::Point2D& goal,
                float step_size = 0.1f);

        static kelo::geometry_common::Path calcStraightLinePath(
                const kelo::geometry_common::Pose2D& start,
                const kelo::geometry_common::Pose2D& goal,
                float step_size = 0.1f);

        static float calculateCSCPath(
                const kelo::geometry_common::Pose2D& start,
                const kelo::geometry_common::Pose2D& goal,
                const kelo::geometry_common::Circle& circle_1,
                const kelo::geometry_common::Circle& circle_2,
                bool is_circle_1_clockwise,
                bool is_circle_2_clockwise,
                bool calculate_path,
                kelo::geometry_common::Path& dubin_path,
                float step_size = 0.2f);

        static bool calculateTangent(
                const kelo::geometry_common::Circle& circle_1,
                const kelo::geometry_common::Circle& circle_2,
                bool circle_1_clockwise,
                bool circle_2_clockwise,
                kelo::geometry_common::LineSegment2D& tangent);

        static float calcDubinsPath(
                const kelo::geometry_common::Pose2D& start,
                const kelo::geometry_common::Pose2D& goal,
                kelo::geometry_common::Path& pose_path,
                float min_turning_radius = 0.5f,
                float step_size = 0.2f);

        static float calcDubinsPathIteratively(
                const kelo::geometry_common::Pose2D& start,
                const kelo::geometry_common::Pose2D& goal,
                kelo::geometry_common::Path& pose_path,
                float min_turning_radius = 0.0f,
                float max_turning_radius = 1.0f,
                float turning_radius_step_size = 0.1f,
                float step_size = 0.2f);

        static bool planDubinsPathIteratively(
                const kelo::geometry_common::Pose2D& start,
                const kelo::geometry_common::Pose2D& goal,
                const OccupancyGrid::Ptr occ_grid,
                kelo::geometry_common::Path& geometric_path,
                float min_turning_radius = 0.01f,
                float max_turning_radius = 1.0f,
                float turning_radius_step_size = 0.1f,
                float step_size = 0.2f);

        static bool planSimplePath(
                const kelo::geometry_common::Pose2D& start,
                const kelo::geometry_common::Pose2D& goal,
                const kelo::geometry_common::PointCloud2D& laser_pts,
                const OccupancyGrid::Ptr occ_grid,
                kelo::geometry_common::Path& geometric_path,
                float step_size = 0.1f,
                float desired_obs_dist = 0.5f);

        static bool planSimpleDubinsPath(
                const kelo::geometry_common::Pose2D& start,
                const kelo::geometry_common::Pose2D& goal,
                const kelo::geometry_common::PointCloud2D& laser_pts,
                const OccupancyGrid::Ptr occ_grid,
                std::vector<kelo::geometry_common::Pose2D>& geometric_path,
                float min_turning_radius = 0.1f,
                float max_turning_radius = 1.0f,
                float turning_radius_step_size = 0.1f,
                float step_size = 0.2f,
                float desired_obs_dist = 0.5f);

        static int calcCollisionIndex(
                const OccupancyGrid::Ptr occ_grid,
                const kelo::geometry_common::Path& geometric_path);

        static int calcIndexAtDist(
                const kelo::geometry_common::Path& geometric_path,
                float linear_dist,
                float angular_dist = 6.28f);

        static int calcIndexAtDist(
                const kelo::geometry_common::PointVec2D& geometric_path,
                float linear_dist);


    protected:

        std::string type_;
        float goal_tolerance_linear_;
        float goal_tolerance_angular_;
        float grid_cell_size_;
        float grid_radius_;
        float turning_radius_;
        float linear_step_size_;
        float footprint_circumcircle_radius_;
        AllNeighbourOffsets neighbour_offsets_;
        OccupancyGrid::Ptr occ_grid_;

};

} // namespace cabin

#endif // CABIN_GEOMETRIC_PLANNER_H
