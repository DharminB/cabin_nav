#ifndef CABIN_UTILS_H
#define CABIN_UTILS_H

#include <vector>
#include <string>

#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>

#include <geometry_common/Box2D.h>
#include <geometry_common/Point2D.h>
#include <geometry_common/Point3D.h>
#include <geometry_common/Pose2D.h>
#include <geometry_common/Circle.h>
#include <geometry_common/LineSegment2D.h>
#include <geometry_common/TransformMatrix2D.h>

#include <cabin_nav/structs/trajectory_point.h>

namespace cabin {

class Utils
{
    public:

        static bool isWithinTolerance(
                const kelo::geometry_common::Pose2D& pose,
                const kelo::geometry_common::Pose2D& goal,
                float linear_tolerance,
                float angular_tolerance);

        static bool isTrajectorySafe(
                const Trajectory& trajectory,
                const kelo::geometry_common::Polygon2D& footprint,
                const kelo::geometry_common::PointCloud2D& laser_pts);

        static int calcCollisionIndex(
                const Trajectory& trajectory,
                const kelo::geometry_common::Polygon2D& footprint,
                const kelo::geometry_common::PointCloud2D& laser_pts);

        static void matMulNxN(
                const std::vector<float>& mat_a,
                const std::vector<float>& mat_b,
                std::vector<float>& mat_c,
                size_t N);

        static void matMulNxNToVector(
                const std::vector<float>& mat_a,
                const std::vector<float>& vec_b,
                std::vector<float>& vec_c,
                size_t N);

        static void outerVecMulNxN(
                const std::vector<float>& vec_a,
                const std::vector<float>& vec_b,
                std::vector<float>& mat_c,
                size_t N);

        static float innerVecMulNxN(
                const std::vector<float>& vec_a,
                const std::vector<float>& vec_b,
                size_t N);

        static void matMulNxNScalar(
                std::vector<float>& mat,
                float scalar);

        static void matAddNxN(
                const std::vector<float>& mat_a,
                const std::vector<float>& mat_b,
                std::vector<float>& mat_c);

        static void matSubNxN(
                const std::vector<float>& mat_a,
                const std::vector<float>& mat_b,
                std::vector<float>& mat_c);

        /**
         * @brief Find intersection point between two lines
         *
         *     c2 - c1
         * x = ------- ,    y = m*x + c
         *     m1 - m2
         *
         * @param m1 slope of first line
         * @param c1 constant of first line
         * @param m2 slope of second line
         * @param c2 constant of second line
         *
         * @return intersection point
         */
        static kelo::geometry_common::Point2D calcIntersectionPt(
                float m1,
                float c1,
                float m2,
                float c2);

        static std::vector<std::vector<int>> generateBFIndexes(
                int max_limit,
                int min_limit,
                size_t size);

        static std::vector<std::vector<int>> generateBFIndexes(
                const std::vector<int>& max_limits,
                const std::vector<int>& min_limits);

        /**
         * @brief transform acceleration from load to robot frame
         *
         * ASSUMPTION: load acceleration is only two dimensional (x and theta)
         * whereas robot acceleration is three dimensional (x, y and theta)
         *
         * @param load_tf Trasformation between load and robot's base_link
         * @param load_u acceleration applied at load frame at different times\n
         * [acc_x_t0, acc_theta_t0, acc_x_t1, acc_theta_t1, ...]
         *
         * @return acceleration applied at robot's base link
         */
        static std::vector<float> transformAccFromLoadToRobotFrame(
                const kelo::geometry_common::TransformMatrix2D& load_tf,
                const std::vector<float>& load_u);

        static kelo::geometry_common::LineSegment2D calcIntersectingSide(
                const kelo::geometry_common::Polygon2D& polygon,
                const kelo::geometry_common::LineSegment2D& l);

        static kelo::geometry_common::Pose2D calcIntermediateDoorGoal(
                const kelo::geometry_common::LineSegment2D& door,
                const kelo::geometry_common::Point2D& other_area_center,
                bool towards_door,
                float dist_from_door = 2.0f);

        static float calcCircumcircleRadius(
                const kelo::geometry_common::Box2D& box);

        static kelo::geometry_common::PointCloud2D downSamplePts(
                const kelo::geometry_common::PointCloud2D& original_pts,
                float dist = 0.1f);

        static float calcDeterminant(
                const kelo::geometry_common::LineSegment2D& l,
                const kelo::geometry_common::Point2D& p);

        static nav_msgs::Path convertToROSPath(
                const Trajectory& trajectory,
                const std::string& frame);

        static std::vector<visualization_msgs::Marker> convertTrajectoryToMarkers(
                const Trajectory& trajectory,
                const std::string& frame = "base_link",
                float line_red = 0.0f,
                float line_green = 0.5f,
                float line_blue = 0.5f,
                float line_alpha = 1.0f,
                float line_width = 0.01f,
                float arrow_red = 0.0f,
                float arrow_green = 0.5f,
                float arrow_blue = 0.5f,
                float arrow_alpha = 1.0f,
                float arrow_length = 0.05f,
                float arrow_width = 0.01f);

};

} // namespace cabin

#endif // CABIN_UTILS_H
