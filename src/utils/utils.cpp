#include <cmath>

#include <geometry_common/Polygon2D.h>
#include <geometry_common/Utils.h>

#include <cabin_nav/utils/utils.h>

using kelo::geometry_common::Box2D;
using kelo::geometry_common::Point2D;
using kelo::geometry_common::Vector2D;
using kelo::geometry_common::Point3D;
using kelo::geometry_common::Pose2D;
using kelo::geometry_common::Velocity2D;
using kelo::geometry_common::Acceleration2D;
using kelo::geometry_common::XYTheta;
using kelo::geometry_common::LineSegment2D;
using kelo::geometry_common::TransformMatrix2D;
using kelo::geometry_common::Circle;
using kelo::geometry_common::Polygon2D;
using kelo::geometry_common::Path;
using kelo::geometry_common::PointCloud2D;
using GCUtils = kelo::geometry_common::Utils;

namespace cabin {

bool Utils::isWithinTolerance(
        const Pose2D& pose,
        const Pose2D& goal,
        float linear_tolerance,
        float angular_tolerance)
{
    float dist = goal.distTo(pose);
    float ang_dist = GCUtils::calcShortestAngle(goal.theta, pose.theta);
    return ( dist < linear_tolerance && std::fabs(ang_dist) < angular_tolerance );
}

bool Utils::isTrajectorySafe(
        const Trajectory& trajectory,
        const Polygon2D& footprint,
        const PointCloud2D& laser_pts)
{
    return ( Utils::calcCollisionIndex(trajectory, footprint, laser_pts) == -1 );
}

int Utils::calcCollisionIndex(
        const Trajectory& trajectory,
        const Polygon2D& footprint,
        const PointCloud2D& laser_pts)
{
    for ( size_t i = 0; i < trajectory.size(); i++ )
    {
        const Polygon2D traj_footprint = TransformMatrix2D(trajectory[i].pos) * footprint;
        if ( traj_footprint.containsAnyPoint(laser_pts) )
        {
            return i;
        }
    }
    return -1;
}

void Utils::matMulNxN(
        const std::vector<float>& mat_a,
        const std::vector<float>& mat_b,
        std::vector<float>& mat_c,
        size_t N)
{
    for ( size_t i = 0; i < N; i++ )
    {
        for ( size_t j = 0; j < N; j++ )
        {
            mat_c[i*N + j] = 0.0f;
            for ( size_t k = 0; k < N; k++ )
            {
                mat_c[i*N + j] += mat_a[i*N + k] * mat_b[k*N + j];
            }
        }
    }
}

void Utils::matMulNxNToVector(
        const std::vector<float>& mat_a,
        const std::vector<float>& vec_b,
        std::vector<float>& vec_c,
        size_t N)
{
    for ( size_t i = 0; i < N; i++ )
    {
        vec_c[i] = 0.0f;
        for ( size_t j = 0; j < N; j++ )
        {
            vec_c[i] += mat_a[i*N + j] * vec_b[j];
        }
    }
}

void Utils::outerVecMulNxN(
        const std::vector<float>& vec_a,
        const std::vector<float>& vec_b,
        std::vector<float>& mat_c,
        size_t N)
{
    for ( size_t i = 0; i < N; i++ )
    {
        for ( size_t j = 0; j < N; j++ )
        {
            mat_c[i*N + j] = vec_a[i] * vec_b[j];
        }
    }
}

float Utils::innerVecMulNxN(
        const std::vector<float>& vec_a,
        const std::vector<float>& vec_b,
        size_t N)
{
    float dot_product = 0.0f;
    for ( size_t i = 0; i < N; i++ )
    {
        dot_product += vec_a[i] * vec_b[i];
    }
    return dot_product;
}

void Utils::matMulNxNScalar(
        std::vector<float>& mat,
        float scalar)
{
    for ( size_t i = 0; i < mat.size(); i++ )
    {
        mat[i] *= scalar;
    }
}

void Utils::matAddNxN(
        const std::vector<float>& mat_a,
        const std::vector<float>& mat_b,
        std::vector<float>& mat_c)
{
    for ( size_t i = 0; i < mat_a.size(); i++ )
    {
        mat_c[i] = mat_a[i] + mat_b[i];
    }
}

void Utils::matSubNxN(
        const std::vector<float>& mat_a,
        const std::vector<float>& mat_b,
        std::vector<float>& mat_c)
{
    for ( size_t i = 0; i < mat_a.size(); i++ )
    {
        mat_c[i] = mat_a[i] - mat_b[i];
    }
}

Point2D Utils::calcIntersectionPt(
        float m1,
        float c1,
        float m2,
        float c2)
{
    Point2D intersection_pt;
    intersection_pt.x = (c2 - c1) / (m1 - m2);
    intersection_pt.y = (m1 * intersection_pt.x) + c1;
    return intersection_pt;
}

std::vector<std::vector<int>> Utils::generateBFIndexes(
        int max_limit,
        int min_limit,
        size_t size)
{
    std::vector<std::vector<int> > indexes;
    if ( min_limit > max_limit )
    {
        return indexes;
    }
    std::vector<int> max_limits(size, max_limit);
    std::vector<int> min_limits(size, min_limit);
    return Utils::generateBFIndexes(max_limits, min_limits);
}

std::vector<std::vector<int>> Utils::generateBFIndexes(
        const std::vector<int>& max_limits,
        const std::vector<int>& min_limits)
{
    std::vector<std::vector<int>> indexes;
    if ( max_limits.size() != min_limits.size() )
    {
        return indexes;
    }

    for ( size_t i = 0; i < max_limits.size(); i++ )
    {
        if ( min_limits[i] > max_limits[i] )
        {
            return indexes;
        }
    }

    std::vector<int> ind(min_limits);
    while ( ind[0] < max_limits[0]+1 )
    {
        indexes.push_back(std::vector<int>(ind));
        ind[ind.size()-1] ++;
        size_t i = ind.size()-1;
        while ( i > 0 && ind[i] > max_limits[i] )
        {
            ind[i] = min_limits[i];
            i--;
            ind[i]++;
        }
    }
    return indexes;
}

std::vector<float> Utils::transformAccFromLoadToRobotFrame(
        const kelo::geometry_common::TransformMatrix2D& load_tf,
        const std::vector<float>& load_u)
{
    size_t control_horizon = load_u.size()/2;
    std::vector<float> robot_u(control_horizon * 3);
    for ( size_t i = 0; i < control_horizon; i++ )
    {
        robot_u[(i*3)  ] = (load_tf[0] * load_u[i*2]) + (load_tf[5] * load_u[(i*2)+1]);
        robot_u[(i*3)+1] = (load_tf[3] * load_u[i*2]) - (load_tf[2] * load_u[(i*2)+1]);
        robot_u[(i*3)+2] = load_u[(i*2)+1];
    }
    return robot_u;
}

LineSegment2D Utils::calcIntersectingSide(
        const Polygon2D& polygon,
        const LineSegment2D& l)
{
    for ( size_t i = 0; i < polygon.size(); i++ )
    {
        size_t j = i+1;
        if ( j == polygon.size() )
        {
            j = 0;
        }
        LineSegment2D side(polygon[i], polygon[j]);
        if ( side.intersects(l) )
        {
            return side;
        }
    }
    return LineSegment2D();
}

Pose2D Utils::calcIntermediateDoorGoal(
        const LineSegment2D& door,
        const Point2D& other_area_center,
        bool towards_door,
        float dist_from_door)
{
    const Pose2D door_center_pose(door.center(),
                                  GCUtils::calcPerpendicularAngle(door.angle()));
    const TransformMatrix2D door_center_tf_mat = door_center_pose.asMat();
    const Point2D forward_pt = door_center_tf_mat * Point2D(dist_from_door, 0.0f);
    const Point2D backward_pt = door_center_tf_mat * Point2D(-dist_from_door, 0.0f);

    Point2D closer_pt = ( forward_pt.distTo(other_area_center) < backward_pt.distTo(other_area_center) )
                        ? forward_pt : backward_pt;
    Pose2D intermediate_goal(closer_pt);
    intermediate_goal.theta = (door_center_pose.position() - closer_pt).angle();
    if ( !towards_door )
    {
        intermediate_goal.theta = GCUtils::calcReverseAngle(intermediate_goal.theta);
    }
    return intermediate_goal;
}

float Utils::calcCircumcircleRadius(
        const Box2D& box)
{
    Vector2D boundary;
    boundary.x = std::max(std::fabs(box.max_x), std::fabs(box.min_x));
    boundary.y = std::max(std::fabs(box.max_y), std::fabs(box.min_y));
    return boundary.magnitude();
}

PointCloud2D Utils::downSamplePts(
        const PointCloud2D& original_pts,
        float dist)
{
    if ( original_pts.size() <= 2 )
    {
        return PointCloud2D(original_pts);
    }

    PointCloud2D pts;
    pts.reserve(original_pts.size());
    pts.push_back(original_pts.front()); // add first point
    float dist_sq = dist * dist;
    for ( size_t i = 1; i+1 < original_pts.size(); i++ )
    {
        if ( pts.back().squaredDistTo(original_pts[i]) > dist_sq ||
             original_pts[i+1].squaredDistTo(original_pts[i]) > dist_sq )
        {
            pts.push_back(original_pts[i]);
        }
    }
    pts.push_back(original_pts.back()); // add last point
    return pts;
}

float Utils::calcDeterminant(
        const LineSegment2D& l,
        const Point2D& p)
{
    return (l.end - l.start).scalarCrossProduct(p - l.start);
}

nav_msgs::Path Utils::convertToROSPath(
        const Trajectory& trajectory,
        const std::string& frame)
{
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = frame;
    path_msg.poses.reserve(trajectory.size());

    for ( const TrajectoryPoint& state : trajectory )
    {
        path_msg.poses.push_back(state.pos.asPoseStamped(frame));
    }
    return path_msg;
}

std::vector<visualization_msgs::Marker> Utils::convertTrajectoryToMarkers(
        const Trajectory& trajectory,
        const std::string& frame,
        float line_red,
        float line_green,
        float line_blue,
        float line_alpha,
        float line_width,
        float arrow_red,
        float arrow_green,
        float arrow_blue,
        float arrow_alpha,
        float arrow_length,
        float arrow_width)
{
    std::vector<visualization_msgs::Marker> markers;
    markers.reserve(trajectory.size() + 1);

    // line strip
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.header.frame_id = frame;
    marker.color.r = line_red;
    marker.color.g = line_green;
    marker.color.b = line_blue;
    marker.color.a = line_alpha;
    marker.scale.x = line_width;
    marker.pose.orientation.w = 1.0f;
    marker.points.reserve(trajectory.size());
    for ( size_t i = 0; i < trajectory.size(); i++ )
    {
        geometry_msgs::Point pt;
        pt.x = trajectory[i].pos.x;
        pt.y = trajectory[i].pos.y;
        marker.points.push_back(pt);
    }
    markers.push_back(marker);

    // individual arrows
    for ( size_t i = 0; i < trajectory.size(); i++ )
    {
        markers.push_back(trajectory[i].pos.asMarker(
                    frame, arrow_red, arrow_green, arrow_blue, arrow_alpha,
                    arrow_length, arrow_width, arrow_width));
    }
    return markers;
}

} // namespace cabin
