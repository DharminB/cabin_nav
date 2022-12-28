#include <cmath>
#include <chrono>

#include <geometry_common/Utils.h>
#include <yaml_common/Parser2.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/utils/utils.h>
#include <cabin_nav/utils/lattice_search_utils.h>

#include <cabin_nav/utils/geometric_planner.h>

using kelo::geometry_common::Box2D;
using kelo::geometry_common::Pose2D;
using kelo::geometry_common::Path;
using kelo::geometry_common::Point2D;
using kelo::geometry_common::Vector2D;
using kelo::geometry_common::Circle;
using kelo::geometry_common::PointVec2D;
using kelo::geometry_common::PointCloud2D;
using kelo::geometry_common::LineSegment2D;
using GCUtils = kelo::geometry_common::Utils;
using Parser = kelo::yaml_common::Parser2;

namespace cabin {

void GeometricPlanner::configure(const YAML::Node& config)
{
    type_ = Parser::get<std::string>(config, "type", "simple_straight");

    goal_tolerance_linear_ = Parser::get<float>(config, "goal_tolerance_linear", 0.2f);
    goal_tolerance_angular_ = Parser::get<float>(config, "goal_tolerance_angular", 1.0f);

    grid_cell_size_ = Parser::get<float>(config, "grid_cell_size", 0.2f);
    grid_radius_ = Parser::get<float>(config, "grid_radius", 5.0f);
    turning_radius_ = Parser::get<float>(config, "turning_radius", 0.5f);
    linear_step_size_ = Parser::get<float>(config, "step_size", 0.2f);

    /* calc neighbour offset */
    if ( type_ == "astar_grid" )
    {
        neighbour_offsets_ = LatticeSearchUtils::calcNeighbourOffset(
                grid_cell_size_, 1, 0, 0, "");
    }
    else if ( type_ == "astar_lattice" )
    {
        size_t num_of_angles = Parser::get<float>(config, "num_of_angles", 16);
        int max_angle_offset = Parser::get<int>(
                config, "max_angle_offset", num_of_angles/8);
        std::string kinematic_model = Parser::get<std::string>(
                config, "kinematic_model", "bicycle");
        neighbour_offsets_ = LatticeSearchUtils::calcNeighbourOffset(
                grid_cell_size_, num_of_angles, turning_radius_,
                max_angle_offset, kinematic_model);
    }
    float occ_grid_radius = Parser::get<float>(config, "occ_grid_radius", 5.0f);
    float occ_grid_cell_size = Parser::get<float>(config, "occ_grid_cell_size", 0.2f);
    occ_grid_ = std::make_shared<OccupancyGrid>(occ_grid_radius, occ_grid_cell_size);
}

void GeometricPlanner::reset()
{
    if ( occ_grid_ != nullptr )
    {
        occ_grid_->clear();
    }
}

void GeometricPlanner::setFootprint(
        const Box2D& box_footprint,
        const Circle& circle_footprint,
        bool is_footprint_box)
{
    if ( is_footprint_box )
    {
        occ_grid_->generateRobotPts(box_footprint);
        footprint_circumcircle_radius_ = Utils::calcCircumcircleRadius(box_footprint);
    }
    else
    {
        occ_grid_->generateRobotPts(circle_footprint);
        footprint_circumcircle_radius_ = circle_footprint.r;
    }
}

bool GeometricPlanner::plan(const Pose2D& start, const Pose2D& goal,
                            const PointCloud2D& laser_pts,
                            Path& geometric_path,
                            std::vector<visualization_msgs::Marker>& markers)
{
    float dist_to_goal = start.distTo(goal);

    geometric_path.clear();

    occ_grid_->clear();
    occ_grid_->update(laser_pts);

    // markers.push_back(occ_grid_->getMarker());

    if ( type_ == "simple_straight" )
    {
        return GeometricPlanner::planSimplePath(start, goal,
                laser_pts, occ_grid_, geometric_path, linear_step_size_,
                footprint_circumcircle_radius_);
    }

    else if ( type_ == "simple_dubins" )
    {
        if ( dist_to_goal < 2.0f * turning_radius_ )
        {
            return true;
        }
        else
        {
            return GeometricPlanner::planSimpleDubinsPath(start, goal,
                    laser_pts, occ_grid_, geometric_path,
                    turning_radius_, 1.0f, 0.1f, linear_step_size_,
                    footprint_circumcircle_radius_);
        }
    }

    else if ( type_ == "dubins_iterative" )
    {
        if ( dist_to_goal < 2.0f * turning_radius_ )
        {
            return true;
        }
        else
        {
            return GeometricPlanner::planDubinsPathIteratively(start, goal,
                    occ_grid_, geometric_path, turning_radius_, 1.0f, 0.1f,
                    linear_step_size_);
        }
    }


    else if ( type_ == "astar_grid" )
    {
        bool success = LatticeSearchUtils::searchLocal(
                start, goal, occ_grid_, neighbour_offsets_,
                grid_cell_size_, grid_radius_, geometric_path);
        if ( success )
        {
            for ( Pose2D& p : geometric_path )
            {
                p.theta = std::atan2(goal.y - p.y, goal.x - p.x);
            }
            geometric_path.push_back(goal);
        }
        return success;
    }

    else if ( type_ == "astar_lattice" )
    {
        return LatticeSearchUtils::searchLocal(
                start, goal, occ_grid_, neighbour_offsets_,
                grid_cell_size_, grid_radius_, geometric_path);
    }
    return false;
}

Path GeometricPlanner::calcArcPath(const Pose2D& start,
        float angular_dist, const Circle& circle, float angular_step_size)
{
    Path pose_path;
    size_t num_of_poses = static_cast<size_t>(
            std::ceil(std::fabs(angular_dist)/angular_step_size));
    if ( num_of_poses == 0 )
    {
        return pose_path;
    }

    float delta_angle = angular_dist / num_of_poses;
    float angle_offset = (start.position() - circle).angle();//std::atan2(start.y - circle.y, start.x - circle.x);
    float theta_offset = ( delta_angle > 0 ) ? M_PI/2 : -M_PI/2;

    pose_path.reserve(num_of_poses+1);
    for ( size_t i = 0; i < num_of_poses+1; i++ )
    {
        Pose2D pose;
        float angle = (delta_angle * i) + angle_offset;
        pose.theta = GCUtils::clipAngle(angle + theta_offset);
        pose.updatePosition(circle.center() + Point2D::initFromRadialCoord(circle.r, angle));
        pose_path.push_back(pose);
    }
    return pose_path;
}

PointVec2D GeometricPlanner::calcStraightLinePath(
        const Point2D& start, const Point2D& goal, float step_size)
{
    Point2D diff = goal - start;
    float dist = start.distTo(goal);
    size_t num_of_pt = static_cast<size_t>(std::ceil(dist/step_size));
    PointVec2D pt_path;
    if ( num_of_pt == 0 )
    {
        return pt_path;
    }
    Point2D delta = diff / static_cast<float>(num_of_pt);
    pt_path.reserve(num_of_pt+1);
    for ( size_t i = 0; i < num_of_pt; i++ )
    {
        pt_path.push_back(start + (delta * i));
    }
    pt_path.push_back(goal);
    return pt_path;
}

Path GeometricPlanner::calcStraightLinePath(
        const Pose2D& start, const Pose2D& goal, float step_size)
{
    Point2D start_pt = start.position();
    Point2D goal_pt = goal.position();
    Point2D diff = goal_pt - start_pt;
    float dist = diff.magnitude();
    size_t num_of_poses = static_cast<size_t>(std::ceil(dist/step_size));
    Path pose_path;
    if ( num_of_poses == 0 )
    {
        return pose_path;
    }
    Point2D delta = diff / static_cast<float>(num_of_poses);
    float theta = (goal_pt - start_pt).angle();
    pose_path.reserve(num_of_poses+1);
    pose_path.push_back(start);
    for ( size_t i = 0; i < num_of_poses; i++ )
    {
        pose_path.push_back(Pose2D(start_pt + (delta * i), theta));
    }
    pose_path.push_back(goal);
    return pose_path;
}

float GeometricPlanner::calculateCSCPath(const Pose2D& start,
                                         const Pose2D& goal,
                                         const Circle& circle_1,
                                         const Circle& circle_2,
                                         bool is_circle_1_clockwise,
                                         bool is_circle_2_clockwise,
                                         bool calculate_path,
                                         Path& dubin_path,
                                         float step_size)
{
    LineSegment2D tangent;
    if ( !calculateTangent(circle_1, circle_2, is_circle_1_clockwise,
                           is_circle_2_clockwise, tangent) )
    {
        return -1;
    }

    float path_length = tangent.length();
    float tangent_angle = tangent.angle();
    float start_angle_diff = GCUtils::calcShortestAngle(tangent_angle, start.theta);
    if ( start_angle_diff < 0.0f && !is_circle_1_clockwise )
    {
        start_angle_diff += 2 * M_PI;
    }
    if ( start_angle_diff > 0.0f && is_circle_1_clockwise )
    {
        start_angle_diff -= 2 * M_PI;
    }
    path_length += std::fabs(start_angle_diff) * circle_1.r;

    float goal_angle_diff = GCUtils::calcShortestAngle(goal.theta, tangent_angle);
    if ( goal_angle_diff < 0.0f && !is_circle_2_clockwise )
    {
        goal_angle_diff += 2 * M_PI;
    }
    if ( goal_angle_diff > 0.0f && is_circle_2_clockwise )
    {
        goal_angle_diff -= 2 * M_PI;
    }
    path_length += std::fabs(goal_angle_diff) * circle_2.r;

    if ( calculate_path )
    {
        dubin_path.clear();
        Pose2D tangent_start(tangent.start, tangent_angle);
        Pose2D tangent_end(tangent.end, tangent_angle);

        Path start_curve_path = GeometricPlanner::calcArcPath(
                start, start_angle_diff, circle_1);
        Path straight_line_path = GeometricPlanner::calcStraightLinePath(
                tangent_start, tangent_end, step_size);
        Path goal_curve_path = GeometricPlanner::calcArcPath(
                tangent_end, goal_angle_diff, circle_2);

        dubin_path.reserve(start_curve_path.size()
                         + straight_line_path.size()
                         + goal_curve_path.size());
        dubin_path.insert(dubin_path.end(), start_curve_path.begin(),
                          start_curve_path.end());
        dubin_path.insert(dubin_path.end(), straight_line_path.begin(),
                          straight_line_path.end());
        dubin_path.insert(dubin_path.end(), goal_curve_path.begin(),
                          goal_curve_path.end());
    }

    return path_length;
}

bool GeometricPlanner::calculateTangent(const Circle& circle_1,
                                        const Circle& circle_2,
                                        bool is_circle_1_clockwise,
                                        bool is_circle_2_clockwise,
                                        LineSegment2D& tangent)
{
    /* source: https://github.com/gieseanw/Dubins/blob/master/Includes.cpp#L16 */
    float dist = circle_1.distTo(circle_2);
    Vector2D vec = (circle_2 - circle_1).asNormalised();

    int sign_1 = ( is_circle_1_clockwise == is_circle_2_clockwise ) ? 1 : -1;
    int sign_2 = ( is_circle_1_clockwise ) ? 1 : -1;

    float cosine = (circle_1.r - (sign_1 * circle_2.r)) / dist;
    if ( std::pow(cosine, 2) > 1.0f )
    {
        return false;
    }
    float sine = std::sqrt(1.0 - std::pow(cosine, 2));

    Vector2D norm_vec;
    norm_vec.x = (vec.x * cosine) - (sign_2 * sine * vec.y);
    norm_vec.y = (vec.y * cosine) + (sign_2 * sine * vec.x);

    tangent.start.x = circle_1.x + (circle_1.r * norm_vec.x);
    tangent.start.y = circle_1.y + (circle_1.r * norm_vec.y);
    tangent.end.x = circle_2.x + (sign_1 * circle_2.r * norm_vec.x);
    tangent.end.y = circle_2.y + (sign_1 * circle_2.r * norm_vec.y);
    return true;
}

float GeometricPlanner::calcDubinsPath(
        const Pose2D& start, const Pose2D& goal,
        Path& pose_path, float turning_radius, float step_size)
{
    float min_path_length = 1e6f;
    for ( size_t i = 0; i < 2; i++ )
    {
        for ( size_t j = 0; j < 2; j++ )
        {
            float start_angle_offset = ( i == 0 ) ? M_PI/2 : -M_PI/2;
            float goal_angle_offset = ( j == 0 ) ? M_PI/2 : -M_PI/2;
            Circle start_circle(start.position() +
                                Point2D::initFromRadialCoord(
                                    turning_radius, start.theta + start_angle_offset),
                                turning_radius);
            Circle goal_circle(goal.position() +
                               Point2D::initFromRadialCoord(
                                   turning_radius, goal.theta + goal_angle_offset),
                               turning_radius);

            float path_length = calculateCSCPath(start, goal, start_circle, goal_circle,
                                                 i, j, false, pose_path, step_size);

            if ( path_length >= 0.0f && path_length < min_path_length )
            {
                min_path_length = calculateCSCPath(start, goal, start_circle, goal_circle,
                                                   i, j, true, pose_path, step_size);
            }
        }
    }
    return min_path_length;
}

float GeometricPlanner::calcDubinsPathIteratively(
        const Pose2D& start,
        const Pose2D& goal,
        Path& pose_path,
        float min_turning_radius,
        float max_turning_radius,
        float turning_radius_step_size,
        float step_size)
{
    pose_path.clear();
    float last_path_length = start.distTo(goal);
    if ( last_path_length < 1e-3f )
    {
        return 0.0f;
    }

    for ( float turning_radius = min_turning_radius;
          GCUtils::roundFloat(turning_radius, 2) <= max_turning_radius;
          turning_radius += turning_radius_step_size )
    {
        float path_length = GeometricPlanner::calcDubinsPath(start, goal,
                pose_path, turning_radius, step_size);
        if ( path_length / last_path_length > 2.0f )
        {
            return GeometricPlanner::calcDubinsPath(start, goal,
                pose_path, turning_radius-turning_radius_step_size, step_size);
        }

        last_path_length = path_length;
    }

    return 1e6f;
}

bool GeometricPlanner::planDubinsPathIteratively(
        const Pose2D& start, const Pose2D& goal,
        const OccupancyGrid::Ptr occ_grid,
        Path& geometric_path,
        float min_turning_radius, float max_turning_radius,
        float turning_radius_step_size, float step_size)
{
    geometric_path.clear();
    float last_path_length = start.distTo(goal);
    if ( last_path_length < 1e-3f )
    {
        return true;
    }

    Path pose_path;
    std::vector<std::pair<float, float> > radius_length_pairs;

    for ( float turning_radius = min_turning_radius;
          GCUtils::roundFloat(turning_radius, 2) <= max_turning_radius;
          turning_radius += turning_radius_step_size )
    {
        float path_length = GeometricPlanner::calcDubinsPath(start, goal,
                pose_path, turning_radius, step_size);

        int pose_path_collision_index = GeometricPlanner::calcCollisionIndex(
                occ_grid, pose_path);

        if ( pose_path_collision_index < 0 )
        {
            radius_length_pairs.push_back(std::make_pair(turning_radius, path_length));
        }
    }

    if ( radius_length_pairs.empty() )
    {
        return false;
    }

    float best_turning_radius = radius_length_pairs[0].first;
    for ( size_t i = 0; i < radius_length_pairs.size(); i++ )
    {
        best_turning_radius = radius_length_pairs[i].first;
        if ( i+1 < radius_length_pairs.size() &&
             radius_length_pairs[i+1].second/radius_length_pairs[i].second > 2.0f )
        {
            break;
        }
    }
    GeometricPlanner::calcDubinsPath(start, goal, geometric_path,
                                     best_turning_radius, step_size);

    return true;
}

bool GeometricPlanner::planSimplePath(
        const Pose2D& start, const Pose2D& goal,
        const PointCloud2D& laser_pts,
        const OccupancyGrid::Ptr occ_grid,
        Path& geometric_path,
        float step_size, float desired_obs_dist)
{
    geometric_path.clear();

    if ( occ_grid->isColliding(goal) )
    {
        return false;
    }

    Path pose_path = GeometricPlanner::calcStraightLinePath(
            start, goal, step_size);
    int pose_path_collision_index = GeometricPlanner::calcCollisionIndex(
            occ_grid, pose_path);

    if ( pose_path_collision_index < 0 )
    {
        geometric_path = pose_path;
        return true;
    }

    Point2D closest_laser_pt = GCUtils::calcClosestPoint(
            laser_pts, pose_path[pose_path_collision_index].position());
    LineSegment2D straight_line(start.position(), goal.position());
    Pose2D collision_pose(straight_line.closestPointTo(closest_laser_pt),
                          straight_line.angle());
    PointVec2D sample_pts = GCUtils::generatePerpendicularPointsAt(collision_pose);

    float best_cost = 1e9f;
    for ( const Point2D& pt : sample_pts )
    {
        Pose2D wp(pt, collision_pose.theta);
        if ( occ_grid->isColliding(wp) )
        {
            continue;
        }
        float obs_dist = pt.distTo(closest_laser_pt);
        float cost = std::pow(obs_dist - desired_obs_dist, 2);
        if ( !geometric_path.empty() && cost >= best_cost )
        {
            continue;
        }
        Path first_half_pose_path = GeometricPlanner::calcStraightLinePath(
                start, wp, step_size);

        if ( GeometricPlanner::calcCollisionIndex(occ_grid, first_half_pose_path) > 0 )
        {
            continue;
        }
        Path second_half_pose_path = GeometricPlanner::calcStraightLinePath(
                wp, goal, step_size);
        if ( GeometricPlanner::calcCollisionIndex(occ_grid, second_half_pose_path) > 0 )
        {
            continue;
        }

        geometric_path.clear();
        geometric_path.reserve(first_half_pose_path.size() + second_half_pose_path.size());
        geometric_path.insert(geometric_path.end(), first_half_pose_path.begin(),
                              first_half_pose_path.end());
        geometric_path.insert(geometric_path.end(), second_half_pose_path.begin(),
                              second_half_pose_path.end());
        best_cost = cost;
    }
    return !geometric_path.empty();
}

bool GeometricPlanner::planSimpleDubinsPath(
        const Pose2D& start, const Pose2D& goal,
        const PointCloud2D& laser_pts,
        const OccupancyGrid::Ptr occ_grid,
        Path& geometric_path,
        float min_turning_radius,
        float max_turning_radius,
        float turning_radius_step_size,
        float step_size,
        float desired_obs_dist)
{
    if ( occ_grid->isColliding(goal) )
    {
        return false;
    }

    Path pose_path;
    GeometricPlanner::calcDubinsPathIteratively(start, goal, pose_path,
            min_turning_radius, max_turning_radius, turning_radius_step_size,
            step_size);
    int pose_path_collision_index = GeometricPlanner::calcCollisionIndex(
            occ_grid, pose_path);

    if ( pose_path_collision_index < 0 )
    {
        geometric_path = pose_path;
        return true;
    }


    Point2D closest_laser_pt = GCUtils::calcClosestPoint(
            laser_pts, pose_path[pose_path_collision_index].position());
    LineSegment2D straight_line(start.position(), goal.position());
    Pose2D collision_pose(straight_line.closestPointTo(closest_laser_pt),
                          straight_line.angle());
    PointVec2D sample_pts = GCUtils::generatePerpendicularPointsAt(collision_pose);

    // Circle collision_pt = pose_path[pose_path_collision_index].position;
    // Circle closest_laser_pt = Utils::getClosestPoint(laser_pts, collision_pt);
    // Circle start_pt(start.x, start.y);
    // Circle goal_pt(goal.x, goal.y);
    // Circle proj_closest_laser_pt = Utils::getProjectedPointOnSegment(
    //         start_pt, goal_pt, closest_laser_pt, true);
    // std::vector<Circle> sample_pts = Utils::generatePerpendicularPoints(
    //         start, goal, proj_closest_laser_pt);

    float best_cost = std::numeric_limits<float>::max();
    for ( const Point2D& pt : sample_pts )
    {
        Pose2D wp(pt, collision_pose.theta);
        if ( occ_grid->isColliding(wp) )
        {
            continue;
        }
        float obs_dist = pt.distTo(closest_laser_pt);
        float cost = std::pow(obs_dist - desired_obs_dist, 2);
        if ( !geometric_path.empty() && cost >= best_cost )
        {
            continue;
        }
        Path first_half_pose_path;
        GeometricPlanner::calcDubinsPathIteratively(start, wp,
                first_half_pose_path, min_turning_radius,
                max_turning_radius, turning_radius_step_size, step_size);

        if ( first_half_pose_path.size() == 0 || 
             GeometricPlanner::calcCollisionIndex(occ_grid, first_half_pose_path) > 0 )
        {
            continue;
        }

        Path second_half_pose_path;
        GeometricPlanner::calcDubinsPathIteratively(wp, goal,
                second_half_pose_path, min_turning_radius,
                max_turning_radius, turning_radius_step_size, step_size);
        if ( second_half_pose_path.size() == 0 ||
             GeometricPlanner::calcCollisionIndex(occ_grid, second_half_pose_path) > 0 )
        {
            continue;
        }
        geometric_path.clear();
        geometric_path.reserve(first_half_pose_path.size() + second_half_pose_path.size());
        geometric_path.insert(geometric_path.end(), first_half_pose_path.begin(),
                              first_half_pose_path.end());
        geometric_path.insert(geometric_path.end(), second_half_pose_path.begin(),
                              second_half_pose_path.end());
        best_cost = cost;
    }
    return !geometric_path.empty();
}

int GeometricPlanner::calcCollisionIndex(const OccupancyGrid::Ptr occ_grid,
                                         const Path& geometric_path)
{
    for ( size_t i = 0; i < geometric_path.size(); i++ )
    {
        if ( occ_grid->isColliding(geometric_path[i]) )
        {
            return i;
        }
    }
    return -1;
}

int GeometricPlanner::calcIndexAtDist(const Path& geometric_path,
                                      float linear_dist, float angular_dist)
{
    if ( geometric_path.empty() )
    {
        return -1;
    }

    float accumulated_dist = 0.0f;
    float accumulated_ang_dist = 0.0f;
    for ( size_t i = 0; i+1 < geometric_path.size(); i++ )
    {
        accumulated_dist += geometric_path[i].distTo(geometric_path[i+1]);
        accumulated_ang_dist += std::fabs(GCUtils::calcShortestAngle(
                    geometric_path[i].theta, geometric_path[i+1].theta));
        if ( accumulated_dist >= linear_dist ||
             accumulated_ang_dist >= angular_dist )
        {
            return i+1;
        }
    }
    return geometric_path.size()-1;
}

int GeometricPlanner::calcIndexAtDist(const PointVec2D& geometric_path,
                                     float linear_dist)
{
    if ( geometric_path.empty() )
    {
        return -1;
    }

    float accumulated_dist = 0.0f;
    for ( size_t i = 0; i+1 < geometric_path.size(); i++ )
    {
        accumulated_dist += geometric_path[i].distTo(geometric_path[i+1]);
        if ( accumulated_dist >= linear_dist )
        {
            return i;
        }
    }
    return geometric_path.size()-1;
}

} // namespace cabin
