#include <chrono>
#include <functional>
#include <queue>

#include <geometry_common/Utils.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/utils/utils.h>
#include <cabin_nav/utils/lattice_search_utils.h>

using kelo::geometry_common::Point2D;
using kelo::geometry_common::LineSegment2D;
using kelo::geometry_common::Pose2D;
using kelo::geometry_common::Path;
using kelo::geometry_common::TransformMatrix2D;
using GCUtils = kelo::geometry_common::Utils;

namespace cabin {

bool LatticeSearchUtils::searchGeneral(
        const Pose2D& start_pose,
        const Pose2D& goal_pose,
        const OccupancyGrid::Ptr occ_grid,
        const AllNeighbourOffsets& neighbour_offsets,
        float grid_cell_size,
        size_t grid_size_x,
        size_t grid_size_y,
        const TransformMatrix2D& tf, 
        Path& pose_path,
        SearchType search_type,
        float time_threshold)
{
    LatticeConfig lattice_config;
    lattice_config.grid_size_x = grid_size_x;
    lattice_config.grid_size_y = grid_size_y;
    lattice_config.grid_cell_size = grid_cell_size;
    lattice_config.num_of_angles = neighbour_offsets.size();
    lattice_config.tf = tf;
    float angle_offset = (2*M_PI)/lattice_config.num_of_angles;

    Lattice<bool> dummy_lattice(lattice_config, false);

    LatticeIndex goal_lat_ind;
    if ( !dummy_lattice.calcGridIndex(goal_pose, goal_lat_ind) )
    {
        std::cerr << Print::Warn << Print::Time() << "[LatticeSearchUtils] "
                  << "Could not initialise goal node."
                  << Print::End << std::endl;
        return false;
    }
    // std::cout << "goal: " << goal_lat_ind << std::endl;

    GoalTestFunction isGoal = std::bind(
            &LatticeSearchUtils::isLatticeIndexSame,
            std::placeholders::_1, goal_lat_ind);

    CollisionTestFunction isColliding = std::bind(
            &LatticeSearchUtils::isLatticePoseColliding,
            std::placeholders::_1, occ_grid, dummy_lattice);

    if ( isColliding(goal_lat_ind) )
    {
        std::cerr << Print::Warn << Print::Time() << "[LatticeSearchUtils] "
                  << "Goal is in collision"
                  << Print::End << std::endl;
        return false;
    }

    HeuristicFunction calcHeuristic = std::bind(
            &LatticeSearchUtils::heuristic3d,
            std::placeholders::_1, goal_lat_ind,
            lattice_config.grid_cell_size,
            lattice_config.num_of_angles,
            angle_offset);

    return LatticeSearchUtils::search(start_pose, isGoal, isColliding,
            calcHeuristic, neighbour_offsets, lattice_config, pose_path,
            search_type, time_threshold);
}

bool LatticeSearchUtils::searchLocal(
        const Pose2D& start_pose, const Pose2D& goal_pose,
        const OccupancyGrid::Ptr occ_grid, const AllNeighbourOffsets& neighbour_offsets,
        float grid_cell_size, float grid_radius, std::vector<Pose2D>& pose_path,
        SearchType search_type, float time_threshold)
{
    LatticeConfig lattice_config(grid_radius, grid_cell_size, neighbour_offsets.size());
    Lattice<bool> dummy_lattice(lattice_config, false);
    float angle_offset = (2*M_PI)/lattice_config.num_of_angles;

    LatticeIndex goal_lat_ind;
    if ( !dummy_lattice.calcGridIndex(goal_pose, goal_lat_ind) )
    {
        std::cerr << Print::Warn << Print::Time() << "[LatticeSearchUtils] "
                  << "Could not initialise goal node."
                  << Print::End << std::endl;
        return false;
    }
    // std::cout << "goal: " << goal_lat_ind << std::endl;

    GoalTestFunction isGoal = std::bind(
            &LatticeSearchUtils::isLatticeIndexSame,
            std::placeholders::_1, goal_lat_ind);

    CollisionTestFunction isColliding = std::bind(
            &LatticeSearchUtils::isLatticePoseColliding,
            std::placeholders::_1, occ_grid, dummy_lattice);

    if ( isColliding(goal_lat_ind) )
    {
        std::cerr << Print::Warn << Print::Time() << "[LatticeSearchUtils] "
                  << "Goal is in collision"
                  << Print::End << std::endl;
        return false;
    }

    HeuristicFunction calcHeuristic = std::bind(
            &LatticeSearchUtils::heuristic3d,
            std::placeholders::_1, goal_lat_ind,
            lattice_config.grid_cell_size,
            lattice_config.num_of_angles,
            angle_offset);

    return LatticeSearchUtils::search(start_pose, isGoal, isColliding,
            calcHeuristic, neighbour_offsets, lattice_config, pose_path,
            search_type, time_threshold);
}

bool LatticeSearchUtils::searchOccGrid(
        const Pose2D& start_pose, const Pose2D& goal_pose,
        const OccupancyGrid::Ptr occ_grid,
        const AllNeighbourOffsets& neighbour_offsets,
        Path& pose_path,
        SearchType search_type,
        float time_threshold)
{
    LatticeConfig lattice_config;
    lattice_config.grid_size_x = occ_grid->getGridSizeX();
    lattice_config.grid_size_y = occ_grid->getGridSizeY();
    lattice_config.grid_cell_size = occ_grid->getGridCellSize();
    lattice_config.num_of_angles = 1;
    lattice_config.tf = occ_grid->getTransform();
    float angle_offset = (2*M_PI)/lattice_config.num_of_angles;

    Lattice<bool> dummy_lattice(lattice_config, false);

    LatticeIndex goal_lat_ind;
    if ( !dummy_lattice.calcGridIndex(goal_pose, goal_lat_ind) )
    {
        std::cerr << Print::Warn << Print::Time() << "[LatticeSearchUtils] "
                  << "Could not initialise goal node."
                  << Print::End << std::endl;
        return false;
    }
    // std::cout << "goal: " << goal_lat_ind << std::endl;

    GoalTestFunction isGoal = std::bind(
            &LatticeSearchUtils::isLatticeIndexSame,
            std::placeholders::_1, goal_lat_ind);

    CollisionTestFunction isColliding = std::bind(
            &LatticeSearchUtils::isLatticeIndexColliding,
            std::placeholders::_1, occ_grid);

    if ( isColliding(goal_lat_ind) )
    {
        std::cerr << Print::Warn << Print::Time() << "[LatticeSearchUtils] "
                  << "Goal is in collision"
                  << Print::End << std::endl;
        return false;
    }

    HeuristicFunction calcHeuristic = std::bind(
            &LatticeSearchUtils::heuristic3d,
            std::placeholders::_1, goal_lat_ind,
            lattice_config.grid_cell_size,
            lattice_config.num_of_angles,
            angle_offset);

    return LatticeSearchUtils::search(start_pose, isGoal, isColliding,
            calcHeuristic, neighbour_offsets, lattice_config, pose_path,
            search_type, time_threshold);
}

bool LatticeSearchUtils::search(
                const Pose2D& start_pose,
                GoalTestFunction isGoal,
                CollisionTestFunction isColliding,
                HeuristicFunction calcHeuristic,
                const AllNeighbourOffsets& neighbour_offsets,
                const LatticeConfig& lattice_config,
                Path& pose_path,
                SearchType search_type,
                float time_threshold)

{
    pose_path.clear();

    size_t num_of_angles = lattice_config.num_of_angles;
    size_t max_val = std::numeric_limits<size_t>::max();

    std::priority_queue<Node, std::vector<Node>,
        std::function<bool(const Node&, const Node&)> > fringe(greaterNode);

    Lattice<size_t> p_lattice(lattice_config, max_val); // parent
    Lattice<bool> c_lattice(lattice_config, false); // closed
    Lattice<float> f_lattice(lattice_config, 0.0f); // f value

    Node start;
    if ( !p_lattice.calcGridIndex(start_pose, start.lat_ind) )
    {
        std::cerr << Print::Warn << Print::Time() << "[LatticeSearchUtils] "
                  << "Could not initialise start node."
                  << Print::End << std::endl;
        return false;
    }

    start.h = start.g = start.f = 0.0f;
    // std::cout << "start: " << start << std::endl;
    fringe.push(start);
    p_lattice.setValueAt(start.lat_ind, p_lattice.calcIndex(start.lat_ind));
    // std::cout << lattice << std::endl;

    LatticeIndex goal_lat_ind;
    bool reached_goal = false;

    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    std::chrono::duration<float> timeout_duration(time_threshold);

    size_t itr = 0;
    size_t itr_limit = p_lattice.getGridSizeX() * p_lattice.getGridSizeY() * num_of_angles;
    // size_t itr_limit = 5e2;
    Node current;

    while ( !fringe.empty() && itr < itr_limit )
    {
        itr++;
        // std::cout << "itr: " << itr << std::endl;

        if ( std::chrono::steady_clock::now() - start_time > timeout_duration )
        {
            std::cerr << Print::Warn << Print::Time() << "[LatticeSearchUtils] "
                      << "Time limit exceeded."
                      << Print::End << std::endl;
            return false;
        }

        current = fringe.top();
        fringe.pop();
        // std::cout << "curr: " << current << std::endl;
        size_t current_index = p_lattice.calcIndex(current.lat_ind);

        /* ignore already closed node */
        if ( c_lattice.getValueAt(current_index) )
        {
            continue;
        }

        c_lattice.setValueAt(current_index, true); // close current node

        if ( isGoal(current.lat_ind) ) /* goal test */
        {
            goal_lat_ind = current.lat_ind;
            reached_goal = true;
            break;
        }

        /* add neighbours of current */
        for ( const NeighbourOffset& n_offset : neighbour_offsets[current.lat_ind.k] )
        {
            // std::cout << "n_offset: " << n_offset << std::endl;
            Node n(current, n_offset.i, n_offset.j, n_offset.k, num_of_angles);
            size_t n_index = p_lattice.calcIndex(n.lat_ind);

            /* ignore if outside lattice */
            if ( n_index == max_val )
            {
                continue;
            }

            /* ignore if it is already closed */
            if ( c_lattice.getValueAt(n_index) )
            {
                // std::cout << "Ignoring " << n << std::endl;
                continue;
            }

            /* update g, h and f values */
            switch ( search_type )
            {
                case SearchType::BFS :
                    n.g += 1.0f;
                    break;
                case SearchType::DIJKSTRA :
                    n.g += n_offset.g;
                    break;
                case SearchType::ASTAR :
                    n.h = calcHeuristic(n.lat_ind);
                    n.g += n_offset.g;
                    break;
            }
            n.f = n.g + n.h;

            /* ignore if n is already in fringe with lower cost */
            if ( p_lattice.getValueAt(n_index) < max_val &&
                 f_lattice.getValueAt(n_index) < n.f )
            {
                // std::cout << "Ignoring f " << n << std::endl;
                continue;
            }
            /* ignore if colliding */
            if ( isColliding(n.lat_ind) )
            {
                c_lattice.setValueAt(n_index, true); // close colliding state
                continue;
            }

            // std::cout << "Upserting " << n << std::endl;
            fringe.push(n);
            p_lattice.setValueAt(n_index, current_index); // update parent
            f_lattice.setValueAt(n_index, n.f); // update f value

            if ( search_type == SearchType::BFS && n.lat_ind == goal_lat_ind )
            {
                c_lattice.setValueAt(n_index, true); // close current node
                itr = itr_limit;
                break;
            }
        }
        // std::cout << std::endl;
    }

    // std::cout << "itr: " << itr << std::endl;
    // std::chrono::duration<float> prop_time_taken = std::chrono::steady_clock::now() - start_time;
    // std::cout << "Propogation time: " << prop_time_taken.count()*1000.0f << " ms" << std::endl;
    if ( !reached_goal || !c_lattice.getValueAt(goal_lat_ind) ) // goal is not closed
    {
        std::cerr << Print::Warn << Print::Time() << "[LatticeSearchUtils] "
                  << "Could not find path."
                  << Print::End << std::endl;
        return false;
    }

    LatticeSearchUtils::backtrace(goal_lat_ind, p_lattice, pose_path);
    std::reverse(pose_path.begin(), pose_path.end());

    // std::chrono::duration<float> search_time_taken = std::chrono::steady_clock::now() - start_time;
    // std::cout << "Search time: " << search_time_taken.count()*1000.0f << " ms" << std::endl;
    return true;
}

bool LatticeSearchUtils::backtrace(
        const LatticeIndex& goal_lat_ind,
        const Lattice<size_t>& lattice,
        Path& pose_path)
{
    pose_path.clear();
    LatticeIndex lat_ind = goal_lat_ind; // back tracing LatticeIndex
    size_t itr = 0;
    size_t itr_limit = lattice.getGridSizeX() * lattice.getGridSizeY(); // ignoring num_of_angles
    size_t parent_index;
    size_t current_index = lattice.calcIndex(lat_ind);
    size_t default_value = lattice.getDefaultValue();
    while ( itr < itr_limit )
    {
        if ( current_index == default_value )
        {
            pose_path.clear();
            return false;
        }
        itr++;
        Pose2D bt_pose = lattice.calcPoseAt(lat_ind);
        pose_path.push_back(bt_pose);
        parent_index = lattice.getValueAt(lat_ind);
        if ( parent_index == current_index )
        {
            break;
        }
        lat_ind = lattice.calcLatticeIndex(parent_index);
        current_index = parent_index;
    }
    return true;
}

std::vector<NeighbourOffset> LatticeSearchUtils::calcBestArcNeighbours(
        float grid_cell_size, size_t num_of_angles, float min_turning_radius,
        float start_angle, int max_angle_offset, const Lattice<bool>& lattice)
{
    Point2D start_point;
    Pose2D start_pose(start_point, start_angle);

    /* find eq. of line that is perp. to start_pose */
    Point2D left_point = Point2D::initFromRadialCoord(min_turning_radius, start_angle + M_PI/2);
    LineSegment2D l1(start_point, left_point);
    float m1 = l1.slope();
    float c1 = l1.constant();

    LatticeIndex start_lat_ind;
    lattice.calcGridIndex(start_pose, start_lat_ind);

    LatticeIndex lat_ind;
    float radius_diff_thr = grid_cell_size/2; // is there a better value?
    std::vector<Pose2D> neighbours(num_of_angles, Pose2D());
    std::vector<float> best_score(num_of_angles, std::numeric_limits<float>::max());
    for ( size_t i = start_lat_ind.i; i < lattice.getGridSizeX(); i++ )
    {
        for ( size_t j = 0; j < lattice.getGridSizeY(); j++ )
        {
            for ( size_t k = 0; k < num_of_angles; k++ )
            {
                lat_ind.i = i;
                lat_ind.j = j;
                lat_ind.k = k;

                // do not allow angle diff to be more than 90 degrees
                if ( std::abs(static_cast<int>(lat_ind.k) -
                              static_cast<int>(start_lat_ind.k)) > max_angle_offset )
                {
                    continue;
                }
                if ( lat_ind.k == start_lat_ind.k ) // same angle so no arc formed
                {
                    continue;
                }
                Pose2D p = lattice.calcPoseAt(lat_ind);
                Point2D pt = p.position();
                left_point = pt + Point2D::initFromRadialCoord(
                        min_turning_radius, p.theta + M_PI/2);

                LineSegment2D l2(pt, left_point);
                Point2D int_pt = Utils::calcIntersectionPt(
                        m1, c1, l2.slope(), l2.constant());
                float d1 = GCUtils::roundFloat(int_pt.distTo(start_point), 3);
                float d2 = GCUtils::roundFloat(int_pt.distTo(pt), 3);
                /* discard if radii is smaller than min_turning_radius */
                if ( d1 < min_turning_radius || d2 < min_turning_radius )
                {
                    continue;
                }
                /* the radii should more or less match to form an arc */
                float radius_error = std::fabs(d1 - d2);
                if ( radius_error > radius_diff_thr )
                {
                    continue;
                }
                float p_angle_with_int_pt = GCUtils::calcShortestAngle(
                        p.theta, (int_pt - pt).angle());
                float start_angle_with_int_pt = GCUtils::calcShortestAngle(
                        start_pose.theta, int_pt.angle());
                /* int_pt should form more or less same angle with start_pose
                 * and p to form an arc */
                if ( std::fabs(p_angle_with_int_pt - start_angle_with_int_pt) > 0.1f )
                {
                    continue;
                }
                float dist = start_pose.distTo(p);
                float score = (dist * 1e1f) + (radius_error * 5e1f); // TODO: how to tune?
                if ( score < best_score[lat_ind.k] )
                {
                    best_score[lat_ind.k] = score;
                    neighbours[lat_ind.k] = p;
                }
            }
        }
    }

    /* fill neighbour_offsets from neighbours and best_score */
    std::vector<NeighbourOffset> neighbour_offsets;
    for ( size_t i = 0; i < num_of_angles; i++ )
    {
        if ( best_score[i] > 1e2f )
        {
            continue;
        }
        lattice.calcGridIndex(neighbours[i], lat_ind);
        NeighbourOffset neighbour_offset;
        neighbour_offset.i = static_cast<int>(lat_ind.i) - static_cast<int>(start_lat_ind.i);
        neighbour_offset.j = static_cast<int>(lat_ind.j) - static_cast<int>(start_lat_ind.j);
        neighbour_offset.k = static_cast<int>(lat_ind.k) - static_cast<int>(start_lat_ind.k);
        neighbour_offsets.push_back(neighbour_offset);
    }
    return neighbour_offsets;
}

NeighbourOffset LatticeSearchUtils::calcBestStraightNeighbour(
        float grid_cell_size, size_t num_of_angles,
        float start_angle, const Lattice<bool>& lattice)
{
    Point2D start_point;
    Pose2D start_pose(start_point, start_angle);

    float straight_line_dist = lattice.getGridLength() * 2;
    LineSegment2D straight_line(
            start_point,
            start_point + Point2D::initFromRadialCoord(straight_line_dist, start_angle));

    LatticeIndex start_lat_ind;
    lattice.calcGridIndex(start_pose, start_lat_ind);

    LatticeIndex lat_ind;
    lat_ind.k = start_lat_ind.k;
    NeighbourOffset neighbour_offset;
    float perp_dist_thr = grid_cell_size/2; // is there a better value?
    float best_score = std::numeric_limits<float>::max();
    for ( size_t i = start_lat_ind.i; i < lattice.getGridSizeX(); i++ )
    {
        for ( size_t j = 0; j < lattice.getGridSizeY(); j++ )
        {
            lat_ind.i = i;
            lat_ind.j = j;

            if ( start_lat_ind == lat_ind )
            {
                continue;
            }

            Point2D pt = lattice.calcPoseAt(lat_ind).position();
            float dist = GCUtils::roundFloat(pt.distTo(start_point), 3);
            float perp_dist = GCUtils::roundFloat(straight_line.minDistTo(pt), 3);

            if ( perp_dist > perp_dist_thr )
            {
                continue;
            }

            float score = (dist * 1e1f) + (perp_dist * 1e2f); // TODO: how to tune?
            if ( score < best_score )
            {
                best_score = score;
                neighbour_offset.i = (int)lat_ind.i - (int)start_lat_ind.i;
                neighbour_offset.j = (int)lat_ind.j - (int)start_lat_ind.j;
            }
        }
    }
    return neighbour_offset;
}

std::vector<NeighbourOffset> LatticeSearchUtils::calcOmniDirectionalNeighbours()
{
    std::vector<NeighbourOffset> neighbour_offsets;
    for ( int i = -1; i < 2; ++i )
    {
        for ( int j = -1; j < 2; ++j )
        {
            if ( i == 0 && j == 0 )
            {
                continue;
            }
            neighbour_offsets.push_back(NeighbourOffset(i, j, 0));
        }
    }
    return neighbour_offsets;
}

std::vector<std::vector<NeighbourOffset> > LatticeSearchUtils::calcNeighbourOffset(
        float grid_cell_size, size_t num_of_angles, float turning_radius,
        int max_angle_offset, const std::string& kinematic_model)
{
    std::vector<std::vector<NeighbourOffset> > neighbour_offsets(num_of_angles);
    if ( num_of_angles == 1 )
    {
        /* Only use when robot is more or less circular */
        std::vector<NeighbourOffset> neighbour_offsets_0 = calcOmniDirectionalNeighbours();
        for ( NeighbourOffset& n_offset : neighbour_offsets_0 )
        {
            n_offset.g = std::sqrt(static_cast<float>(std::pow(n_offset.i, 2) +
                                                      std::pow(n_offset.j, 2))) *
                         grid_cell_size;
        }
        neighbour_offsets[0] = neighbour_offsets_0;
        return neighbour_offsets;
    }

    /* make sure num_of_angles is divisible by 8. This is neccessary because the
     * mirrorring done later works based on this condition. */
    if ( num_of_angles % 8 != 0 )
    {
        num_of_angles = ((num_of_angles/8)+1) * 8; // round up to next multiple of 8
    }

    /* treat turning radius as min_turning_radius to generate arc paths */
    /* technically, for unicycle, min_turning_radius would be zero */
    float min_turning_radius = turning_radius;
    LatticeConfig lattice_config(min_turning_radius*1.5f, grid_cell_size, num_of_angles);
    Lattice<bool> lattice(lattice_config, false);
    float angle_offset = (2*M_PI)/num_of_angles;
    /* iterate from 0 to 45 degrees */
    for ( size_t i = 0; i <= num_of_angles/8; i++ )
    {
        size_t k = i + num_of_angles/2;
        std::vector<NeighbourOffset> neighbour_offset;

        /* add forward straight motion neighbour */
        NeighbourOffset frwd_offset = LatticeSearchUtils::calcBestStraightNeighbour(
                grid_cell_size, num_of_angles, i*angle_offset, lattice);
        neighbour_offset.push_back(frwd_offset);
        // size_t multiple = std::ceil(turning_radius /
        //         (grid_cell_size * std::max(frwd_offset.i, frwd_offset.j)));
        // neighbour_offset.push_back(NeighbourOffset(
        //             frwd_offset.i*multiple, frwd_offset.j*multiple,
        //             frwd_offset.k, frwd_offset.g*multiple));

        /* add forward arc motion neighbours */
        std::vector<NeighbourOffset> arc_neighbour_offsets = LatticeSearchUtils::calcBestArcNeighbours(
                grid_cell_size, num_of_angles, min_turning_radius, i*angle_offset,
                max_angle_offset, lattice);
        neighbour_offset.reserve(neighbour_offset.size() + arc_neighbour_offsets.size());
        neighbour_offset.insert(neighbour_offset.end(), arc_neighbour_offsets.begin(),
                                arc_neighbour_offsets.end());

        /* add omni directional neighbours */
        if ( kinematic_model == "omni-directional" )
        {
            std::vector<NeighbourOffset> neighbour_offset_omni = LatticeSearchUtils::calcOmniDirectionalNeighbours();
            neighbour_offset.reserve(neighbour_offset.size() + neighbour_offset_omni.size());
            neighbour_offset.insert(neighbour_offset.end(), neighbour_offset_omni.begin(),
                                    neighbour_offset_omni.end());
        }
        /* add rotate in place neighbour */
        if ( kinematic_model == "unicycle" || kinematic_model == "omni-directional" )
        {
            neighbour_offset.push_back(NeighbourOffset(0,0,1));
            neighbour_offset.push_back(NeighbourOffset(0,0,-1));
        }

        /* assign cost for each neighbour */
        for ( NeighbourOffset& n_offset : neighbour_offset )
        {
            float dist = std::sqrt(static_cast<float>(std::pow(n_offset.i, 2) +
                                                      std::pow(n_offset.j, 2)));
            if ( dist < 1e-3f ) // rotating in place
            {
                n_offset.g = std::abs(n_offset.k) * angle_offset * 1.5f;
            }
            else if ( n_offset.k == 0 )
            {
                n_offset.g = dist * grid_cell_size;
            }
            else
            {
                n_offset.g = (dist * grid_cell_size) + (std::abs(n_offset.k) * angle_offset);
            }
        }
        neighbour_offsets[k] = neighbour_offset;
    }

    /* mirror neighbours between 0 to 45 degrees along 45 degree axis to get
     * neighbours between 45 to 90 degrees */
    for ( size_t i = 1; i <= num_of_angles/8; i++ )
    {
        int k = (5*(num_of_angles/8)) + i;
        int mirrorred_k = (5*(num_of_angles/8)) - i;
        neighbour_offsets[k] = std::vector<NeighbourOffset>(neighbour_offsets[mirrorred_k].size());
        for ( size_t j = 0; j < neighbour_offsets[k].size(); j++ )
        {
            neighbour_offsets[k][j].i = neighbour_offsets[mirrorred_k][j].j;
            neighbour_offsets[k][j].j = neighbour_offsets[mirrorred_k][j].i;
            neighbour_offsets[k][j].k = -neighbour_offsets[mirrorred_k][j].k;
            neighbour_offsets[k][j].g = neighbour_offsets[mirrorred_k][j].g;
        }
    }

    /* mirror neighbours between 0 to 90 degrees along Y axis to get
     * neighbours between 90 to 180 degrees */
    for ( size_t i = 1; i <= num_of_angles/4; i++ )
    {
        int k = (3*(num_of_angles/4)) + i;
        k = k % num_of_angles;
        int mirrorred_k = (3*(num_of_angles/4)) - i;
        neighbour_offsets[k] = std::vector<NeighbourOffset>(neighbour_offsets[mirrorred_k].size());
        for ( size_t j = 0; j < neighbour_offsets[k].size(); j++ )
        {
            neighbour_offsets[k][j].i = -neighbour_offsets[mirrorred_k][j].i;
            neighbour_offsets[k][j].j = neighbour_offsets[mirrorred_k][j].j;
            neighbour_offsets[k][j].k = -neighbour_offsets[mirrorred_k][j].k;
            neighbour_offsets[k][j].g = neighbour_offsets[mirrorred_k][j].g;
        }
    }

    /* mirror neighbours between 0 to 180 degrees along X axis to get
     * neighbours between -180 to 0 degrees */
    for ( size_t i = 1; i < num_of_angles/2; i++ )
    {
        int k = (num_of_angles/2) - i;
        int mirrorred_k = (num_of_angles/2) + i;
        neighbour_offsets[k] = std::vector<NeighbourOffset>(neighbour_offsets[mirrorred_k].size());
        for ( size_t j = 0; j < neighbour_offsets[k].size(); j++ )
        {
            neighbour_offsets[k][j].i = neighbour_offsets[mirrorred_k][j].i;
            neighbour_offsets[k][j].j = -neighbour_offsets[mirrorred_k][j].j;
            neighbour_offsets[k][j].k = -neighbour_offsets[mirrorred_k][j].k;
            neighbour_offsets[k][j].g = neighbour_offsets[mirrorred_k][j].g;
        }
    }


    return neighbour_offsets;
}

float LatticeSearchUtils::heuristic3d(
        const LatticeIndex& n, const LatticeIndex& goal,
        float grid_cell_size, size_t num_of_angles, float angle_offset)
{
    float h = 0.0f;
    int i_diff = std::abs(static_cast<int>(n.i) - static_cast<int>(goal.i));
    int j_diff = std::abs(static_cast<int>(n.j) - static_cast<int>(goal.j));
    if ( num_of_angles == 1 )
    {
        float grid_dist = ( i_diff > j_diff )
                          ? (i_diff - j_diff) + (1.41f * j_diff) // sqrt(2) ~= 1.41
                          : (j_diff - i_diff) + (1.41f * i_diff);
        return grid_dist * grid_cell_size;
    }

    // float grid_dist = sqrt((i_diff * i_diff) + (j_diff * j_diff));
    float grid_dist = ( i_diff > j_diff )
                      ? (i_diff - j_diff) + (1.41f * j_diff) // sqrt(2) ~= 1.41
                      : (j_diff - i_diff) + (1.41f * i_diff);
    h += grid_dist * grid_cell_size;

    int k_diff_raw = abs((int)n.k - (int)goal.k);
    int k_diff_inv = (int)num_of_angles - k_diff_raw;
    int k_diff = std::min(k_diff_raw, k_diff_inv);
    h += k_diff * angle_offset * 1.0f; // TODO: probably need to tune this ??

    return h;
}

bool LatticeSearchUtils::isLatticeIndexSame(const LatticeIndex& lat_ind1,
                                            const LatticeIndex& lat_ind2)
{
    return ( lat_ind1 == lat_ind2 );
}

bool LatticeSearchUtils::isLatticePoseColliding(
        const LatticeIndex& lat_ind,
        OccupancyGrid::Ptr occ_grid,
        const Lattice<bool>& lattice)
{
    return occ_grid->isColliding(lattice.calcPoseAt(lat_ind));
}

bool LatticeSearchUtils::isLatticeIndexColliding(
        const LatticeIndex& lat_ind,
        OccupancyGrid::Ptr occ_grid)
{
    return occ_grid->isOccupied(lat_ind.i, lat_ind.j);
}


} // namespace cabin
