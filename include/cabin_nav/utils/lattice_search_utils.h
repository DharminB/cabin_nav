#pragma once

#include <geometry_common/Pose2D.h>
#include <geometry_common/TransformMatrix2D.h>

#include <cabin_nav/structs/lattice.h>
#include <cabin_nav/structs/lattice_index.h>
#include <cabin_nav/structs/node.h>
#include <cabin_nav/structs/neighbour_offset.h>
#include <cabin_nav/structs/occupancy_grid.h>

namespace cabin {

enum class SearchType
{
    BFS,
    DIJKSTRA,
    ASTAR
};

const std::vector<std::string> search_type_strings = {
    "BFS",
    "DIJKSTRA",
    "ASTAR"
};

inline std::ostream& operator << (std::ostream& out, const SearchType& search_type)
{
    size_t search_type_int = static_cast<size_t>(search_type);
    std::string search_str = ( search_type_int >= search_type_strings.size() )
                             ? "UNKNOWN"
                             : search_type_strings[search_type_int];
    out << search_str;
    return out;
};

using GoalTestFunction = std::function<bool (const LatticeIndex& )>;
using CollisionTestFunction = std::function<bool (const LatticeIndex& )>;
using HeuristicFunction = std::function<float (const LatticeIndex& )>;

class LatticeSearchUtils
{
    public:

        static bool backtrace(const LatticeIndex& goal_lat_ind,
                              const Lattice<size_t>& lattice,
                              kelo::geometry_common::Path& pose_path);

        static bool searchGeneral(
                const kelo::geometry_common::Pose2D& start_pose,
                const kelo::geometry_common::Pose2D& goal_pose,
                const OccupancyGrid::Ptr occ_grid,
                const AllNeighbourOffsets& neighbour_offsets,
                float grid_cell_size,
                size_t grid_size_x,
                size_t grid_size_y,
                const kelo::geometry_common::TransformMatrix2D& tf, 
                kelo::geometry_common::Path& pose_path,
                SearchType search_type = SearchType::ASTAR,
                float time_threshold = 0.05f);

        /**
         * @brief Search a lattice represented in a local frame (robot's frame)
         * where the robot is in middle of the lattice and the lattice is
         * atleast * `grid_radius` meters in each direction.
         *
         * @return success
         */
        static bool searchLocal(
                const kelo::geometry_common::Pose2D& start_pose,
                const kelo::geometry_common::Pose2D& goal_pose,
                const OccupancyGrid::Ptr occ_grid,
                const AllNeighbourOffsets& neighbour_offsets,
                float grid_cell_size,
                float grid_radius,
                kelo::geometry_common::Path& pose_path,
                SearchType search_type = SearchType::ASTAR,
                float time_threshold = 0.05f);

        static bool searchOccGrid(
                const kelo::geometry_common::Pose2D& start_pose,
                const kelo::geometry_common::Pose2D& goal_pose,
                const OccupancyGrid::Ptr occ_grid,
                const AllNeighbourOffsets& neighbour_offsets,
                kelo::geometry_common::Path& pose_path,
                SearchType search_type = SearchType::ASTAR,
                float time_threshold = 0.05f);

        static bool search(
                const kelo::geometry_common::Pose2D& start_pose,
                GoalTestFunction isGoal,
                CollisionTestFunction isColliding,
                HeuristicFunction calcHeuristic,
                const AllNeighbourOffsets& neighbour_offsets,
                const LatticeConfig& lattice_config,
                kelo::geometry_common::Path& pose_path,
                SearchType search_type,
                float time_threshold);

        static std::vector<NeighbourOffset> calcBestArcNeighbours(
                float grid_cell_size,
                size_t num_of_angles,
                float min_turning_radius,
                float start_angle,
                int max_angle_offset,
                const Lattice<bool>& lattice);

        static NeighbourOffset calcBestStraightNeighbour(
                float grid_cell_size,
                size_t num_of_angles,
                float start_angle,
                const Lattice<bool>& lattice);

        static std::vector<NeighbourOffset> calcOmniDirectionalNeighbours();

        /*
         * kinematic_model: string (omni-directional | unicycle | bicycle )
         */
        static AllNeighbourOffsets calcNeighbourOffset(
                float grid_cell_size,
                size_t num_of_angles,
                float turning_radius,
                int max_angle_offset,
                const std::string& kinematic_model);

        static float heuristic3d(
                const LatticeIndex& n,
                const LatticeIndex& goal,
                float grid_cell_size,
                size_t num_of_angles,
                float angle_offset);

        static bool greaterNode(const Node& n1, const Node& n2)
        {
            return ( n1.f > n2.f );
        }

        static bool isLatticeIndexSame(
                const LatticeIndex& lat_ind1,
                const LatticeIndex& lat_ind2);

        static bool isLatticePoseColliding(
                const LatticeIndex& lat_ind,
                OccupancyGrid::Ptr occ_grid,
                const Lattice<bool>& lattice);

        static bool isLatticeIndexColliding(
                const LatticeIndex& lat_ind,
                OccupancyGrid::Ptr occ_grid);

};

} // namespace cabin
