#pragma once

#include <queue>

#include <cabin_nav/structs/occupancy_grid.h>

namespace cabin {

struct DistMapCell
{
    size_t nearest_obst_cell_index;
    size_t dist_sq;
    bool closed;

    friend std::ostream& operator << (std::ostream& out, const DistMapCell& cell)
    {
        out << "<noci: " << cell.nearest_obst_cell_index
            << ", dist_sq: " << cell.dist_sq
            << ", closed: " << cell.closed
            << ">";
        return out;
    }
};

class CellComparator
{
    public:
        CellComparator(std::vector<DistMapCell>* dist_map):
            dist_map_(dist_map) {}

        bool operator() (size_t lhs, size_t rhs) const
        {
            return ( dist_map_->at(lhs).dist_sq > dist_map_->at(rhs).dist_sq );
        }

    private:
        std::vector<DistMapCell>* dist_map_;
};

class VoronoiCalculator
{
    public:

        /**
         * @brief Calculate generalised voronoi diagram (GVD) from an occupancy
         * grid
         * Based on paper "B. Lau, C. Sprunk and W. Burgard, Improved Updating of
         * Euclidean Distance Maps and Voronoi Diagrams, IEEE Intl. Conf. on
         * Intelligent Robots and Systems (IROS), Taipei, Taiwan, 2010." and its
         * implementation https://github.com/frontw/dynamicvoronoi
         *
         * @param occ_grid Occupancy grid based on which voronoi needs to be
         * calculated
         *
         * @return voronoi diagram represented in occupancy grid where an
         * occupied cell is a voronoi cell
         */
        static OccupancyGrid::Ptr calculateVoronoi(
                const OccupancyGrid& occ_grid, size_t obst_cell_dist_threshold);

    protected:

        static void checkAndUpdateVoronoi(
                size_t size_y,
                size_t current_i, size_t current_j, const DistMapCell& curr_cell,
                size_t neighbour_i, size_t neighbour_j, const DistMapCell& neighbour_cell,
                OccupancyGrid::Ptr voronoi_map, size_t obst_cell_dist_threshold);

        static void addObstacles(
                const OccupancyGrid& occ_grid,
                std::priority_queue<size_t, std::vector<size_t>, CellComparator>& fringe,
                std::vector<DistMapCell>& dist_map);

};

} // namespace cabin
