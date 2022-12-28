#include <functional>
#include <queue>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/utils/voronoi_calculator.h>

namespace cabin {

OccupancyGrid::Ptr VoronoiCalculator::calculateVoronoi(const OccupancyGrid& occ_grid,
        size_t obst_cell_dist_threshold)
{
    if ( obst_cell_dist_threshold < 2 )
    {
        obst_cell_dist_threshold = 2;
    }

    size_t size_x = occ_grid.getGridSizeX();
    size_t size_y = occ_grid.getGridSizeY();
    size_t size = size_x * size_y;
    OccupancyGrid::Ptr voronoi_map = OccupancyGrid::Ptr(
                new OccupancyGrid(size_x, size_y,
                                  occ_grid.getGridCellSize(),
                                  occ_grid.getTransform()) );

    std::vector<DistMapCell> dist_map(size);

    CellComparator cell_comparator(&dist_map);
    std::priority_queue<size_t, std::vector<size_t>, CellComparator> fringe(cell_comparator);

    // add obstacles to fringe
    VoronoiCalculator::addObstacles(occ_grid, fringe, dist_map);

    size_t itr = 0;
    size_t itr_limit = size;
    size_t current_index;
    size_t current_i, current_j;
    size_t nearest_obst_cell_i, nearest_obst_cell_j;
    size_t neighbour_index;
    size_t neighbour_i, neighbour_j;

    while ( !fringe.empty() && itr < itr_limit )
    {
        itr++;
        // std::cout << "itr: " << itr << std::endl;

        current_index = fringe.top();
        fringe.pop();
        current_i = current_index/size_y;
        current_j = current_index - (current_i * size_y);
        // std::cout << "curr: " << current_i << " " << current_j << std::endl;
        DistMapCell& curr_cell = dist_map[current_index];

        if ( curr_cell.closed )
        {
            continue;
        }

        if ( curr_cell.nearest_obst_cell_index >= size ||
             !occ_grid.isOccupied(curr_cell.nearest_obst_cell_index) ) // invalid cell
        {
            continue;
        }

        voronoi_map->setUnoccupied(current_i, current_j);

        nearest_obst_cell_i = curr_cell.nearest_obst_cell_index / size_y;
        nearest_obst_cell_j = curr_cell.nearest_obst_cell_index - (nearest_obst_cell_i * size_y);
        // std::cout << "nearest obst: " << nearest_obst_cell_i
        //           << " " << nearest_obst_cell_j << std::endl;

        for ( int i = -1; i < 2; i++ )
        {
            neighbour_i = current_i + i;
            for ( int j = -1; j < 2; j++ )
            {
                if ( i == 0 && j == 0 )
                {
                    continue;
                }
                neighbour_j = current_j + j;
                neighbour_index = (neighbour_i * size_y) + neighbour_j;

                DistMapCell& neighbour_cell = dist_map[neighbour_index];
                // std::cout << neighbour_i << " " << neighbour_j
                //           << " " << neighbour_cell << std::endl;

                size_t dist_sq = pow((int)neighbour_i - (int)nearest_obst_cell_i, 2) +
                                 pow((int)neighbour_j - (int)nearest_obst_cell_j, 2);
                // std::cout << "dist_sq: " << dist_sq << std::endl;

                if ( dist_sq < neighbour_cell.dist_sq ) // found a valid nearer obst for neighbour
                {
                    neighbour_cell.dist_sq = dist_sq;
                    neighbour_cell.nearest_obst_cell_index = curr_cell.nearest_obst_cell_index;
                    fringe.push(neighbour_index);
                    continue;
                }
                else // check voronoi conditions and update voronoi map
                {
                    VoronoiCalculator::checkAndUpdateVoronoi(size_y,
                            current_i, current_j, curr_cell,
                            neighbour_i, neighbour_j, neighbour_cell,
                            voronoi_map, obst_cell_dist_threshold);
                }
            }
        }
    }
    return voronoi_map;
}

void VoronoiCalculator::checkAndUpdateVoronoi(
        size_t size_y,
        size_t current_i, size_t current_j, const DistMapCell& curr_cell,
        size_t neighbour_i, size_t neighbour_j, const DistMapCell& neighbour_cell,
        OccupancyGrid::Ptr voronoi_map, size_t obst_cell_dist_threshold)
{
    if ( curr_cell.dist_sq < 2 || // current cell is an obstacle or next to it
         neighbour_cell.dist_sq < 2 ) // neighbour is an obstacle or next to it
    {
        return;
    }

    // calc nearest obst cell indices
    size_t nearest_obst_cell_i = curr_cell.nearest_obst_cell_index / size_y;
    size_t nearest_obst_cell_j = curr_cell.nearest_obst_cell_index -
                                 (nearest_obst_cell_i * size_y);

    size_t neighbour_nearest_obst_cell_i = neighbour_cell.nearest_obst_cell_index / size_y;
    size_t neighbour_nearest_obst_cell_j = neighbour_cell.nearest_obst_cell_index -
                                           (neighbour_nearest_obst_cell_i * size_y);

    // if obst cells of curr and neighbour are either the same
    // cell or they are adjacent to each other
    if ( std::abs((int)neighbour_nearest_obst_cell_i -
                  (int)nearest_obst_cell_i) < (int)obst_cell_dist_threshold &&
         std::abs((int)neighbour_nearest_obst_cell_j -
                  (int)nearest_obst_cell_j) < (int)obst_cell_dist_threshold )
    {
        return;
    }

    // calc dist of cell from other cell's nearest obstacle
    size_t neighbour_dist_sq = pow((int)neighbour_i - (int)nearest_obst_cell_i, 2) +
                               pow((int)neighbour_j - (int)nearest_obst_cell_j, 2);
    size_t current_dist_sq = pow((int)neighbour_nearest_obst_cell_i - (int)current_i, 2) +
                             pow((int)neighbour_nearest_obst_cell_j - (int)current_j, 2);

    // calculate increase in dist if the cell were to switch nearest obstacle
    int current_dist_inc = (int)current_dist_sq - (int)curr_cell.dist_sq;
    int neighbour_dist_inc = (int)neighbour_dist_sq - (int)neighbour_cell.dist_sq;

    if ( current_dist_inc <= neighbour_dist_inc && curr_cell.dist_sq > 2 )
    {
        voronoi_map->setOccupied(current_i, current_j);
        // std::cout << "changing curr" << std::endl;
    }
    if ( neighbour_dist_inc <= current_dist_inc && neighbour_cell.dist_sq > 2 )
    {
        voronoi_map->setOccupied(neighbour_i, neighbour_j);
        // std::cout << "changing neighbour" << std::endl;
    }
}

void VoronoiCalculator::addObstacles(
        const OccupancyGrid& occ_grid,
        std::priority_queue<size_t, std::vector<size_t>, CellComparator>& fringe,
        std::vector<DistMapCell>& dist_map)
{
    size_t size_x = occ_grid.getGridSizeX();
    size_t size_y = occ_grid.getGridSizeY();
    size_t size = size_x * size_y;

    for ( size_t i = 0; i < size_x; i++ )
    {
        for ( size_t j = 0; j < size_y; j++ )
        {
            size_t index = (i*size_y)+j;
            // free cell
            if ( !occ_grid.isOccupied(i, j) )
            {
                dist_map[index].nearest_obst_cell_index = size+1; // considered invalid
                dist_map[index].dist_sq = size+1;
                dist_map[index].closed = false;
                continue;
            }

            dist_map[index].nearest_obst_cell_index = index; // nearest obst is itself
            dist_map[index].dist_sq = 0;
            dist_map[index].closed = false;

            // at borders (don't add to fringe; optimisation)
            if ( i == 0 || i+1 == size_x || j == 0 || j+1 == size_y )
            {
                continue;
            }

            // surrounded by other occupied cells (don't add to fringe; optimisation)
            if ( occ_grid.isOccupied(i+1, j) && occ_grid.isOccupied(i, j+1) &&
                 occ_grid.isOccupied(i-1, j) && occ_grid.isOccupied(i, j-1) &&
                 occ_grid.isOccupied(i+1, j+1) && occ_grid.isOccupied(i-1, j+1) &&
                 occ_grid.isOccupied(i+1, j-1) && occ_grid.isOccupied(i-1, j-1) )
            {
                continue;
            }

            fringe.push(index);
        }
    }
}
} // namespace cabin
