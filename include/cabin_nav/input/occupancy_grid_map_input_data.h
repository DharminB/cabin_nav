#ifndef CABIN_OCCUPANCY_GRID_MAP_INPUT_DATA_H
#define CABIN_OCCUPANCY_GRID_MAP_INPUT_DATA_H

#include <cabin_nav/input/input_data.h>
#include <cabin_nav/structs/occupancy_grid.h>

namespace cabin {

class OccupancyGridMapInputData : public InputData
{
    public:

        using Ptr = std::shared_ptr<OccupancyGridMapInputData>;
        using ConstPtr = std::shared_ptr<const OccupancyGridMapInputData>;

        OccupancyGrid::Ptr occ_grid_map{nullptr};

        OccupancyGridMapInputData():
            InputData("occupancy_grid_map") {}

        virtual ~OccupancyGridMapInputData() = default;

        std::ostream& write(std::ostream& out) const;

        static bool getOccupancyGridMap(
                const InputData::Map& input_data_map,
                const std::string& input_name,
                OccupancyGrid::Ptr& _occ_grid_map);

};

} // namespace cabin

#endif // CABIN_OCCUPANCY_GRID_MAP_INPUT_DATA_H
