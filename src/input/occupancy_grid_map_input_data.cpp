#include <cabin_nav/input/occupancy_grid_map_input_data.h>

namespace cabin {

std::ostream& OccupancyGridMapInputData::write(std::ostream& out) const
{
    out << "<Input type: " << getType() << ", occ_grid_map: " << occ_grid_map << ">";
    return out;
}

bool OccupancyGridMapInputData::getOccupancyGridMap(
        const InputData::Map& input_data_map,
        const std::string& input_name,
        OccupancyGrid::Ptr& _occ_grid_map)
{
    if ( !InputData::isValid(input_data_map, input_name, "occupancy_grid_map") )
    {
        return false;
    }

    OccupancyGridMapInputData::ConstPtr occupancy_grid_map_input_data =
        std::static_pointer_cast<OccupancyGridMapInputData>(
                input_data_map.at(input_name));

    _occ_grid_map = occupancy_grid_map_input_data->occ_grid_map;

    return true;
}

} // namespace cabin
