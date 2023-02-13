#include <cabin_nav/input/laser_input_data.h>

namespace cabin {

std::ostream& LaserInputData::write(std::ostream& out) const
{
    out << "<Input type: " << getType() << ", laser_pts: " << laser_pts.size() << ">";
    return out;
}

bool LaserInputData::getLaserPts(
        const InputData::Map& input_data_map,
        const std::string& input_name,
        kelo::geometry_common::PointCloud2D& _laser_pts)
{
    if ( !InputData::isValid(input_data_map, input_name, "laser") )
    {
        return false;
    }

    LaserInputData::ConstPtr laser_input_data =
        std::static_pointer_cast<LaserInputData>(
                input_data_map.at(input_name));

    _laser_pts = laser_input_data->laser_pts;

    return true;
}

bool LaserInputData::getLaserPts3D(
        const InputData::Map& input_data_map,
        const std::string& input_name,
        kelo::geometry_common::PointCloud3D& _laser_pts_3d)
{
    if ( !InputData::isValid(input_data_map, input_name, "laser") )
    {
        return false;
    }

    LaserInputData::ConstPtr laser_input_data =
        std::static_pointer_cast<LaserInputData>(
                input_data_map.at(input_name));

    _laser_pts_3d = laser_input_data->laser_pts_3d;

    return true;
}

} // namespace cabin
