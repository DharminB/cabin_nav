#include <cabin_nav/input/pointcloud_input_data.h>

namespace cabin {

std::ostream& PointCloudInputData::write(std::ostream& out) const
{
    out << "<Input type: " << getType() << ", pts: " << pts.size() << ">";
    return out;
}

bool PointCloudInputData::getPointCloud(
        const InputData::Map& input_data_map,
        const std::string& input_name,
        kelo::geometry_common::PointCloud3D& _pts)
{
    if ( !InputData::isValid(input_data_map, input_name, "pointcloud") )
    {
        return false;
    }

    PointCloudInputData::ConstPtr pointcloud_input_data =
        std::static_pointer_cast<PointCloudInputData>(
                input_data_map.at(input_name));

    _pts = pointcloud_input_data->pts;

    return true;
}

} // namespace cabin
