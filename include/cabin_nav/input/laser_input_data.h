#ifndef CABIN_LASER_INPUT_DATA_H
#define CABIN_LASER_INPUT_DATA_H

#include <geometry_common/Point2D.h>
#include <geometry_common/Point3D.h>

#include <cabin_nav/input/input_data.h>

namespace cabin {

class LaserInputData : public InputData
{
    public:

        using Ptr = std::shared_ptr<LaserInputData>;
        using ConstPtr = std::shared_ptr<const LaserInputData>;

        kelo::geometry_common::PointCloud2D laser_pts;
        kelo::geometry_common::PointCloud3D laser_pts_3d;

        LaserInputData():
            InputData("laser") {}

        virtual ~LaserInputData() = default;

        std::ostream& write(std::ostream& out) const;

        static bool getLaserPts(
                const InputData::Map& input_data_map,
                const std::string& input_name,
                kelo::geometry_common::PointCloud2D& _laser_pts);

        static bool getLaserPts3D(
                const InputData::Map& input_data_map,
                const std::string& input_name,
                kelo::geometry_common::PointCloud3D& _laser_pts_3d);

};

} // namespace cabin

#endif // CABIN_LASER_INPUT_DATA_H
