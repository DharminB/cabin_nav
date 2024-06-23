#pragma once

#include <geometry_common/Point3D.h>

#include <cabin_nav/input/input_data.h>

namespace cabin {

class PointCloudInputData : public InputData
{
    public:

        using Ptr = std::shared_ptr<PointCloudInputData>;
        using ConstPtr = std::shared_ptr<const PointCloudInputData>;

        kelo::geometry_common::PointCloud3D pts;

        PointCloudInputData():
            InputData("pointcloud") {}

        virtual ~PointCloudInputData() = default;

        std::ostream& write(std::ostream& out) const;

        static bool getPointCloud(
                const InputData::Map& input_data_map,
                const std::string& input_name,
                kelo::geometry_common::PointCloud3D& _pts);

};

} // namespace cabin
