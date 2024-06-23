#pragma once

#include <geometry_common/XYTheta.h>

#include <cabin_nav/input/input_data.h>

namespace cabin {

class VelocityInputData : public InputData
{
    public:

        using Ptr = std::shared_ptr<VelocityInputData>;
        using ConstPtr = std::shared_ptr<const VelocityInputData>;

        kelo::geometry_common::Velocity2D vel;
        kelo::geometry_common::Velocity2D prev_cmd_vel;
        kelo::geometry_common::Velocity2D odom_vel;

        VelocityInputData():
            InputData("velocity") {}

        virtual ~VelocityInputData() = default;

        std::ostream& write(std::ostream& out) const;

        static bool getVelocity(
                const InputData::Map& input_data_map,
                const std::string& input_name,
                kelo::geometry_common::Velocity2D& _vel);

};

} // namespace cabin
