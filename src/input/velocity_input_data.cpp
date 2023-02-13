#include <cabin_nav/input/velocity_input_data.h>

namespace cabin {

std::ostream& VelocityInputData::write(std::ostream& out) const
{
    out << "<Input type: " << getType() << ", vel: " << vel << ">";
    return out;
}

bool VelocityInputData::getVelocity(
        const InputData::Map& input_data_map,
        const std::string& input_name,
        kelo::geometry_common::Velocity2D& _vel)
{
    if ( !InputData::isValid(input_data_map, input_name, "velocity") )
    {
        return false;
    }

    VelocityInputData::ConstPtr velocity_input_data =
        std::static_pointer_cast<VelocityInputData>(
                input_data_map.at(input_name));

    _vel = velocity_input_data->vel;

    return true;
}

} // namespace cabin
