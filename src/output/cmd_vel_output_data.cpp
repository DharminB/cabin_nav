#include <cabin_nav/output/cmd_vel_output_data.h>

namespace cabin {

void CmdVelOutputData::reset()
{
    trajectory.clear();
}

std::ostream& CmdVelOutputData::write(std::ostream& out) const
{
    out << "<OutputData type: " << getType() << ", trajectory: " << trajectory << ">";
    return out;
}

bool CmdVelOutputData::setTrajectory(
        const OutputData::Map& output_data_map,
        const Trajectory& _traj)
{
    if ( !OutputData::isValid(output_data_map, "cmd_vel", "cmd_vel") )
    {
        return false;
    }

    CmdVelOutputData::Ptr cmd_vel_output_data =
        std::static_pointer_cast<CmdVelOutputData>(
                output_data_map.at("cmd_vel"));

    cmd_vel_output_data->trajectory = _traj;

    return true;
}

} // namespace cabin
