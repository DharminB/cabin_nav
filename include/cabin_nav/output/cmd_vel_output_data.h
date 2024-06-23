#pragma once

#include <cabin_nav/output/output_data.h>

#include <cabin_nav/structs/trajectory_point.h>

namespace cabin {

class CmdVelOutputData : public OutputData
{
    public:

        using Ptr = std::shared_ptr<CmdVelOutputData>;
        using ConstPtr = std::shared_ptr<const CmdVelOutputData>;

        Trajectory trajectory;

        CmdVelOutputData():
            OutputData("cmd_vel") {}

        virtual ~CmdVelOutputData() = default;

        void reset();

        std::ostream& write(std::ostream& out) const;

        static bool setTrajectory(
                const OutputData::Map& output_data_map,
                const Trajectory& _traj);

};

} // namespace cabin
