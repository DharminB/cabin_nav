#ifndef CABIN_OUTPUT_DATA_H
#define CABIN_OUTPUT_DATA_H

#include <cabin_nav/structs/trajectory_point.h>

namespace cabin {

struct OutputData
{
    using Ptr = std::shared_ptr<OutputData>;

    Trajectory trajectory;

    std::vector<visualization_msgs::Marker> markers;


    void reset()
    {
        trajectory.clear();
        markers.clear();
    }

    friend std::ostream& operator << (std::ostream& out, const OutputData& output_data)
    {
        out << "trajectory: " << std::endl;
        for ( size_t i = 0; i < output_data.trajectory.size(); i++ )
        {
            out << "    " << output_data.trajectory[i] << std::endl;
        }

        out << "markers: " << output_data.markers.size() << std::endl;
        return out;
    };
};

} // namespace cabin

#endif // CABIN_OUTPUT_DATA_H
