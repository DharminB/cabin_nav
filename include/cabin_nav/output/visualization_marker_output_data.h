#pragma once

#include <visualization_msgs/Marker.h>

#include <cabin_nav/output/output_data.h>

namespace cabin {

class VisualizationMarkerOutputData : public OutputData
{
    public:

        using Ptr = std::shared_ptr<VisualizationMarkerOutputData>;
        using ConstPtr = std::shared_ptr<const VisualizationMarkerOutputData>;

        std::vector<visualization_msgs::Marker> markers;

        VisualizationMarkerOutputData():
            OutputData("visualization_marker") {}

        virtual ~VisualizationMarkerOutputData() = default;

        void reset();

        std::ostream& write(std::ostream& out) const;

        static bool clear(
                const OutputData::Map& output_data_map,
                const std::string& output_name);

        static bool addMarker(
                const OutputData::Map& output_data_map,
                const std::string& output_name,
                const visualization_msgs::Marker& marker);

        static bool addMarkers(
                const OutputData::Map& output_data_map,
                const std::string& output_name,
                const std::vector<visualization_msgs::Marker>& markers);

        static bool setMarkers(
                const OutputData::Map& output_data_map,
                const std::string& output_name,
                const std::vector<visualization_msgs::Marker>& markers);

};

} // namespace cabin
