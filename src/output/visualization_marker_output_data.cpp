#include <cabin_nav/output/visualization_marker_output_data.h>

namespace cabin {

void VisualizationMarkerOutputData::reset()
{
    markers.clear();
}

std::ostream& VisualizationMarkerOutputData::write(std::ostream& out) const
{
    out << "<OutputData type: " << getType() << ", markers: " << markers.size() << ">";
    return out;
}

bool VisualizationMarkerOutputData::clear(
        const OutputData::Map& output_data_map,
        const std::string& output_name)
{
    if ( !OutputData::isValid(output_data_map, output_name, "visualization_marker") )
    {
        return false;
    }

    VisualizationMarkerOutputData::Ptr visualization_marker_output_data =
        std::static_pointer_cast<VisualizationMarkerOutputData>(
                output_data_map.at(output_name));

    visualization_marker_output_data->markers.clear();

    return true;
}

bool VisualizationMarkerOutputData::addMarker(
        const OutputData::Map& output_data_map,
        const std::string& output_name,
        const visualization_msgs::Marker& marker)
{
    if ( !OutputData::isValid(output_data_map, output_name, "visualization_marker") )
    {
        return false;
    }

    VisualizationMarkerOutputData::Ptr visualization_marker_output_data =
        std::static_pointer_cast<VisualizationMarkerOutputData>(
                output_data_map.at(output_name));

    visualization_marker_output_data->markers.push_back(marker);

    return true;
}

bool VisualizationMarkerOutputData::addMarkers(
        const OutputData::Map& output_data_map,
        const std::string& output_name,
        const std::vector<visualization_msgs::Marker>& markers)
{
    if ( !OutputData::isValid(output_data_map, output_name, "visualization_marker") )
    {
        return false;
    }

    VisualizationMarkerOutputData::Ptr visualization_marker_output_data =
        std::static_pointer_cast<VisualizationMarkerOutputData>(
                output_data_map.at(output_name));

    visualization_marker_output_data->markers.reserve(markers.size());
    visualization_marker_output_data->markers.insert(
            visualization_marker_output_data->markers.end(),
            markers.begin(), markers.end());

    return true;
}

bool VisualizationMarkerOutputData::setMarkers(
        const OutputData::Map& output_data_map,
        const std::string& output_name,
        const std::vector<visualization_msgs::Marker>& markers)
{
    if ( !OutputData::isValid(output_data_map, output_name, "visualization_marker") )
    {
        return false;
    }

    VisualizationMarkerOutputData::Ptr visualization_marker_output_data =
        std::static_pointer_cast<VisualizationMarkerOutputData>(
                output_data_map.at(output_name));

    visualization_marker_output_data->markers = markers;

    return true;
}

} // namespace cabin
