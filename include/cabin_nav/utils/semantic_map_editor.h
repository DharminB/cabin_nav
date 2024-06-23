#pragma once

#include <unordered_map>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <interactive_markers/interactive_marker_server.h>

#include <yaml-cpp/yaml.h>

#include <geometry_common/Point2D.h>

#include <cabin_nav/semantic_map/semantic_map.h>

namespace cabin {

class SemanticMapEditor
{
    public:

        SemanticMapEditor():
            nh_("~"),
            interactive_marker_server_("corners") {}

        virtual ~SemanticMapEditor() = default;

        bool initialise();

        void save() const;

    private:

        ros::NodeHandle nh_;

        ros::Publisher connection_pub_;

        interactive_markers::InteractiveMarkerServer interactive_marker_server_;

        std::string semantic_map_file_;
        std::string frame_;

        YAML::Node data_;

        SemanticMap semantic_map_;

        visualization_msgs::MarkerArray getMarkerArray();

        std::array<float, 3> connection_color_{0.0f, 0.33f, 0.0f};
        std::array<float, 3> topology_color_{0.0f, 1.0f, 0.0f};

        void addInteractiveMarker(
                const std::string& name,
                const std::array<float, 3>& color,
                const kelo::geometry_common::Point2D& pt);

        void interactiveMarkerCb(
                const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

        void updateConnections();

};

} // namespace cabin
