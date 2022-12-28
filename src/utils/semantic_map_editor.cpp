#include <iomanip>
#include <iostream>
#include <fstream>

#include <yaml_common/Parser2.h>
#include <geometry_common/Utils.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/utils/semantic_map_editor.h>

using kelo::geometry_common::Point2D;
using kelo::geometry_common::Polygon2D;
using Parser = kelo::yaml_common::Parser2;
using GCUtils = kelo::geometry_common::Utils;

namespace cabin {

bool SemanticMapEditor::initialise()
{
    nh_.param<std::string>("semantic_map_file", semantic_map_file_, "");

    if ( semantic_map_file_.empty() )
    {
        std::cout << Print::Err << Print::Time() << "[SemanticMapEditor] "
                  << "No semantic map file provided"
                  << Print::End << std::endl;
        return false;
    }
    nh_.param<std::string>("frame", frame_, "map");

    connection_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("connections", 1);

    ros::Duration(1.0f).sleep();

    if ( !semantic_map_.initialise(semantic_map_file_) )
    {
        std::cout << Print::Err << Print::Time() << "[SemanticMapEditor] "
                  << "Could not load semantic map file from path " << semantic_map_file_
                  << Print::End << std::endl;
        return false;
    }

    /* create interactive markers for each area */
    for ( auto it = semantic_map_.areas_.begin();
          it != semantic_map_.areas_.end();
          it ++ )
    {
        const Area::Ptr& area = it->second;
        const kelo::geometry_common::Polygon2D& polygon = area->getPolygon();
        for ( size_t i = 0; i < polygon.vertices.size(); i++ )
        {
            std::stringstream ss;
            ss << area->getName() << "____" << std::setfill('0') << std::setw(3) << i;
            const std::string identifier = ss.str();
            addInteractiveMarker(
                    identifier, Area::getcolorFromType(area->getType()), polygon.vertices[i]);
        }
    }

    /* create interactive markers for each topology node */
    for ( auto it = semantic_map_.topology_nodes_.begin();
          it != semantic_map_.topology_nodes_.end();
          it ++ )
    {
        const TopologyNode::Ptr& node = it->second;
        std::stringstream ss;
        ss << "TopologyNode____" << std::setfill('0') << std::setw(3) << node->getId();
        const std::string identifier = ss.str();
        addInteractiveMarker(
                identifier, topology_color_, node->getPosition());
    }

    updateConnections();

    std::cout << Print::Success << Print::Time() << "[SemanticMapEditor] "
              << "Initialised" << Print::End << std::endl;
    return true;
}

void SemanticMapEditor::addInteractiveMarker(
        const std::string& name,
        const std::array<float, 3>& color,
        const Point2D& pt)
{
    visualization_msgs::InteractiveMarker marker = pt.asInteractiveMarker(
            name, frame_, color[0], color[1], color[2], 1.0f, 0.3f);
    interactive_marker_server_.insert(
            marker, boost::bind(&SemanticMapEditor::interactiveMarkerCb, this, _1));
    interactive_marker_server_.applyChanges();
}

void SemanticMapEditor::interactiveMarkerCb(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
    if ( feedback->event_type != visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP )
    {
        return;
    }
    std::cout << feedback->marker_name << " is moved" << std::endl;

    size_t delimiter_index = feedback->marker_name.find("____");
    const std::string object_name = feedback->marker_name.substr(0, delimiter_index);
    if ( object_name.find("TopologyNode") != std::string::npos )
    {
        const size_t node_id = std::stoul(feedback->marker_name.substr(delimiter_index+4));
        semantic_map_.topology_nodes_[node_id]->position_.x =
            GCUtils::roundFloat(feedback->pose.position.x, 3);
        semantic_map_.topology_nodes_[node_id]->position_.y =
            GCUtils::roundFloat(feedback->pose.position.y, 3);
    }
    else // area corner was moved
    {
        const std::string area_name = object_name;
        const size_t area_corner_index = std::stoul(feedback->marker_name.substr(delimiter_index+4));
        semantic_map_.areas_[area_name]->polygon_.vertices[area_corner_index].x =
            GCUtils::roundFloat(feedback->pose.position.x, 3);
        semantic_map_.areas_[area_name]->polygon_.vertices[area_corner_index].y =
            GCUtils::roundFloat(feedback->pose.position.y, 3);
        semantic_map_.areas_[area_name]->center_ =
            semantic_map_.areas_[area_name]->polygon_.meanPoint();
    }
    updateConnections();
}

void SemanticMapEditor::updateConnections()
{
    connection_pub_.publish(getMarkerArray());
    ros::Duration(0.1f).sleep();
}

visualization_msgs::MarkerArray SemanticMapEditor::getMarkerArray()
{
    visualization_msgs::MarkerArray marker_array_msg;

    size_t marker_id = 0;

    /* markers for area polygons and their names */
    for ( auto it = semantic_map_.areas_.begin();
          it != semantic_map_.areas_.end();
          it ++ )
    {
        const Area::Ptr& area = it->second;
        visualization_msgs::Marker marker = area->asMarker(frame_, 1.0f, 0.1f);
        marker.header.stamp = ros::Time::now();
        marker.id = marker_id ++;
        marker.ns = "connections";
        marker_array_msg.markers.push_back(marker);

        visualization_msgs::Marker text_marker;
        text_marker.header.stamp = ros::Time::now();
        text_marker.header.frame_id = frame_;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.id = marker_id ++;
        text_marker.ns = "connections";
        text_marker.color = marker.color;
        float perimeter = std::fabs(area->getPolygon().length());
        text_marker.scale.z = GCUtils::clip(perimeter * 0.05f, 5.0f, 0.25f);
        text_marker.pose.position = area->getCenter().asPoint();
        text_marker.text = area->getName();
        marker_array_msg.markers.push_back(text_marker);
    }

    /* marker for all connections between areas */
    visualization_msgs::Marker connections_marker;
    connections_marker.header.stamp = ros::Time::now();
    connections_marker.header.frame_id = frame_;
    connections_marker.type = visualization_msgs::Marker::LINE_LIST;
    connections_marker.id = marker_id ++;
    connections_marker.ns = "connections";
    connections_marker.color.r = connection_color_[0];
    connections_marker.color.g = connection_color_[1];
    connections_marker.color.b = connection_color_[2];
    connections_marker.color.a = 0.7f;
    connections_marker.scale.x = 0.15f;
    connections_marker.pose.orientation.w = 1.0f;
    connections_marker.points.reserve(semantic_map_.connections_.size() * 2);
    for ( const Connection& connection : semantic_map_.connections_ )
    {
        connections_marker.points.push_back(
                semantic_map_.areas_.at(connection.area_1)->getCenter().asPoint());
        connections_marker.points.push_back(
                semantic_map_.areas_.at(connection.area_2)->getCenter().asPoint());
    }
    marker_array_msg.markers.push_back(connections_marker);

    /* markers for topology node ids */
    for ( auto it = semantic_map_.topology_nodes_.begin();
          it != semantic_map_.topology_nodes_.end();
          it ++ )
    {
        const TopologyNode::Ptr& node = it->second;

        visualization_msgs::Marker text_marker;
        text_marker.header.stamp = ros::Time::now();
        text_marker.header.frame_id = frame_;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.id = marker_id ++;
        text_marker.ns = "connections";
        /* black because light green is barely visible */
        text_marker.color.a = 1.0f;
        text_marker.scale.z = 0.5f;
        text_marker.pose.position = node->getPosition().asPoint();
        text_marker.pose.position.x += 0.3f;
        text_marker.pose.position.y += 0.3f;
        text_marker.text = std::to_string(node->getId());
        marker_array_msg.markers.push_back(text_marker);
    }

    /* marker for all connections between topology nodes */
    visualization_msgs::Marker topology_connections_marker;
    topology_connections_marker.header.stamp = ros::Time::now();
    topology_connections_marker.header.frame_id = frame_;
    topology_connections_marker.type = visualization_msgs::Marker::LINE_LIST;
    topology_connections_marker.id = marker_id ++;
    topology_connections_marker.ns = "connections";
    topology_connections_marker.color.r = topology_color_[0];
    topology_connections_marker.color.g = topology_color_[1];
    topology_connections_marker.color.b = topology_color_[2];
    topology_connections_marker.color.a = 0.7f;
    topology_connections_marker.scale.x = 0.15f;
    topology_connections_marker.pose.orientation.w = 1.0f;
    topology_connections_marker.points.reserve(semantic_map_.topology_connections_.size() * 2);
    for ( const TopologyConnection& connection : semantic_map_.topology_connections_ )
    {
        topology_connections_marker.points.push_back(
                semantic_map_.topology_nodes_.at(connection.node_1)->getPosition().asPoint());
        topology_connections_marker.points.push_back(
                semantic_map_.topology_nodes_.at(connection.node_2)->getPosition().asPoint());
    }
    marker_array_msg.markers.push_back(topology_connections_marker);

    return marker_array_msg;
}

void SemanticMapEditor::save() const
{
    YAML::Emitter out;
    out << semantic_map_;
    std::ofstream fout(semantic_map_file_);
    if ( fout.is_open() )
    {
        fout << out.c_str();
        fout.close();
        std::cout << Print::Success << Print::Time() << "[SemanticMapEditor] "
                  << "Saved in " << semantic_map_file_
                  << Print::End << std::endl;
    }
    else
    {
        std::cout << Print::Err << Print::Time() << "[SemanticMapEditor] "
                  << "Unable to open file " << semantic_map_file_ << ". "
                  << "Printing to standard output instead."
                  << Print::End << std::endl;
        std::cout << std::endl << std::endl << out.c_str() << std::endl << std::endl;
    }
}

} // namespace cabin

int main(int argc, char **argv)
{
    ros::init(argc, argv, "semantic_map_editor");

    cabin::SemanticMapEditor ed;

    if ( !ed.initialise() )
    {
        ros::shutdown();
        return 1;
    }

    ros::spin();

    std::cout << std::endl << std::endl;
    std::cout << "Would you like to save changes? [Y/n] : ";
    std::string choice;
    std::getline(std::cin, choice);
    if ( choice.empty() || choice.find_first_of("Yy") != std::string::npos )
    {
        ed.save();
    }

    return 0;
}
