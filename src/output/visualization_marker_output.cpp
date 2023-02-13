#include <visualization_msgs/MarkerArray.h>

#include <yaml_common/Parser2.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/output/visualization_marker_output.h>
#include <cabin_nav/output/visualization_marker_output_data.h>

using Parser = kelo::yaml_common::Parser2;

namespace cabin {

bool VisualizationMarkerOutput::configure(const YAML::Node& config)
{
    if ( !Parser::read<std::string>(config, "topic", topic_) )
    {
        std::cerr << Print::Err << Print::Time() << "[VisualizationMarkerOutput] "
                  << "topic not provided" << Print::End << std::endl;
        return false;
    }
    queue_size_ = Parser::get<size_t>(config, "queue_size", 1);

    robot_frame_ = Parser::get<std::string>(config, "robot_frame", "base_link");
    global_frame_ = Parser::get<std::string>(config, "global_frame", "map");
    marker_namespace_ = Parser::get<std::string>(config, "marker_namespace", "navigation");

    use_refresh_style_ = Parser::get<bool>(config, "use_refresh_style", false);
    max_markers_ = Parser::get<size_t>(config, "max_markers", 50);

    default_marker_.pose.orientation.w = 1.0f;
    default_marker_.scale.x = 0.01f; // make it tiny
    default_marker_.scale.y = 0.01f;
    default_marker_.scale.z = 0.01f;
    default_marker_.color.a = 0.01f; // and almost transparent
    default_marker_.header.frame_id = robot_frame_;
    default_marker_.ns = marker_namespace_;

    return true;
}

void VisualizationMarkerOutput::initializeOutputData(
        OutputData::Ptr& output_data) const
{
    output_data = std::make_shared<VisualizationMarkerOutputData>();
}

bool VisualizationMarkerOutput::setData(
        const OutputData::Ptr& output_data,
        const InputData::Map& input_data_map)
{
    std::lock_guard<std::mutex> guard(loop_thread_mutex_);

    if ( output_data->getType() != getType() )
    {
        std::cout << Print::Err << Print::Time() << "[VisualizationMarkerOutput] "
                  << "output_data's type is not \"" << getType() << "\"."
                  << Print::End << std::endl;
        return false;
    }

    VisualizationMarkerOutputData::ConstPtr vis_output_data =
        std::static_pointer_cast<const VisualizationMarkerOutputData>(output_data);

    if ( !vis_output_data->markers.empty() )
    {
        visualization_msgs::MarkerArray marker_array_msg;
        if ( use_refresh_style_ )
        {
            marker_array_msg.markers.reserve(vis_output_data->markers.size());
            for ( size_t i = 0; i < vis_output_data->markers.size(); i++ )
            {
                visualization_msgs::Marker marker = vis_output_data->markers[i];
                if ( marker.header.frame_id == "robot" )
                {
                    marker.header.frame_id = robot_frame_;
                }
                else if ( marker.header.frame_id == "global" )
                {
                    marker.header.frame_id = global_frame_;
                }
                marker.ns = marker_namespace_;
                marker.id = i;
                marker_array_msg.markers.push_back(marker);
            }
            pubDeleteAllMarker();
        }
        else
        {
            size_t marker_size = vis_output_data->markers.size();
            marker_array_msg.markers.reserve(max_markers_);
            for ( size_t i = 0; i < max_markers_; i++ )
            {
                visualization_msgs::Marker marker;
                if ( i < marker_size )
                {
                    marker = vis_output_data->markers[i];
                    if ( marker.header.frame_id == "robot" )
                    {
                        marker.header.frame_id = robot_frame_;
                    }
                    else if ( marker.header.frame_id == "global" )
                    {
                        marker.header.frame_id = global_frame_;
                    }
                    marker.ns = marker_namespace_;
                }
                else
                {
                    marker = default_marker_;
                }
                marker.id = i;
                marker_array_msg.markers.push_back(marker);
            }
        }
        pub_.publish(marker_array_msg);
    }
    else
    {
        pubDeleteAllMarker();
    }
    return true;
}

void VisualizationMarkerOutput::start()
{
    if ( is_active_ )
    {
        std::cout << Print::Warn << Print::Time() << "[VisualizationMarkerOutput] "
                  << "Already active. Ignoring start() call."
                  << Print::End << std::endl;
        return;
    }

    pub_ = nh_.advertise<visualization_msgs::MarkerArray>(topic_, queue_size_);
    is_active_ = true;
}

void VisualizationMarkerOutput::stop()
{
    if ( !is_active_ )
    {
        std::cout << Print::Warn << Print::Time() << "[VisualizationMarkerOutput] "
                  << "Not active yet. Ignoring stop() call."
                  << Print::End << std::endl;
        return;
    }

    is_active_ = false;
}

void VisualizationMarkerOutput::step()
{
}

void VisualizationMarkerOutput::pubDeleteAllMarker()
{
    visualization_msgs::MarkerArray clear_marker_array_msg;
    visualization_msgs::Marker clear_marker;
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    clear_marker_array_msg.markers.push_back(clear_marker);
    pub_.publish(clear_marker_array_msg);
}

std::ostream& VisualizationMarkerOutput::write(std::ostream& out) const
{
    out << "<Output type: " << getType() << ">";
    return out;
}

} // namespace cabin
