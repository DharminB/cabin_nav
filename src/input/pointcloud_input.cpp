#include <geometry_common/Utils.h>
#include <yaml_common/Parser2.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/input/pointcloud_input.h>
#include <cabin_nav/input/pointcloud_input_data.h>

using kelo::geometry_common::Point3D;
using GCUtils = kelo::geometry_common::Utils;
using Parser = kelo::yaml_common::Parser2;

namespace cabin {

bool PointCloudInput::configure(const YAML::Node& config)
{
    if ( !Parser::read<std::string>(config, "topic", topic_) )
    {
        std::cerr << Print::Err << Print::Time() << "[PointCloudInput] "
                  << "topic not provided"
                  << Print::End << std::endl;
        return false;
    }
    always_active_ = Parser::get<bool>(config, "always_active", false);
    robot_frame_ = Parser::get<std::string>(config, "robot_frame", "base_link");
    row_sub_sample_factor_ = Parser::get<size_t>(config, "row_sub_sample_factor", 2);
    col_sub_sample_factor_ = Parser::get<size_t>(config, "col_sub_sample_factor", 2);
    tf_listener_ = std::make_shared<tf::TransformListener>();
    return true;
}

bool PointCloudInput::getData(InputData::Ptr& input_data)
{
    std::lock_guard<std::mutex> guard(mutex_);

    if ( input_data == nullptr ) // for first iteration
    {
        input_data = std::make_shared<PointCloudInputData>();
    }

    if ( input_data->getType() != getType() )
    {
        std::cout << Print::Err << Print::Time() << "[PointCloudInput] "
                  << "input_data's type is not \"" << getType() << "\"."
                  << Print::End << std::endl;
        return false;
    }

    PointCloudInputData::Ptr pointcloud_input_data =
        std::static_pointer_cast<PointCloudInputData>(input_data);

    if ( !is_tf_valid_ )
    {
        if ( !initialiseTransformMat(cloud_.header.frame_id) )
        {
            std::cerr << Print::Warn << Print::Time() << "[PointCloudInput] "
                      << "Could not lookup transform from " << robot_frame_
                      << " to " << cloud_.header.frame_id
                      << Print::End << std::endl;
            return false;
        }
    }

    pointcloud_input_data->pts = GCUtils::convertToPointCloud3D(
                cloud_, row_sub_sample_factor_, col_sub_sample_factor_);
    tf_.transform(pointcloud_input_data->pts);

    /* sanity check */
    if ( pointcloud_input_data->pts.empty() )
    {
        std::cerr << Print::Warn << Print::Time() << "[PointCloudInput] "
                  << "No points" << Print::End << std::endl;
        return false;
    }

    return true;
}

void PointCloudInput::activate()
{
    if ( is_active_ )
    {
        std::cout << Print::Warn << Print::Time() << "[PointCloudInput] "
                  << "Already active. Ignoring activate() call"
                  << Print::End << std::endl;
        return;
    }

    is_active_ = true;
    sub_ = nh_.subscribe(topic_, 1, &PointCloudInput::pointcloudCb, this);
}

void PointCloudInput::deactivate()
{
    if ( !is_active_ )
    {
        std::cout << Print::Warn << Print::Time() << "[PointCloudInput] "
                  << "Already inactive. Ignoring deactivate() call"
                  << Print::End << std::endl;
        return;
    }

    if ( always_active_ )
    {
        return;
    }

    is_active_ = false;
    sub_.shutdown();
}

void PointCloudInput::pointcloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    std::lock_guard<std::mutex> guard(mutex_);
    cloud_ = sensor_msgs::PointCloud2(*msg);
}

bool PointCloudInput::initialiseTransformMat(const std::string& frame_id)
{
    tf::StampedTransform stamped_transform;
    try
    {
        tf_listener_->lookupTransform(robot_frame_, frame_id,
                                      ros::Time(0), stamped_transform);
        tf_.update(stamped_transform);
        is_tf_valid_ = true;
        return true;
    }
    catch ( const tf::TransformException& ex )
    {
        std::cout << Print::Warn << Print::Time() << "[PointCloudInput] "
                  << ex.what() << Print::End << std::endl;
        return false;
    }
}

std::ostream& PointCloudInput::write(std::ostream& out) const
{
    out << "<Input type: " << getType() << ">";
    return out;
}

} // namespace cabin
