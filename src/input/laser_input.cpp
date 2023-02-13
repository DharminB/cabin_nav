#include <yaml_common/Parser2.h>
#include <geometry_common/Utils.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/input/laser_input.h>
#include <cabin_nav/input/laser_input_data.h>

using kelo::geometry_common::Point2D;
using kelo::geometry_common::Point3D;
using kelo::geometry_common::PointCloud3D;
using GCUtils = kelo::geometry_common::Utils;
using Parser = kelo::yaml_common::Parser2;

namespace cabin {

bool LaserInput::configure(const YAML::Node& config)
{
    if ( !Parser::read<std::string>(config, "topic", topic_) )
    {
        std::cerr << Print::Err << Print::Time() << "[LaserInput] "
                  << "topic not provided"
                  << Print::End << std::endl;
        return false;
    }
    always_active_ = Parser::get<bool>(config, "always_active", false);
    robot_frame_ = Parser::get<std::string>(config, "robot_frame", "base_link");
    tf_listener_ = std::make_shared<tf::TransformListener>();
    return true;
}

bool LaserInput::getData(InputData::Ptr& input_data)
{
    std::lock_guard<std::mutex> guard(mutex_);

    if ( input_data == nullptr ) // for first iteration
    {
        input_data = std::make_shared<LaserInputData>();
    }

    if ( input_data->getType() != getType() )
    {
        std::cout << Print::Err << Print::Time() << "[LaserInput] "
                  << "input_data's type is not \"" << getType() << "\"."
                  << Print::End << std::endl;
        return false;
    }

    LaserInputData::Ptr laser_input_data =
        std::static_pointer_cast<LaserInputData>(input_data);

    if ( !is_tf_valid_ )
    {
        if ( !initialiseTransformMat(scan_.header.frame_id) )
        {
            std::cout << Print::Warn << Print::Time() << "[LaserInput] "
                      << "Could not lookup transform from " << robot_frame_
                      << " to " << scan_.header.frame_id
                      << Print::End << std::endl;
            return false;
        }
    }

    PointCloud3D cloud = GCUtils::convertToPointCloud<Point3D>(scan_);
    tf_.transform(cloud);
    laser_input_data->laser_pts_3d = cloud;
    laser_input_data->laser_pts.clear();
    laser_input_data->laser_pts.reserve(cloud.size());
    for ( const Point3D& pt : cloud )
    {
        laser_input_data->laser_pts.push_back(Point2D(pt.x, pt.y));
    }

    /* sanity check */
    if ( laser_input_data->laser_pts_3d.empty() )
    {
        std::cout << Print::Warn << Print::Time() << "[LaserInput] "
                  << "No points"
                  << Print::End << std::endl;
        return false;
    }

    return true;
}

void LaserInput::activate()
{
    if ( is_active_ )
    {
        std::cout << Print::Warn << Print::Time() << "[LaserInput] "
                  << "Already active. Ignoring activate() call"
                  << Print::End << std::endl;
        return;
    }

    is_active_ = true;
    sub_ = nh_.subscribe(topic_, 1, &LaserInput::scanCb, this);
}

void LaserInput::deactivate()
{
    if ( !is_active_ )
    {
        std::cout << Print::Warn << Print::Time() << "[LaserInput] "
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

void LaserInput::scanCb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    std::lock_guard<std::mutex> guard(mutex_);
    scan_ = sensor_msgs::LaserScan(*msg);
}

bool LaserInput::initialiseTransformMat(const std::string& frame_id)
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
        std::cout << Print::Warn << Print::Time() << "[LaserInput] "
                  << ex.what() << Print::End << std::endl;
        return false;
    }
}

std::ostream& LaserInput::write(std::ostream& out) const
{
    out << "<Input type: " << getType() << ">";
    return out;
}

} // namespace cabin
