#include <yaml_common/Parser2.h>
#include <geometry_common/Utils.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/input/velocity_input.h>

using kelo::geometry_common::XYTheta;
using GCUtils = kelo::geometry_common::Utils;
using Parser = kelo::yaml_common::Parser2;

namespace cabin {

bool VelocityInput::configure(const YAML::Node& config)
{
    if ( !Parser::read<std::string>(config, "cmd_vel_topic", cmd_vel_topic_) ||
         !Parser::read<std::string>(config, "odom_topic", odom_topic_) )
    {
        std::cout << Print::Err << Print::Time() << "[VelocityInput] "
                  << "Missing topic names."
                  << Print::End << std::endl;
        return false;
    }

    tolerance_ = Parser::get<XYTheta>(config, "tolerance", XYTheta(0.2f, 0.2f, 0.5f));
    always_active_ = Parser::get<bool>(config, "always_active", false);

    return true;
}

bool VelocityInput::getData(InputData::Ptr& input_data, const std::string& input_name)
{
    std::lock_guard<std::mutex> cmd_vel_guard(cmd_vel_cb_mutex_);
    std::lock_guard<std::mutex> odom_guard(odom_cb_mutex_);

    /* applyAccLimits is a misnomer here. Basically this function just clips the
     * prev_cmd_vel_ around odom_vel_ with certain tolerance_ (which is a weird
     * way is like applying acceleration limits to clip target velocity). This
     * is done in order to have a tradeoff between following 2 effects
     * 1. if only odom is considered, the acceleration that the robot executes
     * will be lower than intended. This is caused because there will be delay
     * between commanding a velocity and that velocity being executed and read
     * by the odometry calculation
     * 2. if only prev_cmd_vel is considered, the robot essentially drives open
     * loop which can be hazardous
     * Thus, by having some tolerance will counter the delay of odom but still
     * reduce the amount of danger caused by fully open loop control. */
    input_data->current_vel = GCUtils::applyAccLimits(
            prev_cmd_vel_, odom_vel_, tolerance_, 1.0f);

    return true;
}

void VelocityInput::activate()
{
    if ( is_active_ )
    {
        std::cout << Print::Warn << Print::Time() << "[VelocityInput] "
                  << "Already active. Ignoring activate() call"
                  << Print::End << std::endl;
        return;
    }

    is_active_ = true;
    cmd_vel_sub_ = nh_.subscribe(cmd_vel_topic_, 1, &VelocityInput::cmdVelCb, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 1, &VelocityInput::odomCb, this);
}

void VelocityInput::deactivate()
{
    if ( !is_active_ )
    {
        std::cout << Print::Warn << Print::Time() << "[VelocityInput] "
                  << "Already inactive. Ignoring deactivate() call"
                  << Print::End << std::endl;
        return;
    }

    if ( always_active_ )
    {
        return;
    }

    is_active_ = false;
    cmd_vel_sub_.shutdown();
    odom_sub_.shutdown();
}

void VelocityInput::odomCb(const nav_msgs::Odometry::ConstPtr& msg)
{
    std::lock_guard<std::mutex> guard(odom_cb_mutex_);
    odom_vel_.x = msg->twist.twist.linear.x;
    odom_vel_.y = msg->twist.twist.linear.y;
    odom_vel_.theta = msg->twist.twist.angular.z;
}

void VelocityInput::cmdVelCb(const geometry_msgs::Twist::ConstPtr& msg)
{
    std::lock_guard<std::mutex> guard(cmd_vel_cb_mutex_);
    prev_cmd_vel_.x = msg->linear.x;
    prev_cmd_vel_.y = msg->linear.y;
    prev_cmd_vel_.theta = msg->angular.z;
}

} // namespace cabin
