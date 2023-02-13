#ifndef CABIN_VELOCITY_INPUT_H
#define CABIN_VELOCITY_INPUT_H

#include <mutex>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <geometry_common/XYTheta.h>

#include <cabin_nav/input/input_data.h>
#include <cabin_nav/input/input.h>

namespace cabin {

class VelocityInput : public Input
{
    public:

        VelocityInput():
            Input("velocity"),
            nh_("~") {}

        virtual ~VelocityInput() = default;

        bool configure(const YAML::Node& config);

        bool getData(InputData::Ptr& input_data);

        void activate();

        void deactivate();

        std::ostream& write(std::ostream& out) const;

    protected:

        ros::NodeHandle nh_;
        ros::Subscriber cmd_vel_sub_;
        ros::Subscriber odom_sub_;

        std::string cmd_vel_topic_;
        std::string odom_topic_;

        kelo::geometry_common::XYTheta tolerance_;

        std::mutex cmd_vel_cb_mutex_;
        kelo::geometry_common::Velocity2D prev_cmd_vel_;

        std::mutex odom_cb_mutex_;
        kelo::geometry_common::Velocity2D odom_vel_;

        void odomCb(const nav_msgs::Odometry::ConstPtr& msg);

        void cmdVelCb(const geometry_msgs::Twist::ConstPtr& msg);

};

} // namespace cabin

#endif // CABIN_VELOCITY_INPUT_H
