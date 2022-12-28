#ifndef CABIN_NAVIGATOR_ROS_H
#define CABIN_NAVIGATOR_ROS_H

#include <mutex>

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <cabin_nav/cabin_navigator.h>

namespace cabin {

class CABINNavigatorROS : public CABINNavigator
{
    public:

        CABINNavigatorROS():
            nh_("~") {}

        virtual ~CABINNavigatorROS() = default;

        virtual bool initialise();

    protected:

        ros::NodeHandle nh_;

        ros::Subscriber goal_sub_;
        ros::Subscriber task_request_sub_;
        ros::Subscriber cancel_sub_;

        void goalCb(const geometry_msgs::PoseStamped::ConstPtr& msg);

        void taskRequestCb(const std_msgs::String::ConstPtr& msg);

        void cancelCb(const std_msgs::Empty::ConstPtr& msg);

};

} // namespace cabin
#endif // CABIN_NAVIGATOR_ROS_H
