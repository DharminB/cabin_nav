#include <ros/ros.h>

#include <cabin_nav/cabin_navigator_ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cabin_navigator");

    cabin::CABINNavigatorROS nav;

    if ( !nav.initialise() )
    {
        ros::shutdown();
        return 1;
    }

    if ( !nav.start() )
    {
        ROS_FATAL("Could not start CABINNavigator");
        ros::shutdown();
        return 1;
    }

    ROS_INFO("Initialised");
    ros::Rate rate(50.0f);
    while ( ros::ok() && nav.isActive() )
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
