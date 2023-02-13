#ifndef CABIN_LASER_INPUT_H
#define CABIN_LASER_INPUT_H

#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

#include <geometry_common/TransformMatrix2D.h>
#include <geometry_common/TransformMatrix3D.h>

#include <cabin_nav/input/input_data.h>
#include <cabin_nav/input/input.h>

namespace cabin {

class LaserInput : public Input
{
    public:

        LaserInput():
            Input("laser"),
            nh_("~") {}

        virtual ~LaserInput() = default;

        bool configure(const YAML::Node& config);

        bool getData(InputData::Ptr& input_data);

        void activate();

        void deactivate();

        std::ostream& write(std::ostream& out) const;

    protected:

        ros::NodeHandle nh_;
        ros::Subscriber sub_;

        std::string topic_;
        std::shared_ptr<tf::TransformListener> tf_listener_;

        kelo::geometry_common::TransformMatrix3D tf_;
        bool is_tf_valid_{false};
        std::string robot_frame_;

        std::mutex mutex_;
        sensor_msgs::LaserScan scan_;

        void scanCb(const sensor_msgs::LaserScan::ConstPtr& msg);

        bool initialiseTransformMat(const std::string& frame_id);

};

} // namespace cabin

#endif // CABIN_LASER_INPUT_H
