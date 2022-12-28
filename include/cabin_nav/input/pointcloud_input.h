#ifndef CABIN_POINTCLOUD_INPUT_H
#define CABIN_POINTCLOUD_INPUT_H

#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>

#include <geometry_common/TransformMatrix3D.h>

#include <cabin_nav/input/input_data.h>
#include <cabin_nav/input/input.h>

namespace cabin {

class PointCloudInput : public Input
{
    public:

        PointCloudInput():
            nh_("~") {}

        virtual ~PointCloudInput() = default;

        bool configure(const YAML::Node& config);

        bool getData(InputData::Ptr& input_data, const std::string& input_name);

        void activate();

        void deactivate();

    protected:

        ros::NodeHandle nh_;
        ros::Subscriber sub_;

        std::string topic_;
        std::shared_ptr<tf::TransformListener> tf_listener_;

        kelo::geometry_common::TransformMatrix3D tf_;
        bool is_tf_valid_{false};
        std::string robot_frame_;

        std::mutex mutex_;
        sensor_msgs::PointCloud2 cloud_;

        size_t row_sub_sample_factor_{2};
        size_t col_sub_sample_factor_{2};

        void pointcloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg);

        bool initialiseTransformMat(const std::string& frame_id);

};

} // namespace cabin

#endif // CABIN_POINTCLOUD_INPUT_H
