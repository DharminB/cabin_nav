#ifndef CABIN_IMAGE_INPUT_H
#define CABIN_IMAGE_INPUT_H

#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <tf/transform_listener.h>

#include <geometry_common/TransformMatrix3D.h>

#include <cabin_nav/input/input_data.h>
#include <cabin_nav/input/input.h>
#include <cabin_nav/structs/img_data.h>

namespace cabin {

class ImageInput : public Input
{
    public:

        ImageInput():
            Input("image"),
            nh_("~") {}

        virtual ~ImageInput() = default;

        bool configure(const YAML::Node& config);

        bool getData(InputData::Ptr& input_data);

        void activate();

        void deactivate();

        std::ostream& write(std::ostream& out) const;

    protected:

        ros::NodeHandle nh_;

        std::mutex mutex_;
        std::string topic_;
        ros::Subscriber sub_;
        sensor_msgs::Image img_;

        std::mutex camera_info_mutex_;
        std::string camera_info_topic_;
        ros::Subscriber camera_info_sub_;
        sensor_msgs::CameraInfo camera_info_;

        kelo::geometry_common::TransformMatrix3D tf_;
        bool is_tf_valid_{false};
        std::string robot_frame_;
        cv::Mat cam_mat_;
        cv::Mat dist_coeff_;
        std::shared_ptr<tf::TransformListener> tf_listener_;

        void imgCb(const sensor_msgs::Image::ConstPtr& msg);

        void cameraInfoCb(const sensor_msgs::CameraInfo::ConstPtr& msg);

        bool initialiseTransformMat(const std::string& frame_id);

};

} // namespace cabin

#endif // CABIN_IMAGE_INPUT_H
