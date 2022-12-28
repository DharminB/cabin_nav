#include <cv_bridge/cv_bridge.h>

#include <yaml_common/Parser2.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/structs/img_data.h>
#include <cabin_nav/input/image_input.h>

using Parser = kelo::yaml_common::Parser2;

namespace cabin {

bool ImageInput::configure(const YAML::Node& config)
{
    if ( !Parser::read<std::string>(config, "topic", topic_) ||
         !Parser::read<std::string>(config, "camera_info_topic",
                                    camera_info_topic_) )
    {
        std::cerr << Print::Err << Print::Time() << "[ImageInput] "
                  << "topic and/or camera_info_topic not provided"
                  << Print::End << std::endl;
        return false;
    }
    always_active_ = Parser::get<bool>(config, "always_active", false);
    robot_frame_ = Parser::get<std::string>(config, "robot_frame", "base_link");
    tf_listener_ = std::make_shared<tf::TransformListener>();
    return true;
}

bool ImageInput::getData(InputData::Ptr& input_data, const std::string& input_name)
{
    std::lock_guard<std::mutex> guard(mutex_);
    std::lock_guard<std::mutex> cam_info_guard(camera_info_mutex_);

    if ( !is_tf_valid_ )
    {
        if ( initialiseTransformMat(camera_info_.header.frame_id) )
        {
            camera_info_sub_.shutdown();
            cam_mat_ = cv::Mat(3, 3, CV_64FC1, (void *) camera_info_.K.data());
            dist_coeff_ = cv::Mat(camera_info_.D.size(), 1, CV_64FC1, (void *) camera_info_.D.data());
            is_tf_valid_ = true;
        }
        else
        {
            std::cerr << Print::Warn << Print::Time() << "[ImageInput] "
                      << "Could not lookup transform from " << robot_frame_
                      << " to " << camera_info_.header.frame_id
                      << Print::End << std::endl;
            return false;
        }
    }

    input_data->img_data[input_name] = ImgData();
    try
    {
        input_data->img_data[input_name].img = cv_bridge::toCvCopy(img_)->image;
    }
    catch ( const cv_bridge::Exception& e )
    {
        ROS_WARN_STREAM("Could not convert image msg to cv::Mat for input " << topic_);
        return false;
    }
    input_data->img_data[input_name].cam_mat = cam_mat_;
    input_data->img_data[input_name].dist_coeff = dist_coeff_;
    input_data->img_data[input_name].robot_frame_to_cam_tf_mat = tf_;

    /* sanity check */
    if ( input_data->img_data[input_name].img.cols == 0 ||
         input_data->img_data[input_name].img.rows == 0 )
    {
        std::cerr << Print::Warn << Print::Time() << "[ImageInput] "
                  << "No image" << Print::End << std::endl;
        return false;
    }

    return true;
}

void ImageInput::activate()
{
    if ( is_active_ )
    {
        std::cout << Print::Warn << Print::Time() << "[ImageInput] "
                  << "Already active. Ignoring activate() call"
                  << Print::End << std::endl;
        return;
    }

    is_active_ = true;
    sub_ = nh_.subscribe(topic_, 1, &ImageInput::imgCb, this);
    camera_info_sub_ = nh_.subscribe(camera_info_topic_, 1, &ImageInput::cameraInfoCb, this);
}

void ImageInput::deactivate()
{
    if ( !is_active_ )
    {
        std::cout << Print::Warn << Print::Time() << "[ImageInput] "
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
    camera_info_sub_.shutdown();
}

void ImageInput::imgCb(const sensor_msgs::Image::ConstPtr& msg)
{
    std::lock_guard<std::mutex> guard(mutex_);
    img_ = sensor_msgs::Image(*msg);
}

void ImageInput::cameraInfoCb(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    std::lock_guard<std::mutex> guard(camera_info_mutex_);
    camera_info_ = sensor_msgs::CameraInfo(*msg);
}

bool ImageInput::initialiseTransformMat(const std::string& frame_id)
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


} // namespace cabin
