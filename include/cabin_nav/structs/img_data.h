#pragma once

#include <regex>

#include <opencv2/core.hpp>

#include <geometry_common/TransformMatrix3D.h>

#include <cabin_nav/utils/utils.h>

namespace cabin {

struct ImgData
{
    cv::Mat img;
    cv::Mat cam_mat;
    cv::Mat dist_coeff;
    kelo::geometry_common::TransformMatrix3D robot_frame_to_cam_tf_mat;

    friend std::ostream& operator << (std::ostream& out, const ImgData& img_data)
    {
        out << "img: " << img_data.img.cols << " x "
                       << img_data.img.rows << std::endl;
        out << "cam_mat: " << img_data.cam_mat.cols << " x "
                           << img_data.cam_mat.rows << std::endl;
        out << "dist_coeff: " << img_data.dist_coeff.cols << " x "
                              << img_data.dist_coeff.rows << std::endl;

        std::ostringstream tf_stream;
        tf_stream << "tf_mat: " << img_data.robot_frame_to_cam_tf_mat;
        std::string tf_string = tf_stream.str();
        tf_string = std::regex_replace(tf_string, std::regex("\n"), "\n        ");
        out << tf_string << std::endl;

        return out;
    };

};

} // namespace cabin
