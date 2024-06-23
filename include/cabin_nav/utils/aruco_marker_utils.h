#pragma once

#include <opencv2/aruco.hpp>

#include <geometry_common/Pose2D.h>
#include <geometry_common/TransformMatrix3D.h>

namespace cabin {

class ArucoMarkerUtils
{
    public:

        ArucoMarkerUtils(float marker_length = 0.2f):
            marker_length_(marker_length),
            aruco_dictionary_(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000)),
            upright_marker_tf_(0.0f, 0.0f, 0.0f, -M_PI/2, -M_PI/2, 0.0f) {}

        virtual ~ArucoMarkerUtils() = default;

        bool getAllMarkerPoses(
                const cv::Mat& img,
                const cv::Mat& cam_mat,
                const cv::Mat& dist_coeff,
                const kelo::geometry_common::TransformMatrix3D& robot_to_cam_tf,
                std::vector<int>& valid_marker_ids,
                std::vector<kelo::geometry_common::Pose2D>& marker_poses,
                bool debug = true);

        bool getCartCenterPose(
                const std::vector<int>& marker_ids,
                const std::vector<kelo::geometry_common::Pose2D>& marker_poses,
                int cart_id,
                float cart_length,
                float cart_width,
                kelo::geometry_common::Pose2D& cart_center_pose,
                bool debug = true);

        bool getCartCenterPoseFromImg(
                const cv::Mat& img,
                const cv::Mat& cam_mat,
                const cv::Mat& dist_coeff,
                const kelo::geometry_common::TransformMatrix3D& robot_to_cam_tf,
                int cart_id,
                float cart_length,
                float cart_width,
                std::vector<int>& marker_ids,
                std::vector<kelo::geometry_common::Pose2D>& marker_poses,
                kelo::geometry_common::Pose2D& cart_center_pose,
                bool debug = true);

        void setMarkerLength(float marker_length);

    protected:
        float marker_length_;

        cv::Ptr<cv::aruco::Dictionary> aruco_dictionary_;

        kelo::geometry_common::TransformMatrix3D upright_marker_tf_;
};

} // namespace cabin
