#include <opencv2/calib3d.hpp>

#include <geometry_common/TransformMatrix2D.h>
#include <geometry_common/Utils.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/utils/aruco_marker_utils.h>

using kelo::geometry_common::Pose2D;
using kelo::geometry_common::TransformMatrix2D;
using kelo::geometry_common::TransformMatrix3D;
using GCUtils = kelo::geometry_common::Utils;

namespace cabin {

void ArucoMarkerUtils::setMarkerLength(float marker_length)
{
    marker_length_ = marker_length;
}

bool ArucoMarkerUtils::getAllMarkerPoses(
        const cv::Mat& img,
        const cv::Mat& cam_mat,
        const cv::Mat& dist_coeff,
        const TransformMatrix3D& robot_to_cam_tf,
        std::vector<int>& valid_marker_ids,
        std::vector<Pose2D>& marker_poses,
        bool debug)
{
    valid_marker_ids.clear();
    marker_poses.clear();

    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::aruco::detectMarkers(img, aruco_dictionary_, marker_corners, marker_ids);

    /* print marker ids */
    // std::cout << "marker_ids: ";
    // for ( size_t i = 0; i < marker_ids.size(); i++ )
    // {
    //     std::cout << marker_ids[i] << " ";
    // }
    // std::cout << std::endl;

    if ( marker_ids.empty() )
    {
        if ( debug )
        {
            std::cerr << Print::Warn << Print::Time() << "[ArucoMarkerUtils] "
                      << "Could not find any aruco markers in image"
                      << Print::End << std::endl;
        }
        return false;
    }

    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(
            marker_corners, marker_length_, cam_mat, dist_coeff, rvecs, tvecs);

    if ( rvecs.empty() )
    {
        if ( debug )
        {
            std::cerr << Print::Warn << Print::Time()
                      << "[aruco_marker_utils] Could not estimate poses of aruco markers"
                      << Print::End << std::endl;
        }
        return false;
    }

    if ( rvecs.size() != marker_ids.size() )
    {
        if ( debug )
        {
            std::cerr << Print::Warn << Print::Time()
                      << "[aruco_marker_utils] Marker transforms and marker ids have different sizes."
                      << Print::End << std::endl;
        }
        return false;
    }

    for ( size_t i = 0; i < marker_ids.size(); i++ )
    {
        if ( std::isnan(rvecs[i][0]) || std::isnan(rvecs[i][1]) || std::isnan(rvecs[i][2]) )
        {
            continue;
        }

        valid_marker_ids.push_back(marker_ids[i]);

        /* convert from rvec to rotation matrix */
        size_t marker_index = i;
        cv::Mat rot_mat;
        cv::Rodrigues(rvecs[marker_index], rot_mat);

        /* create a transformation matrix from rotation matrix and tvecs */
        TransformMatrix3D marker_in_cam_frame(
                tvecs[marker_index][0],
                tvecs[marker_index][1],
                tvecs[marker_index][2],
                std::atan2(rot_mat.at<double>(2, 1), rot_mat.at<double>(2, 2)), // roll
                std::atan2(-rot_mat.at<double>(2, 0), std::sqrt(
                        std::pow(rot_mat.at<double>(2, 1), 2) +
                        std::pow(rot_mat.at<double>(2, 2), 2))), // pitch
                std::atan2(rot_mat.at<double>(1, 0), rot_mat.at<double>(0, 0))); // yaw

        /* transform marker from camera frame to robot frame */
        TransformMatrix3D marker_in_robot_frame = robot_to_cam_tf * marker_in_cam_frame;

        /* rotate marker frame such that +ve Z is pointing up and
         * +ve X is pointing away from the surface of aruco marker */
        TransformMatrix3D marker_mat = marker_in_robot_frame * upright_marker_tf_;

        /* create a 2D pose from marker matrix */
        marker_poses.push_back(Pose2D(marker_mat.x(), marker_mat.y(), marker_mat.yaw()));
    }

    if ( valid_marker_ids.empty() )
    {
        if ( debug )
        {
            std::cerr << Print::Warn << Print::Time() << "[ArucoMarkerUtils] "
                      << "All aruco markers have nan transform "
                      << Print::End << std::endl;
        }
        return false;
    }
    else
    {
        return true;
    }
}

bool ArucoMarkerUtils::getCartCenterPose(
    const std::vector<int>& marker_ids,
    const std::vector<Pose2D>& marker_poses,
    int cart_id,
    float cart_length,
    float cart_width,
    Pose2D& cart_center_pose,
    bool debug)
{
    std::vector<size_t> valid_cart_id_indexes;
    for ( size_t i = 0; i < marker_ids.size(); i++ )
    {
        if ( (marker_ids[i] / 4) == cart_id )
        {
            valid_cart_id_indexes.push_back(i);
        }
    }

    if ( valid_cart_id_indexes.empty() )
    {
        return false;
    }

    if ( valid_cart_id_indexes.size() > 2 )
    {
        // Should never be here unless the light curves too much (only possible
        // in the vicinity of a black hole)
        if ( debug )
        {
            std::cerr << Print::Warn << Print::Time() << "[ArucoMarkerUtils] "
                      << "Perceived more than 2 markers. "
                      << "Ignoring all markers other than the first two."
                      << Print::End << std::endl;
        }
        valid_cart_id_indexes.erase(
                valid_cart_id_indexes.begin()+2, valid_cart_id_indexes.end());
    }

    std::vector<Pose2D> center_poses;
    for ( size_t i : valid_cart_id_indexes )
    {
        int side = marker_ids[i] % 4;
        TransformMatrix2D tf;
        switch ( side )
        {
            case 0: // front
                tf.updateX(-cart_length/2);
                break;
            case 1: // left
                tf.updateX(-cart_width/2);
                tf.updateTheta(-M_PI/2);
                break;
            case 2: // back
                tf.updateX(-cart_length/2);
                tf.updateTheta(M_PI);
                break;
            case 3: // right
                tf.updateX(-cart_width/2);
                tf.updateTheta(M_PI/2);
                break;
        }
        center_poses.push_back(Pose2D(TransformMatrix2D(marker_poses[i]) * tf));
    }

    cart_center_pose = GCUtils::calcMeanPose(center_poses);
    return true;
}


bool ArucoMarkerUtils::getCartCenterPoseFromImg(
        const cv::Mat& img,
        const cv::Mat& cam_mat,
        const cv::Mat& dist_coeff,
        const TransformMatrix3D& robot_to_cam_tf,
        int cart_id,
        float cart_length,
        float cart_width,
        std::vector<int>& marker_ids,
        std::vector<Pose2D>& marker_poses,
        Pose2D& cart_center_pose,
        bool debug)
{
    return ( getAllMarkerPoses(img, cam_mat, dist_coeff, robot_to_cam_tf,
                               marker_ids, marker_poses, debug) )
           ? getCartCenterPose(marker_ids, marker_poses, cart_id, cart_length,
                               cart_width, cart_center_pose, debug)
           : false;
}

} // namespace cabin
