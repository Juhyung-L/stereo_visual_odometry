#include <iostream>

#include <Eigen/Core>

#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <sophus/se3.hpp>

#include "visual_odometry/solve/estimator.hpp"

namespace VO
{
bool Estimator::estimate(Context& context, const cv::Matx33d& K)
{
    std::vector<cv::Point2f> points_2d = context.frame_curr_->getPointsLeft2D(0);
    std::vector<cv::Point3f> points_3d = context.frame_curr_->getPointsLeft3D();

    /**
     * Perspective-N-Point (PNP) estimates the camera pose (rotational and translational vectors)
     * by minimizing the reprojection error of 3D points into 2D points.
     * The reprojection is computed using the camera intrinsic matrix K
     */

    std::vector<int> inliers;
    cv::Matx41d coeffs = cv::Matx41d::zeros();
    cv::Matx31d tvec = cv::Matx31d::zeros();
    cv::Matx31d rvec = cv::Matx31d::zeros();
    cv::solvePnPRansac(points_3d, points_2d, K, coeffs, rvec, tvec,
        false, 1000, 3.0, 0.99, inliers);
    
    // remove feature if it was not used to estimate motion
    std::vector<std::shared_ptr<Feature>> left_feature_curr;
    std::vector<cv::Point2f> right_feature_curr;

    left_feature_curr.reserve(inliers.size());
    right_feature_curr.reserve(inliers.size());

    for (int inlier_idx : inliers)
    {
        left_feature_curr.push_back(context.frame_curr_->features_left_[inlier_idx]);
        right_feature_curr.push_back(context.frame_curr_->features_right_[inlier_idx]);
    }

    context.frame_curr_->features_left_ = left_feature_curr;
    context.frame_curr_->features_right_ = right_feature_curr;
    std::cout << inliers.size() << " features used to estimate motion\n";

    // convert the rotational vector intor rotational matrix using Rodrigues
    cv::Matx33d R = cv::Matx33d::eye();
    cv::Rodrigues(rvec, R);

    // convert to Sophus SE3d
    Eigen::Matrix3d eigen_R;
    Eigen::Vector3d eigen_t;
    cv::cv2eigen<double, 3, 3>(R, eigen_R);
    cv::cv2eigen<double, 3, 1>(tvec, eigen_t);

    Sophus::SE3d T(eigen_R, eigen_t);
    Sophus::SE3d curr_pose = T.inverse();
    Sophus::SE3d delta_pose = curr_pose * context.frame_prev_->pose_.inverse();
    double delta_pose_norm = delta_pose.log().norm();
    if (delta_pose_norm > max_delta_pose_norm_)
    {
        return false;
    }

    context.frame_curr_->pose_ = curr_pose;

    std::cout << "x: " << curr_pose.translation().x() 
              << "y: " << curr_pose.translation().y() 
              << "z: " << curr_pose.translation().z() << std::endl;
    return true;
}
}