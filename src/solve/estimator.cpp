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
    
    // remove features if it was not used to estimate motion
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

    double distance_traveled_sq_ = 
        eigen_t.x()*eigen_t.x() +
        eigen_t.y()*eigen_t.y() +
        eigen_t.z()*eigen_t.z();
    if (distance_traveled_sq_ > max_allowed_translation_sq_)
    {
        return false;
    }

    // I thought it would be current_pose = T * prev_pose?
    context.frame_curr_->pose_ = context.frame_prev_->pose_ * T.inverse();

    // transform landmarks into world frame
    for (std::shared_ptr<Feature>& feature : context.frame_curr_->features_left_)
    {
        feature->landmark_->pose_ = context.frame_curr_->pose_ * feature->landmark_->pose_;
    }

    Sophus::SE3d& curr_pose = context.frame_curr_->pose_;
    std::cout << "x: " << curr_pose.translation().x() 
              << "y: " << curr_pose.translation().y() 
              << "z: " << curr_pose.translation().z() << std::endl;
    return true;
}
}