#include <iostream>

#include <Eigen/Core>

#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <sophus/se3.hpp>

#include "visual_odometry/solve/estimator.hpp"

namespace VO
{
Estimator::Estimator()
{}

void Estimator::estimate(Context& context, const cv::Matx33d& K)
{
    std::vector<cv::Point2f> points_2d = context.frame_curr_->getPointsLeft2D();
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
    std::vector<std::shared_ptr<Feature>> left_feature_prev;
    std::vector<std::shared_ptr<Feature>> left_feature_curr;
    std::vector<cv::Point2f> right_feature_prev;
    std::vector<cv::Point2f> right_feature_curr;

    left_feature_prev.reserve(inliers.size());
    left_feature_curr.reserve(inliers.size());
    right_feature_prev.reserve(inliers.size());
    right_feature_curr.reserve(inliers.size());

    for (int inlier_idx : inliers)
    {
        left_feature_prev.push_back(context.frame_prev_->features_left_[inlier_idx]);
        left_feature_curr.push_back(context.frame_curr_->features_left_[inlier_idx]);
    
        right_feature_prev.push_back(context.frame_prev_->features_right_[inlier_idx]);
        right_feature_curr.push_back(context.frame_curr_->features_right_[inlier_idx]);
    }

    context.frame_prev_->features_left_ = left_feature_prev;
    context.frame_curr_->features_left_ = left_feature_curr;
    context.frame_prev_->features_right_ = right_feature_prev;
    context.frame_curr_->features_right_ = right_feature_curr;
    std::cout << inliers.size() << " features used to estimate motion\n";

    // convert the rotational vector intor rotational matrix using Rodrigues
    cv::Matx33d R = cv::Matx33d::eye();
    cv::Rodrigues(rvec, R);

    // convert to Sophus SE3d
    Eigen::Matrix3d eigen_R;
    Eigen::Vector3d eigen_T;
    cv::cv2eigen<double, 3, 3>(R, eigen_R);
    cv::cv2eigen<double, 3, 1>(tvec, eigen_T);

    Sophus::SE3d T(eigen_R, eigen_T);

    context.frame_curr_->pose_ = context.frame_prev_->pose_ * T.inverse();

    // transform landmarks into world frame
    for (std::shared_ptr<Feature>& feature : context.frame_curr_->features_left_)
    {
        cv::Point3f& point = feature->landmark_->pose_;
        Eigen::Vector3d v3d(point.x, point.y, point.z);
        v3d = context.frame_curr_->pose_ * v3d;
        point.x = v3d[0];
        point.y = v3d[1];
        point.z = v3d[2];
    }

    Sophus::SE3d& curr_pose = context.frame_curr_->pose_;
    std::cout << "x: " << curr_pose.translation().x() 
              << "y: " << curr_pose.translation().y() 
              << "z: " << curr_pose.translation().z() << std::endl;
}
}