#include <iostream>

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
    cv::Mat coeffs = cv::Mat::zeros(4, 1, CV_64FC1);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
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
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    cv::Rodrigues(rvec, R);

    // convert to Sophus SE3d
    Eigen::Matrix3d eigen_R;
    Eigen::Vector3d eigen_T;
    eigen_R << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
               R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
               R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);
    eigen_T << tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0);

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