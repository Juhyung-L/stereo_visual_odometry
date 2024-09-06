#include <iostream>

#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

#include <ceres/ceres.h>

#include "visual_odometry/solve/optimizer.hpp"

namespace VO
{
void Optimizer::optimize(std::queue<std::shared_ptr<Frame>> keyframes, Map::Landmarks landmarks, const cv::Matx33d& cv_K)
{
    std::cout << "Optimizing\n";

    Eigen::Matrix3d K;
    cv::cv2eigen<double, 3, 3>(cv_K, K);
    ceres::Problem problem;
    ceres::Manifold* se3_parameterization = new SE3Parameterization();

}
}