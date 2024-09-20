#ifndef OPTIMIZER_HPP_
#define OPTIMIZER_HPP_

#include <memory>

#include <ceres/rotation.h>

#include <opencv2/core.hpp>

#include "visual_odometry/sensor/map.hpp"

namespace VO
{
class Optimizer
{
public:
    Optimizer() = default;
    void optimize(const std::shared_ptr<Map>& map, 
        const cv::Matx33d& K, double loss_function_scale);
};

class ReprojectionError
{
public:
    ReprojectionError(double observed_x, double observed_y, const cv::Matx33d& intrinsics)
    : observed_x_(observed_x)
    , observed_y_(observed_y)
    , intrinsics_(intrinsics)
    {}

    template<typename T>
    bool operator()(const T* const extrinsics, const T* const p, T* residuals) const
    {
        T q[4];
        q[0] = extrinsics[3];
        q[1] = -extrinsics[2];
        q[2] = -extrinsics[1];
        q[3] = -extrinsics[0];

        T R[9];
        ceres::QuaternionToRotation(q, R);

        T p_w[3];
        p_w[0] = p[0];
        p_w[1] = p[1];
        p_w[2] = p[2];

        T p_c[3];
        p_c[0] = R[8]*p_w[0] + R[5]*p_w[1] + R[2]*p_w[2];
        p_c[1] = R[7]*p_w[0] + R[4]*p_w[1] + R[1]*p_w[2];
        p_c[2] = R[6]*p_w[0] + R[3]*p_w[1] + R[0]*p_w[2];

        p_c[0] -= R[8]*extrinsics[4] + R[5]*extrinsics[5] + R[2]*extrinsics[6];
        p_c[1] -= R[7]*extrinsics[4] + R[4]*extrinsics[5] + R[1]*extrinsics[6];
        p_c[2] -= R[6]*extrinsics[4] + R[3]*extrinsics[5] + R[0]*extrinsics[6];

        T predicted_x = p_c[0] / p_c[2] * intrinsics_(0, 0) + intrinsics_(0, 2);
        T predicted_y = p_c[1] / p_c[2] * intrinsics_(1, 1) + intrinsics_(1, 2);

        residuals[0] = predicted_x - observed_x_;
        residuals[1] = predicted_y - observed_y_;
        return true;
    }

private:
    double observed_x_;
    double observed_y_;
    cv::Matx33d intrinsics_;
};
}

#endif