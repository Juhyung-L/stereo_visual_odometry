#ifndef ESTIMATOR_HPP_
#define ESTIMATOR_HPP_

#include <opencv2/core.hpp>

#include "visual_odometry/sensor/context.hpp"

namespace VO
{
class Estimator
{
public:
    Estimator() = default;
    bool estimate(Context& context, const cv::Matx33d& K);

    double max_delta_pose_norm_{50.0};
};
}

#endif