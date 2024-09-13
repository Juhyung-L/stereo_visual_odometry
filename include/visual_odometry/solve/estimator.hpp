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

    double max_allowed_translation_sq_{10.0};
};
}

#endif