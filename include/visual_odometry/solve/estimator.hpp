#ifndef ESTIMATOR_HPP_
#define ESTIMATOR_HPP_

#include <opencv2/core.hpp>

#include "visual_odometry/sensor/context.hpp"

namespace VO
{
class Estimator
{
public:
    Estimator();
    void estimate(Context& context, const cv::Matx33d& K);
};
}

#endif