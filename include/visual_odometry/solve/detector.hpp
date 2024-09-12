#ifndef DETECTOR_HPP_
#define DETECTOR_HPP_

#include <opencv2/features2d.hpp>

#include "visual_odometry/sensor/context.hpp"

namespace VO
{
class Detector
{
public:
    Detector() = default;
    Detector(int grid_cell_size);
    void detectFeatures(Context& context);
private:
    int grid_cell_size_{15};
    cv::Ptr<cv::FeatureDetector> detector_;
};
}

#endif