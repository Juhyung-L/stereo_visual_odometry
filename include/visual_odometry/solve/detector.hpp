#ifndef DETECTOR_HPP_
#define DETECTOR_HPP_

#include <opencv2/features2d.hpp>

#include "visual_odometry/sensor/context.hpp"

namespace VO
{
class Detector
{
public:
    Detector();
    void detectFeatures(Context& context);
    
    int grid_cell_size_{5};
private:
    cv::Ptr<cv::FeatureDetector> detector_;
};
}

#endif