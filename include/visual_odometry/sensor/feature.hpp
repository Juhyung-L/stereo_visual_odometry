#ifndef FEATURE_HPP_
#define FEATURE_HPP_

#include <memory>

#include <opencv2/core.hpp>

#include "visual_odometry/sensor/map_point.hpp"

namespace VO
{
class MapPoint;
class Feature
{
public:
    Feature() = default;
    Feature(unsigned long frame_id, const cv::Point2f& point)
    : frame_id_(frame_id)
    , point_(point)
    {}

    Feature(unsigned long frame_id, const cv::Point2f& point, const std::shared_ptr<MapPoint> landmark)
    : frame_id_(frame_id)
    , point_(point)
    , landmark_(landmark)
    {}

    unsigned long id_{0};
    unsigned long frame_id_{0};
    cv::Point2f point_;
    bool is_inlier_{false};
    std::shared_ptr<MapPoint> landmark_{nullptr};
};
}

#endif