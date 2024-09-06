#ifndef FEATURE_HPP_
#define FEATURE_HPP_

#include <memory>

#include <opencv2/core.hpp>

#include "visual_odometry/sensor/map_point.hpp"
#include "visual_odometry/sensor/frame.hpp"

namespace VO
{
class MapPoint;
class Frame;
class Feature
{
public:
    Feature() = default;
    Feature(const std::weak_ptr<Frame>& frame, const cv::Point2f& point_2d)
    : frame_(frame)
    , point_2d_(point_2d)
    {}

    Feature(const std::weak_ptr<Frame>& frame, const cv::Point2f& point_2d, const std::shared_ptr<MapPoint> landmark)
    : frame_(frame)
    , point_2d_(point_2d)
    , landmark_(landmark)
    {}

    std::weak_ptr<Frame> frame_;
    cv::Point2f point_2d_;
    std::shared_ptr<MapPoint> landmark_{nullptr};
};
}

#endif