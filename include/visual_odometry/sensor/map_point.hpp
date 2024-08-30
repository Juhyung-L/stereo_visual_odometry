#ifndef MAP_POINT_HPP_
#define MAP_POINT_HPP_

#include <opencv2/core.hpp>
#include <chrono>

#include "visual_odometry/sensor/feature.hpp"

namespace VO
{
class Feature;
class MapPoint
{
public:
    MapPoint() = default;
    MapPoint(unsigned long id, const cv::Point3f& pose)
    : id_(id)
    , pose_(pose)
    {}

    unsigned long id_{0};
    cv::Point3f pose_;
    unsigned int times_observed_{0};
    std::vector<std::weak_ptr<Feature>> observations_;
};
}

#endif