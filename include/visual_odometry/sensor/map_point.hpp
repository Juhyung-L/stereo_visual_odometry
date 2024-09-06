#ifndef MAP_POINT_HPP_
#define MAP_POINT_HPP_

#include <Eigen/Core>
#include <chrono>

#include "visual_odometry/sensor/feature.hpp"

namespace VO
{
class Feature;
class MapPoint
{
public:
    MapPoint() = default;
    MapPoint(const Eigen::Vector3d& pose)
    : pose_(pose)
    {}

    Eigen::Vector3d pose_;
    std::vector<std::weak_ptr<Feature>> observations_;
};
}

#endif