#ifndef OPTIMIZER_HPP_
#define OPTIMIZER_HPP_

#include "visual_odometry/sensor/map.hpp"

namespace VO
{
class Optimizer
{
public:
    Optimizer() = default;
    void optimize(Map::KeyFrames keyframes, Map::Landmarks landmarks);
};
}

#endif