#ifndef MAP_HPP_
#define MAP_HPP_

#include <unordered_set>
#include <deque>
#include <memory>

#include "visual_odometry/sensor/map_point.hpp"
#include "visual_odometry/sensor/frame.hpp"

namespace VO
{
class Map
{
public:
    Map() = default;
    void insertKeyFrame(std::shared_ptr<Frame> key_frame);
    void insertLandmark(std::shared_ptr<MapPoint> landmark);
    void cleanMap();

    std::unordered_set<std::shared_ptr<MapPoint>> landmarks_;
    std::deque<std::shared_ptr<Frame>> key_frames_;

    const unsigned int num_active_key_frames_{10};
};
}

#endif