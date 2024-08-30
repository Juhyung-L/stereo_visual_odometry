#ifndef MAP_HPP_
#define MAP_HPP_

#include <unordered_map>

#include "visual_odometry/sensor/map_point.hpp"
#include "visual_odometry/sensor/frame.hpp"

namespace VO
{
class Map
{
public:
    typedef std::unordered_map<unsigned long, std::shared_ptr<MapPoint>> Landmarks;
    typedef std::unordered_map<unsigned long, std::shared_ptr<Frame>> KeyFrames;

    Map() = default;
    void insertKeyFrame(std::shared_ptr<Frame> key_frame);
    void insertLandmark(std::shared_ptr<MapPoint> landmark);
    void removeKeyFrame();
    void cleanMap();

    std::shared_ptr<Frame> curr_frame_{nullptr};
    Landmarks landmarks_;
    Landmarks active_landmarks_;
    KeyFrames key_frames_;
    KeyFrames active_key_frames_;

    const unsigned int num_active_key_frames_{10};
    const double min_dist_th{0.2};
};
}

#endif