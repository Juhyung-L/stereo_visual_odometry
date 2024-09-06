#include <iostream>
#include <limits>

#include "visual_odometry/sensor/map.hpp"

namespace VO
{
void Map::insertKeyFrame(const std::shared_ptr<Frame> key_frame)
{
    key_frames_.push_back(key_frame);

    if (key_frames_.size() > num_active_key_frames_)
    {
        key_frames_.pop_front();
    }
}

void Map::insertLandmark(const std::shared_ptr<MapPoint> landmark)
{
    landmarks_.insert(landmark);
}

void Map::cleanMap()
{
    int landmark_deletion_count = 0;
    for (auto it = landmarks_.begin(); it != landmarks_.end();)
    {
        std::vector<std::weak_ptr<Feature>>& observations = (*it)->observations_;
        observations.erase(std::remove_if(observations.begin(), observations.end(),
            [](const std::weak_ptr<Feature> feature) {return feature.expired();}), 
            observations.end());

        // if after deleting all the expired features, the features list is empty,
        // delete the landmark from map
        if (observations.empty())
        {
            it = landmarks_.erase(it);
            ++landmark_deletion_count;
        }
        else
        {
            ++it;
        }
        
    }
    std::cout << landmark_deletion_count << " landmarks removed" << std::endl;
}
}