#include <iostream>
#include <limits>

#include "visual_odometry/sensor/map.hpp"

namespace VO
{
void Map::insertFrame(const std::shared_ptr<Frame>& key_frame)
{
    frames_.push_back(key_frame);

    if (frames_.size() > num_active_frames_)
    {
        frames_.pop_front();
    }
}

void Map::insertLandmark(const std::shared_ptr<MapPoint>& landmark)
{
    landmarks_.insert(landmark);
}

void Map::cleanMap()
{
    int landmark_deletion_count = 0;
    auto it = landmarks_.begin();
    while (it != landmarks_.end())
    {
        std::vector<std::weak_ptr<Feature>>& observations = (*it)->observations_;
        bool all_expired = true;
        for (const std::weak_ptr<Feature>& feature : observations)
        {
            // if just one feature is not expired, all_expired will be false
            all_expired = all_expired && feature.expired();
        }

        // if all features are expired, erase the landmark
        if (all_expired)
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