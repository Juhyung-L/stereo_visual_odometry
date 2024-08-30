#include <iostream>
#include <limits>

#include "visual_odometry/sensor/map.hpp"

namespace VO
{
void Map::insertKeyFrame(const std::shared_ptr<Frame> key_frame)
{
    curr_frame_ = key_frame;
    key_frames_[key_frame->id_] = key_frame;
    active_key_frames_[key_frame->id_] = key_frame;

    if (active_key_frames_.size() > num_active_key_frames_)
    {
        removeKeyFrame();
    }
}

void Map::insertLandmark(const std::shared_ptr<MapPoint> landmark)
{
    landmarks_[landmark->id_] = landmark;
    active_landmarks_[landmark->id_] = landmark;
}

void Map::removeKeyFrame()
{
    if (!curr_frame_)
    {
        return;
    }

    double max_dist = std::numeric_limits<double>::min();
    double min_dist = std::numeric_limits<double>::max();
    double max_kf_id = 0;
    double min_kf_id = 0;
    auto T_w_c = curr_frame_->pose_.inverse();
    for (const auto& kf : active_key_frames_)
    {
        if (kf.second == curr_frame_)
        {
            continue;
        }
        // inverse of current transformation * another transformation = transformation from current -> another
        // taking the logarithm map then the norm results in a distance value that considers both the 
        // translational and rotational difference of the two transformations
        double dist = (kf.second->pose_ * T_w_c).log().norm();
        if (dist > max_dist)
        {
            max_dist = dist;
            max_kf_id  = kf.first;
        }
        if (dist < min_dist)
        {
            min_dist = dist;
            min_kf_id = kf.first;
        }
    }

    std::shared_ptr<Frame> frame_to_remove{nullptr};
    if (min_dist < min_dist_th)
    {
        frame_to_remove = key_frames_.at(min_kf_id);
    }
    else
    {
        frame_to_remove = key_frames_.at(max_kf_id);
    }
    active_key_frames_.erase(frame_to_remove->id_);
}

void Map::cleanMap()
{
    int landmark_deletion_count = 0;
    for (auto it = active_landmarks_.begin(); it != active_landmarks_.end();)
    {
        std::vector<std::weak_ptr<Feature>>& observations = it->second->observations_;
        observations.erase(std::remove_if(observations.begin(), observations.end(),
            [](const std::weak_ptr<Feature> feature) {return feature.expired();}), 
            observations.end());

        // if after deleting all the expired features, the features list is empty,
        // delete the landmark from map
        if (observations.empty())
        {
            it = active_landmarks_.erase(it);
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