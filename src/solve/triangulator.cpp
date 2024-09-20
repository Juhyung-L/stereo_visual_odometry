#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include "visual_odometry/solve/triangulator.hpp"

namespace VO
{
Triangulator::Triangulator(const Camera& camera_left, const Camera& camera_right, std::shared_ptr<Map> map)
: map_(map)
, camera_left_(camera_left)
, camera_right_(camera_right)
{}

std::size_t Triangulator::firstFeatureWithoutLandmark(const std::vector<std::shared_ptr<Feature>>& features)
{
    for (std::size_t i=0; i<features.size(); ++i)
    {
        if (!features[i]->landmark_)
        {
            return i;
        }
    }
    return features.size();
}

/**
 * - Estimates the 3D position of the 2D features using triangulation.
 * - The 3D points are made into MapPoints, which are inserted into the map.
 */

void Triangulator::triangulate(Context& context)
{
    std::size_t start_idx = firstFeatureWithoutLandmark(context.frame_prev_->features_left_);
    // if all features already have a landmark, return immediately
    if (start_idx == context.frame_prev_->features_left_.size()) {return;}
    std::vector<cv::Point2f> matches_2d_left = context.frame_prev_->getPointsLeft2D(start_idx);
    std::vector<cv::Point2f> matches_2d_right(
        context.frame_prev_->features_right_.begin()+start_idx,
        context.frame_prev_->features_right_.end());

    cv::Mat points_4d;
    std::vector<cv::Point3f> points_3d;
    
    // points_4d will become a 4 x N matrix where N is the number of points triangulated 
    // N == matches_2d_left.size() == matches_2d_right.size()
    cv::triangulatePoints(camera_left_.P_, camera_right_.P_, matches_2d_left, matches_2d_right, points_4d);

    cv::convertPointsFromHomogeneous(points_4d.t(), points_3d);

    // triangulated points are set as the corresponding 2D feature's landmark (MapPoint)
    for (std::size_t i=0; i<points_3d.size(); ++i)
    {
        std::shared_ptr<MapPoint>& landmark = context.frame_prev_->features_left_[i+start_idx]->landmark_;
        landmark = std::make_shared<MapPoint>();
        landmark->pose_.x() = points_3d[i].x;
        landmark->pose_.y() = points_3d[i].y;
        landmark->pose_.z() = points_3d[i].z;
        // transform landmark into world frame
        landmark->pose_ = context.frame_prev_->pose_ * landmark->pose_;
        map_->insertLandmark(landmark);
        landmark->observations_.push_back(context.frame_prev_->features_left_[i+start_idx]);
    }
    std::cout << matches_2d_left.size() << " points triangulated\n";
}
}