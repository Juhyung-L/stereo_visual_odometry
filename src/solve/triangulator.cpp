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

/**
 * - Estimates the 3D position of the 2D features using triangulation.
 * - The 3D points are made into MapPoints, which are inserted into the map.
 */

void Triangulator::triangulate(Context& context)
{
    std::vector<cv::Point2f> matches_2d_left = context.frame_prev_->getPointsLeft2D();
    std::vector<cv::Point2f>& matches_2d_right = context.frame_prev_->features_right_;

    std::cout << matches_2d_left.size() << " points triangulated\n";

    cv::Mat points_4d;
    std::vector<cv::Point3f> points_3d;
    
    // points_4d will become a 4 x N matrix where N is the number of points triangulated 
    // N == matches_2d_left.size() == matches_2d_right.size()
    cv::triangulatePoints(camera_left_.P_, camera_right_.P_, matches_2d_left, matches_2d_right, points_4d);

    cv::convertPointsFromHomogeneous(points_4d.t(), points_3d);

    // triangulated points are set as the corresponding 2D feature's landmark (MapPoint)
    for (std::size_t i=0; i<points_3d.size(); ++i)
    {
        std::shared_ptr<MapPoint>& landmark = context.frame_prev_->features_left_[i]->landmark_;
        if (!landmark)
        {
            landmark = std::make_shared<MapPoint>();
            map_->insertLandmark(landmark);
        }
        landmark->pose_.x() = points_3d[i].x;
        landmark->pose_.y() = points_3d[i].y;
        landmark->pose_.z() = points_3d[i].z;
        landmark->observations_.push_back(context.frame_prev_->features_left_[i]);

        // set landmark of the corresponding feature in the current frame
        context.frame_curr_->features_left_[i]->landmark_ = landmark;
    }
}
}