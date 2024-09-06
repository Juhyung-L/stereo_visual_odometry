#include "visual_odometry/sensor/frame.hpp"

namespace VO
{
std::vector<cv::Point2f> Frame::getPointsLeft2D()
{
    return featuresToPoint2D(features_left_);
}

std::vector<cv::Point3f> Frame::getPointsLeft3D()
{
    return featuresToPoint3D(features_left_);
}

void Frame::pushFeatureLeft(const cv::Point2f& point_2d)
{
    features_left_.push_back(std::make_shared<Feature>(shared_from_this(), point_2d));
}

std::vector<cv::Point2f> Frame::featuresToPoint2D(const std::vector<std::shared_ptr<Feature>>& features)
{
    std::vector<cv::Point2f> points;
    points.reserve(features.size());
    for (const auto& feature : features)
    {
        points.push_back(feature->pixel);
    }
    return points;
}

std::vector<cv::Point3f> Frame::featuresToPoint3D(const std::vector<std::shared_ptr<Feature>>& features)
{
    std::vector<cv::Point3f> points;
    points.reserve(features.size());
    for (const auto& feature : features)
    {
        points.emplace_back(feature->landmark_->pose_.x(), 
            feature->landmark_->pose_.y(), 
            feature->landmark_->pose_.z());
    }
    return points;
}
}