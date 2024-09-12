#include "visual_odometry/sensor/frame.hpp"

namespace VO
{
std::vector<cv::Point2f> Frame::getPointsLeft2D(std::size_t start_idx)
{
    return featuresToPoint2D(features_left_, start_idx);
}

std::vector<cv::Point3f> Frame::getPointsLeft3D()
{
    return featuresToPoint3D(features_left_);
}

void Frame::pushFeatureLeft(const cv::Point2f& point_2d)
{
    features_left_.push_back(std::make_shared<Feature>(shared_from_this(), point_2d));
}

void Frame::pushFeatureLeftWithLandmark(const cv::Point2f& point_2d, const std::shared_ptr<MapPoint>& landmark)
{
    std::shared_ptr<Feature> feature = std::make_shared<Feature>(shared_from_this(), point_2d, landmark);
    features_left_.push_back(feature);
    landmark->observations_.push_back(feature);
}

// convert std::vector<Feature> to std::vector<Point2f> starting at start_idx
std::vector<cv::Point2f> Frame::featuresToPoint2D(
    const std::vector<std::shared_ptr<Feature>>& features, std::size_t start_idx)
{
    std::vector<cv::Point2f> points;
    points.reserve(features.size() - start_idx);
    for (std::size_t i=start_idx; i<features.size(); ++i)
    {
        points.push_back(features[i]->pixel);
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