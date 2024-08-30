#ifndef FRAME_HPP_
#define FRAME_HPP_

#include <memory>

#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#include "visual_odometry/sensor/feature.hpp"

namespace VO
{
class Frame
{
public:
    Frame() = default;
    Frame(const cv::Mat& img_left, const cv::Mat& img_right, unsigned long id)
    : img_left_(img_left)
    , img_right_(img_right)
    , id_(id)
    {}
    cv::Mat img_left_;
    cv::Mat img_right_;
    std::vector<std::shared_ptr<Feature>> features_left_;
    std::vector<cv::Point2f> features_right_;
    unsigned long id_;

    Sophus::SE3d pose_{Sophus::SE3d()};

    std::vector<cv::Point2f> getPointsLeft2D()
    {
        return featuresToPoint2D(features_left_);
    }

    std::vector<cv::Point3f> getPointsLeft3D()
    {
        return featuresToPoint3D(features_left_);
    }

    void pushFeatureLeft(const cv::Point2f& point_2d)
    {
        features_left_.push_back(std::make_shared<Feature>(id_, point_2d));
    }

private:
    std::vector<cv::Point2f> featuresToPoint2D(const std::vector<std::shared_ptr<Feature>>& features)
    {
        std::vector<cv::Point2f> points;
        points.reserve(features.size());
        for (const auto& feature : features)
        {
            points.push_back(feature->point_);
        }
        return points;
    }

    // all features need to have a landmark set
    // I think that's the case all the time?
    std::vector<cv::Point3f> featuresToPoint3D(const std::vector<std::shared_ptr<Feature>>& features)
    {
        std::vector<cv::Point3f> points;
        points.reserve(features.size());
        for (const auto& feature : features)
        {
            points.push_back(feature->landmark_->pose_);
        }
        return points;
    }
};
}

#endif