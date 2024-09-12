#ifndef FRAME_HPP_
#define FRAME_HPP_

#include <memory>

#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#include "visual_odometry/sensor/feature.hpp"

namespace VO
{
class Feature;
class MapPoint;
class Frame : public std::enable_shared_from_this<Frame>
{
public:
    Frame() = default;
    Frame(const cv::Mat& img_left, const cv::Mat& img_right)
    : img_left_(img_left)
    , img_right_(img_right)
    {}
    cv::Mat img_left_;
    cv::Mat img_right_;
    std::vector<std::shared_ptr<Feature>> features_left_;
    std::vector<cv::Point2f> features_right_;

    Sophus::SE3d pose_{Sophus::SE3d()};

    std::vector<cv::Point2f> getPointsLeft2D(std::size_t start_idx);
    std::vector<cv::Point3f> getPointsLeft3D();
    void pushFeatureLeft(const cv::Point2f& point_2d);
    void pushFeatureLeftWithLandmark(const cv::Point2f& point_2d, const std::shared_ptr<MapPoint>& landmark);

private:
    std::vector<cv::Point2f> featuresToPoint2D(
        const std::vector<std::shared_ptr<Feature>>& features, std::size_t start_idx);
    std::vector<cv::Point3f> featuresToPoint3D(const std::vector<std::shared_ptr<Feature>>& features);
};
}

#endif