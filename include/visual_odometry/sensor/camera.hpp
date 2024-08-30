#ifndef CAMERA_HPP_
#define CAMERA_HPP_

#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

namespace VO
{
class Camera
{
public:
    Camera() = default;
    Camera(double fx, double fy, double cx, double cy, double tx, double ty, double tz)
    : fx_(fx), fy_(fy), cx_(cx), cy_(cy), tx_(tx), ty_(ty), tz_(tz)
    {
        K_ << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
        P_ << fx_, 0, cx_, tx_, 0, fy_, cy_, ty_, 0, 0, 1, tz_;
    }

    Sophus::SE3d pose_;
    Sophus::SE3d pose_inverse_;

    cv::Point3f pixelToWorld(const cv::Point2f& pixel, double depth)
    {
        return cv::Point3f(
            (pixel.x - cx_) * depth / fx_,
            (pixel.y - cy_) * depth / fy_,
            depth);
    }

    cv::Matx33d K_;
    cv::Matx34d P_;

private:
    double fx_{0.0};
    double fy_{0.0};
    double cx_{0.0};
    double cy_{0.0};
    double tx_{0.0};
    double ty_{0.0};
    double tz_{0.0};
};
}

#endif