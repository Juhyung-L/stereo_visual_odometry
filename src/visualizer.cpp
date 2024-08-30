#include <opencv2/imgproc.hpp>

#include "geometry_msgs/msg/point.hpp"
#include "cv_bridge/cv_bridge.h"

#include "visual_odometry/visualizer.hpp"

namespace VO
{
Visualizer::Visualizer()
: Node("VO_visualizer")
{
    line_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "vo_line", rclcpp::SystemDefaultsQoS());
    points_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "vo_points", rclcpp::SystemDefaultsQoS());
    img_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "vo_image", rclcpp::SystemDefaultsQoS());
}

void Visualizer::visualize(const Context& context)
{
    // visualizeFeatures(context);
    // visualizeMapPoints(context);
    visualizePose(context);
}

void Visualizer::visualizeFeatures(const Context& context)
{
    cv::Mat img_left;
    cv::Mat img_right;
    // cv::Mat img_left = context.frame_curr_->img_left_.clone();
    // cv::Mat img_right = context.frame_curr_->img_right_.clone();
    cv::cvtColor(context.frame_curr_->img_left_, img_left, cv::COLOR_GRAY2BGR);
    cv::cvtColor(context.frame_curr_->img_right_, img_right, cv::COLOR_GRAY2BGR);
    
    // draw lines connecting previous feature and current feature
    for (std::size_t i=0; i<context.frame_curr_->features_left_.size(); ++i)
    {
        cv::line(img_left, 
            context.frame_prev_->features_left_[i]->point_,
            context.frame_curr_->features_left_[i]->point_, 
            cv::Scalar(0,0,255));
    }

    for (std::size_t i=0; i<context.frame_curr_->features_right_.size(); ++i)
    {
        cv::line(img_right, 
            context.frame_prev_->features_right_[i],
            context.frame_curr_->features_right_[i], 
            cv::Scalar(0,0,255));
    }
    
    cv::Mat both_imgs(img_left.rows, img_left.cols+img_right.cols, CV_8UC3);
    cv::hconcat(img_left, img_right, both_imgs);
    // for (std::size_t i=0; i<context.frame_curr_->features_left_.size(); ++i)
    // {
    //     cv::line(both_imgs,
    //         context.frame_curr_->features_left_[i]->point_,
    //         cv::Point(context.frame_curr_->features_right_[i].x+img_left.cols, context.frame_curr_->features_right_[i].y),
    //         cv::Scalar(0,0,255));
    // }

    sensor_msgs::msg::Image img_msg;
    cv_bridge::CvImage cv_img;
    cv_img.header.stamp = this->now();
    cv_img.encoding = "bgr8";
    cv_img.image = both_imgs;
    cv_img.toImageMsg(img_msg);

    img_pub_->publish(img_msg);
}

void Visualizer::visualizeMapPoints(const Context& context)
{
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = this->now();
    m.lifetime = rclcpp::Duration::from_seconds(0);
    m.frame_locked = false;
    m.id = this->now().nanoseconds();
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.color.a = 1.0;
    m.color.r = 0.0;
    m.color.g = 1.0; // green
    m.color.b = 0.0;
    m.action = visualization_msgs::msg::Marker::ADD;

    for (const auto& feature : context.frame_curr_->features_left_)
    {
        geometry_msgs::msg::Point p;
        p.x = feature->landmark_->pose_.x;
        p.y = feature->landmark_->pose_.y;
        p.z = feature->landmark_->pose_.z;
        m.points.push_back(p);
    }

    points_pub_->publish(m);
}

void Visualizer::visualizePose(const Context& context)
{
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = this->now();
    m.lifetime = rclcpp::Duration::from_seconds(0);
    m.frame_locked = false;
    m.id = this->now().nanoseconds();
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.scale.x = 1.0;
    m.color.a = 1.0;
    m.color.r = 1.0; // red
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point p;
    p.x = context.frame_prev_->pose_.translation().x();
    p.y = context.frame_prev_->pose_.translation().y();
    p.z = context.frame_prev_->pose_.translation().z();
    m.points.push_back(p);

    p.x = context.frame_curr_->pose_.translation().x();
    p.y = context.frame_curr_->pose_.translation().y();
    p.z = context.frame_curr_->pose_.translation().z();
    m.points.push_back(p);

    line_pub_->publish(m);
}
}