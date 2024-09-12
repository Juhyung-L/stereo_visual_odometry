#include <opencv2/imgproc.hpp>

#include "geometry_msgs/msg/point.hpp"
#include "cv_bridge/cv_bridge.h"

#include "visual_odometry/visualizer.hpp"

namespace VO
{
Visualizer::Visualizer()
: Node("VO_visualizer")
{
    pose_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "vo/poses", rclcpp::SystemDefaultsQoS());
    landmark_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "vo/landmarks", rclcpp::SystemDefaultsQoS());
    frame_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "vo/frame", rclcpp::SystemDefaultsQoS());
    ground_truth_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "vo/ground_truth", rclcpp::SystemDefaultsQoS());
}

void Visualizer::visualize(const Context& context, const std::vector<Sophus::SE3d>& poses)
{
    // visualizeFeatures(context);
    visualizeLandmarks(context);
    visualizePose(poses);
}

void Visualizer::visualizeFeatures(const Context& context)
{
    cv::Mat img_left;
    cv::Mat img_right;

    cv::cvtColor(context.frame_curr_->img_left_, img_left, cv::COLOR_GRAY2BGR);
    cv::cvtColor(context.frame_curr_->img_right_, img_right, cv::COLOR_GRAY2BGR);
    
    // draw lines connecting previous feature and current feature
    for (std::size_t i=0; i<context.frame_curr_->features_left_.size(); ++i)
    {
        cv::line(img_left, 
            context.frame_prev_->features_left_[i]->pixel,
            context.frame_curr_->features_left_[i]->pixel, 
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

    frame_pub_->publish(img_msg);
}

void Visualizer::visualizeLandmarks(const Context& context)
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
        p.x = feature->landmark_->pose_.x();
        p.y = feature->landmark_->pose_.y();
        p.z = feature->landmark_->pose_.z();
        m.points.push_back(p);
    }

    landmark_pub_->publish(m);
}

void Visualizer::visualizePose(const std::vector<Sophus::SE3d>& poses)
{
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";

    // delete the last pose
    if (prev_poses_id_ > -1)
    {
        m.id = prev_poses_id_;
        m.action = visualization_msgs::msg::Marker::DELETE;
        pose_pub_->publish(m);
    }
    ++prev_poses_id_;
    m.header.stamp = this->now();
    m.lifetime = rclcpp::Duration::from_seconds(0);
    m.frame_locked = false;
    m.id = prev_poses_id_;
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.scale.x = 1.0;
    m.color.a = 1.0;
    m.color.r = 1.0; // red
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point p;
    for (std::size_t i=1; i<poses.size(); ++i)
    {
        p.x = poses[i-1].translation().x();
        p.y = poses[i-1].translation().y();
        p.z = poses[i-1].translation().z();
        m.points.push_back(p);

        p.x = poses[i].translation().x();
        p.y = poses[i].translation().y();
        p.z = poses[i].translation().z();
        m.points.push_back(p);
    }
    pose_pub_->publish(m);
}

void Visualizer::visualizeGroundTruth(const std::vector<Sophus::SE3f>& ground_truth_poses)
{
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.lifetime = rclcpp::Duration::from_seconds(0);
    m.frame_locked = false;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.scale.x = 1.0;
    m.color.a = 1.0;
    m.color.r = 1.0;
    m.color.g = 1.0;
    m.color.b = 1.0;
    m.action = visualization_msgs::msg::Marker::ADD;

    visualization_msgs::msg::MarkerArray m_arr;
    geometry_msgs::msg::Point p;
    for (std::size_t i=1; i<ground_truth_poses.size(); ++i)
    {
        m.points.clear();
        m.header.stamp = this->now();
        m.id = this->now().nanoseconds();

        p.x = ground_truth_poses[i-1].translation().x();
        p.y = ground_truth_poses[i-1].translation().y();
        p.z = ground_truth_poses[i-1].translation().z();
        m.points.push_back(p);

        p.x = ground_truth_poses[i].translation().x();
        p.y = ground_truth_poses[i].translation().y();
        p.z = ground_truth_poses[i].translation().z();
        m.points.push_back(p);
        m_arr.markers.push_back(m);
    }
    ground_truth_pub_->publish(m_arr);
}
}