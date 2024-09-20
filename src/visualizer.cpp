#include <opencv2/imgproc.hpp>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "cv_bridge/cv_bridge.h"

#include "visual_odometry/visualizer.hpp"

namespace VO
{
Visualizer::Visualizer()
: Node("VO_visualizer")
{
    poses_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "vo/poses", rclcpp::SystemDefaultsQoS());
    landmark_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "vo/landmarks", rclcpp::SystemDefaultsQoS());
    frame_left_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "vo/frame_left", rclcpp::SystemDefaultsQoS());
    frame_right_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "vo/frame_right", rclcpp::SystemDefaultsQoS());
    ground_truth_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "vo/ground_truth", rclcpp::SystemDefaultsQoS());

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void Visualizer::visualize(const Context& context, const std::vector<Sophus::SE3d>& poses)
{
    if (!context.frame_prev_) {return;}
    // visualizeFeatures(context);
    // visualizeLandmarks(context);
    visualizePose(poses);
    // publishPoseTransform(poses.back());
}

void Visualizer::publishPoseTransform(const Sophus::SE3d& cur_pose)
{
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = "map";
    t.child_frame_id = "base_link";

    t.transform.translation.x = cur_pose.translation().x();
    t.transform.translation.y = cur_pose.translation().y();
    t.transform.translation.z = cur_pose.translation().z();

    t.transform.rotation.x = cur_pose.unit_quaternion().x();
    t.transform.rotation.y = cur_pose.unit_quaternion().y();
    t.transform.rotation.z = cur_pose.unit_quaternion().z();
    t.transform.rotation.w = cur_pose.unit_quaternion().w();

    tf_broadcaster_->sendTransform(t);
}

void Visualizer::visualizeFeatures(const Context& context)
{
    cv::Mat img_left;
    cv::Mat img_right;

    cv::cvtColor(context.frame_prev_->img_left_, img_left, cv::COLOR_GRAY2BGR);
    cv::cvtColor(context.frame_prev_->img_right_, img_right, cv::COLOR_GRAY2BGR);
    
    for (std::size_t i=0; i<context.frame_curr_->features_left_.size(); ++i)
    {
        cv::circle(img_left, 
            context.frame_prev_->features_left_[i]->pixel,
            3.0,
            cv::Scalar(0,0,255));
    }

    for (std::size_t i=0; i<context.frame_curr_->features_right_.size(); ++i)
    {
        cv::circle(img_right, 
            context.frame_prev_->features_right_[i],
            3.0, 
            cv::Scalar(0,0,255));
    }

    cv_bridge::CvImage cv_img;
    cv_img.header.stamp = this->now();
    cv_img.encoding = "bgr8";
    sensor_msgs::msg::Image img_msg;
    cv_img.image = img_left;
    cv_img.toImageMsg(img_msg);
    frame_left_pub_->publish(img_msg);

    cv_img.image = img_right;
    cv_img.toImageMsg(img_msg);
    frame_right_pub_->publish(img_msg);
}

void Visualizer::visualizeLandmarks(const Context& context)
{
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = this->now();
    m.lifetime = rclcpp::Duration::from_seconds(0);
    m.frame_locked = false;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.id = this->now().nanoseconds();
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.color.a = 1.0;
    m.color.r = 0.0;
    m.color.g = 1.0; // green
    m.color.b = 0.0;

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
    m.header.stamp = this->now();
    m.lifetime = rclcpp::Duration::from_seconds(0);
    m.frame_locked = false;
    m.action = visualization_msgs::msg::Marker::DELETE;

    // delete the last pose
    if (prev_lines_id_ > -1)
    {
        m.id = prev_lines_id_;
        poses_pub_->publish(m);
    }
    ++prev_lines_id_;
    m.id = prev_lines_id_;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.scale.x = 2.0;
    m.color.a = 1.0;
    m.color.r = 1.0; // red
    m.color.g = 0.0;
    m.color.b = 0.0;

    geometry_msgs::msg::Point p;
    for (const Sophus::SE3d& pose : poses)
    {
        p.x = pose.translation().x();
        p.y = pose.translation().y();
        p.z = pose.translation().z();
        m.points.push_back(p);
    }
    poses_pub_->publish(m);
}

void Visualizer::visualizeGroundTruth(const std::vector<Sophus::SE3f>& ground_truth_poses)
{
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.lifetime = rclcpp::Duration::from_seconds(0);
    m.frame_locked = false;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.scale.x = 1.0;
    m.color.a = 1.0;
    m.color.r = 1.0;
    m.color.g = 1.0;
    m.color.b = 1.0;

    geometry_msgs::msg::Point p;
    for (const Sophus::SE3f& pose : ground_truth_poses)
    {
        p.x = pose.translation().x();
        p.y = pose.translation().y();
        p.z = pose.translation().z();
        m.points.push_back(p);
    }
    ground_truth_pub_->publish(m);
}
}