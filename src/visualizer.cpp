#include <opencv2/imgproc.hpp>

#include "geometry_msgs/msg/point.hpp"
#include "cv_bridge/cv_bridge.h"

#include "visual_odometry/visualizer.hpp"

namespace VO
{
Visualizer::Visualizer()
: Node("VO_visualizer")
{
    lines_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "vo/lines", rclcpp::SystemDefaultsQoS());
    arrows_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "vo/arrows", rclcpp::SystemDefaultsQoS());
    landmark_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "vo/landmarks", rclcpp::SystemDefaultsQoS());
    frame_left_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "vo/frame_left", rclcpp::SystemDefaultsQoS());
    frame_right_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "vo/frame_right", rclcpp::SystemDefaultsQoS());
    ground_truth_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "vo/ground_truth", rclcpp::SystemDefaultsQoS());
}

void Visualizer::visualize(const Context& context, const std::vector<Sophus::SE3d>& poses)
{
    if (!context.frame_prev_) {return;}
    visualizeFeatures(context);
    visualizeLandmarks(context);
    visualizePoseAsLines(poses);
    // visualizePoseAsArrows(poses);
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
    m.id = this->now().nanoseconds();
    m.type = visualization_msgs::msg::Marker::POINTS;
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

void Visualizer::visualizePoseAsLines(const std::vector<Sophus::SE3d>& poses)
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
        lines_pub_->publish(m);
    }
    ++prev_lines_id_;
    m.id = prev_lines_id_;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.scale.x = 1.0;
    m.color.a = 1.0;
    m.color.r = 1.0; // red
    m.color.g = 0.0;
    m.color.b = 0.0;

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
    lines_pub_->publish(m);
}

void Visualizer::visualizePoseAsArrows(const std::vector<Sophus::SE3d>& poses)
{
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";

    // delete previous arrows
    m.action = visualization_msgs::msg::Marker::DELETE; 
    visualization_msgs::msg::MarkerArray m_arr;
    for (int32_t i=0; i<prev_arrows_id_; ++i)
    {
        m.id = i;
        m_arr.markers.push_back(m);
    }
    arrows_pub_->publish(m_arr);

    m.header.stamp = this->now();
    m.lifetime = rclcpp::Duration::from_seconds(0);
    m.frame_locked = false;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.scale.x = 1.0; // length
    m.scale.y = 0.2; // width
    m.scale.z = 0.2; // height
    m.color.a = 1.0;
    m.color.r = 1.0; // red
    m.color.g = 0.0;
    m.color.b = 0.0;

    m_arr.markers.clear();
    int32_t id = 0;

    Eigen::AngleAxisd roll(0.0, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch(-90.0 * M_PI / 180.0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw(0.0, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond camera_rotation = roll * pitch* yaw;
    for (const Sophus::SE3d& pose : poses)
    {
        m.id = id;
        m.pose.position.x = pose.translation().x();
        m.pose.position.y = pose.translation().y();
        m.pose.position.z = pose.translation().z();

        Eigen::Quaterniond q = pose.unit_quaternion();
        q *= camera_rotation;
        m.pose.orientation.x = q.x();
        m.pose.orientation.y = q.y();
        m.pose.orientation.z = q.z();
        m.pose.orientation.w = q.w();
        m_arr.markers.push_back(m);
        ++id;
    }
    arrows_pub_->publish(m_arr);
    prev_arrows_id_ = id;
}

void Visualizer::visualizeGroundTruth(const std::vector<Sophus::SE3f>& ground_truth_poses)
{
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.lifetime = rclcpp::Duration::from_seconds(0);
    m.frame_locked = false;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.scale.x = 0.6;
    m.color.a = 1.0;
    m.color.r = 1.0;
    m.color.g = 1.0;
    m.color.b = 1.0;

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