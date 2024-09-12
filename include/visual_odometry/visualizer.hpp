#ifndef VISUALIZER_HPP_
#define VISUALIZER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "visual_odometry/sensor/context.hpp"

namespace VO
{
class Visualizer : public rclcpp::Node
{
public:
    Visualizer();
    void visualize(const Context& context, const std::vector<Sophus::SE3d>& poses);
    void visualizeGroundTruth(const std::vector<Sophus::SE3f>& ground_truth_poses);

private:
    void visualizeLandmarks(const Context& context);
    void visualizeFeatures(const Context& context);
    void visualizePose(const std::vector<Sophus::SE3d>& poses);

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ground_truth_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr landmark_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr frame_pub_;

    int32_t prev_poses_id_{-1};
};
}

#endif