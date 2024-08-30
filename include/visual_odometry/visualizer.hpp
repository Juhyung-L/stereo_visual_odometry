#ifndef VISUALIZER_HPP_
#define VISUALIZER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "visual_odometry/sensor/context.hpp"

namespace VO
{
class Visualizer : public rclcpp::Node
{
public:
    Visualizer();
    void visualize(const Context& context);

private:
    void visualizeMapPoints(const Context& context);
    void visualizeFeatures(const Context& context);
    void visualizePose(const Context& context);

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr line_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
};
}

#endif