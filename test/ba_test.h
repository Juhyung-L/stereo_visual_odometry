#include <unordered_set>

#include <Eigen/Geometry>

#include <ceres/rotation.h>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/color_rgba.hpp"

#include "visual_odometry/sensor/map.hpp"
#include "visual_odometry/sensor/map_point.hpp"
#include "visual_odometry/sensor/frame.hpp"

using namespace VO;

class Visualizer : public rclcpp::Node
{
public:
    Visualizer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("vis_node", options)
    {
        poses_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "ba_test/poses", rclcpp::SystemDefaultsQoS());
        landmarks_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "ba_test/landmarks", rclcpp::SystemDefaultsQoS());
        correspondence_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "ba_test/correspondence", rclcpp::SystemDefaultsQoS());
    }

    void visualizeAll(
        const std::shared_ptr<Map>& map,
        const std_msgs::msg::ColorRGBA& poses_color,
        const std_msgs::msg::ColorRGBA& landmarks_color,
        const std_msgs::msg::ColorRGBA& /*correspondence_color*/)
    {
        visualizePoses(map, poses_color);
        visualizeLandmarks(map, landmarks_color);
        // visualizeCorrespondence(map, correspondence_color);
    }

    void clearAllMarkers()
    {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = this->now();
        m.action = visualization_msgs::msg::Marker::DELETEALL;
        landmarks_pub_->publish(m);
    }

private:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr poses_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr landmarks_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr correspondence_pub_;

    void visualizePoses(const std::shared_ptr<Map>& map,
        const std_msgs::msg::ColorRGBA& color)
    {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = this->now();
        m.lifetime = rclcpp::Duration::from_seconds(0);
        m.frame_locked = false;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.type = visualization_msgs::msg::Marker::ARROW;
        m.color = color;
        m.scale.x = 1.0; // length
        m.scale.y = 0.2; // width
        m.scale.z = 0.2; // height

        visualization_msgs::msg::MarkerArray m_arr;
        m_arr.markers.reserve(map->frames_.size());
        for (const std::shared_ptr<Frame>& frame : map->frames_)
        {
            m.id = this->now().nanoseconds();
            Sophus::Vector3d& pose = frame->pose_.translation();
            m.pose.position.x = pose.x();
            m.pose.position.y = pose.y();
            m.pose.position.z = pose.z();
            
            const Eigen::Quaterniond& q = frame->pose_.unit_quaternion();
            m.pose.orientation.x = q.x();
            m.pose.orientation.y = q.y();
            m.pose.orientation.z = q.z();
            m.pose.orientation.w = q.w();
            m_arr.markers.push_back(m);
        }

        poses_pub_->publish(m_arr);
    }

    void visualizeLandmarks(const std::shared_ptr<Map>& map,
        const std_msgs::msg::ColorRGBA& color)
    {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = this->now();
        m.lifetime = rclcpp::Duration::from_seconds(0);
        m.frame_locked = false;
        m.id = this->now().nanoseconds();
        m.action = visualization_msgs::msg::Marker::ADD;
        m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        m.color = color;
        m.scale.x = 0.1;
        m.scale.y = 0.1;
        m.scale.z = 0.1;
        m.points.reserve(map->landmarks_.size());
        for (const auto& landmark : map->landmarks_)
        {
            geometry_msgs::msg::Point p;
            p.x = landmark->pose_.x();
            p.y = landmark->pose_.y();
            p.z = landmark->pose_.z();
            m.points.push_back(p);
        }

        landmarks_pub_->publish(m);
    }

    void visualizeCorrespondence(const std::shared_ptr<Map>& map,
        const std_msgs::msg::ColorRGBA& color)
    {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = this->now();
        m.lifetime = rclcpp::Duration::from_seconds(0);
        m.frame_locked = false;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.type = visualization_msgs::msg::Marker::ARROW;
        m.color = color;
        m.scale.x = 0.01;
        m.scale.y = 0.01;
        m.scale.z = 0.01;
        m.points.reserve(2);

        visualization_msgs::msg::MarkerArray m_arr;
        for (const auto& landmark : map->landmarks_)
        {
            std::unordered_set<Frame*> frame_set;
            for (const std::weak_ptr<Feature>& weak : landmark->observations_)
            {
                m.id = this->now().nanoseconds();
                m.points.clear();
                const std::shared_ptr<Feature> observation = weak.lock();
                const std::shared_ptr<Frame>& frame = observation->frame_.lock();
                if (frame_set.find(frame.get()) == frame_set.end())
                {
                    geometry_msgs::msg::Point p;
                    p.x = landmark->pose_.x();
                    p.y = landmark->pose_.y();
                    p.z = landmark->pose_.z();
                    m.points.push_back(p);

                    p.x = frame->pose_.translation().x();
                    p.y = frame->pose_.translation().y();
                    p.z = frame->pose_.translation().z();
                    m.points.push_back(p);
                    frame_set.insert(frame.get());
                    m_arr.markers.push_back(m);
                }
            }
        }

        correspondence_pub_->publish(m_arr);
    }
};

class ReprojectionError
{
public:
    ReprojectionError(double observed_x, double observed_y, const Eigen::Matrix3d& intrinsics)
    : observed_x_(observed_x)
    , observed_y_(observed_y)
    , intrinsics_(intrinsics)
    {}

    template<typename T>
    bool operator()(const T* const extrinsics, const T* const p, T* residuals) const
    {
        T q[4];
        q[0] = extrinsics[3];
        q[1] = -extrinsics[2];
        q[2] = -extrinsics[1];
        q[3] = -extrinsics[0];

        T R[9];
        ceres::QuaternionToRotation(q, R);

        T p_w[3];
        p_w[0] = p[0];
        p_w[1] = p[1];
        p_w[2] = p[2];

        T p_c[3];
        p_c[0] = R[8]*p_w[0] + R[5]*p_w[1] + R[2]*p_w[2];
        p_c[1] = R[7]*p_w[0] + R[4]*p_w[1] + R[1]*p_w[2];
        p_c[2] = R[6]*p_w[0] + R[3]*p_w[1] + R[0]*p_w[2];

        p_c[0] -= R[8]*extrinsics[4] + R[5]*extrinsics[5] + R[2]*extrinsics[6];
        p_c[1] -= R[7]*extrinsics[4] + R[4]*extrinsics[5] + R[1]*extrinsics[6];
        p_c[2] -= R[6]*extrinsics[4] + R[3]*extrinsics[5] + R[0]*extrinsics[6];

        T predicted_x = -p_c[1] / p_c[0] * intrinsics_(0, 0) + intrinsics_(0, 2);
        T predicted_y = -p_c[2] / p_c[0] * intrinsics_(1, 1) + intrinsics_(1, 2);

        residuals[0] = predicted_x - observed_x_;
        residuals[1] = predicted_y - observed_y_;
        return true;
    }

private:
    double observed_x_;
    double observed_y_;
    Eigen::Matrix3d intrinsics_;
};