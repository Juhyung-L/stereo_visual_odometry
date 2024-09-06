#include <iostream>
#include <fstream>
#include <vector>
#include <random>

#include <Eigen/Geometry>

#include <opencv2/core.hpp>

#include "std_msgs/msg/color_rgba.hpp"

#include "ba_test.h"

// copied from KITTI dataset
const double fx = 7.188560000000e+02;
const double fy = 7.188560000000e+02;
const double cx = 6.071928000000e+02;
const double cy = 1.852157000000e+02;
const double frame_width = 1920.0;
const double frame_height = 1080.0;

void makeMapFromData(std::string filename, std::shared_ptr<Map>& map)
{
    std::ifstream file(filename);

    if (!file.is_open())
    {
        throw std::runtime_error("Could not open file: " + filename);
    }

    std::string line;
    bool reading_poses = false;
    bool reading_landmarks = false;

    while (std::getline(file, line)) 
    {
        if (line == "camera_poses") 
        {
            reading_poses = true;
            reading_landmarks = false;
            continue;
        } 
        else if (line == "landmarks") 
        {
            reading_poses = false;
            reading_landmarks = true;
            continue;
        }

        if (reading_poses) 
        {
            double x, y, z, roll, pitch, yaw;
            if (sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf", &x, &y, &z, &roll, &pitch, &yaw) == 6) 
            {
                Eigen::Vector3d t(x, y, z);
                Eigen::Quaterniond q(
                    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())*
                    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())*
                    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
                std::shared_ptr<Frame> frame = std::make_shared<Frame>();
                frame->pose_ = Sophus::SE3d(q.toRotationMatrix(), t);
                map->insertKeyFrame(frame);
            }
        } 
        else if (reading_landmarks) 
        {
            double x, y, z;
            if (sscanf(line.c_str(), "%lf %lf %lf", &x, &y, &z) == 3) 
            {
                std::shared_ptr<MapPoint> landmark = std::make_shared<MapPoint>();
                landmark->pose_ = Eigen::Vector3d(x, y, z);
                map->insertLandmark(landmark);
            }
        }
    }
    file.close();
}

bool project3DtoPixel(const Eigen::Vector3d& point_3d, 
    const Eigen::Matrix3d& intrinsics,
    const double frame_width,
    const double frame_height,
    cv::Point2f& pixel)
{
    double pixel_x = -point_3d.y() / point_3d.x() * intrinsics(0, 0) + intrinsics(0, 2);
    double pixel_y = point_3d.z() / point_3d.x() * intrinsics(1, 1) + intrinsics(1, 2);
    
    pixel.x = pixel_x;
    pixel.y = pixel_y;
    return (0.0 <= pixel.x && pixel.x < frame_width &&
            0.0 <= pixel.y && pixel.y < frame_height);
}

void makeFeaturesAndCorrespondence(const std::shared_ptr<Map>& map,
    const Eigen::Matrix3d& intrinsics)
{    
    for (auto& frame : map->key_frames_)
    {
        for (auto& landmark : map->landmarks_)
        {
            // transform landmark from world frame to camera frame
            Eigen::Vector3d w_point(
                landmark->pose_.x(),
                landmark->pose_.y(),
                landmark->pose_.z());
            Eigen::Vector3d c_point = frame->pose_.inverse() * w_point;
            if (c_point.x() < 0.0) {continue;} // landmark is behind camera
            cv::Point2f pixel;
            // project 3d point onto image
            if (project3DtoPixel(c_point, intrinsics, frame_width, frame_height, pixel))
            {
                std::shared_ptr<Feature> feature = std::make_shared<Feature>(frame, pixel, landmark);
                frame->features_left_.push_back(feature);
                landmark->observations_.push_back(feature);
            }
        }
    }
}

std::shared_ptr<Map> copyMap(const std::shared_ptr<Map>& map)
{
    std::shared_ptr<Map> map_cpy = std::make_shared<Map>();
    for (const auto& frame : map->key_frames_)
    {
        std::shared_ptr<Frame> frame_cpy = std::make_shared<Frame>();
        frame_cpy->pose_ = frame->pose_;
        map_cpy->insertKeyFrame(frame_cpy);
        frame_cpy->features_left_.reserve(frame->features_left_.size());
        for (const auto& feature : frame->features_left_)
        {
            frame_cpy->pushFeatureLeft(feature->point_2d_);
        }
    }

    std::unordered_map<std::shared_ptr<MapPoint>, std::shared_ptr<MapPoint>> mp;
    for (const auto& landmark : map->landmarks_)
    {
        std::shared_ptr<MapPoint> landmark_cpy = std::make_shared<MapPoint>();
        landmark_cpy->pose_ = landmark->pose_;
        map_cpy->insertLandmark(landmark_cpy);
        mp[landmark] = landmark_cpy;
    }

    for (std::size_t i=0; i<map->key_frames_.size(); ++i)
    {
        for (std::size_t j=0; j<map->key_frames_[i]->features_left_.size(); ++j)
        {
            std::shared_ptr<MapPoint>& landmark_cpy = 
                mp[map->key_frames_[i]->features_left_[j]->landmark_];
            map_cpy->key_frames_[i]->features_left_[j]->landmark_ = landmark_cpy;
            landmark_cpy->observations_.push_back(map_cpy->key_frames_[i]->features_left_[j]);
        }
    }
    return map_cpy;
}

std::shared_ptr<Map> copyMapAndAddNoise(const std::shared_ptr<Map>& map)
{
    std::shared_ptr<Map> map_cpy = copyMap(map);

    std::random_device rd;
    std::mt19937 gen(rd());

    const double pose_translation_stddev = 0.1;
    const double pose_rotation_stddev = 0.05;
    const double landmark_stddev = 0.1;

    std::normal_distribution<double> pose_translation_noise(0, pose_translation_stddev);
    std::normal_distribution<double> pose_rotation_noise(0, pose_rotation_stddev);
    std::normal_distribution<double> landmark_noise(0, landmark_stddev);

    for (auto& frame : map_cpy->key_frames_)
    {
        frame->pose_.translation().x() += pose_translation_noise(gen);
        frame->pose_.translation().y() += pose_translation_noise(gen);
        frame->pose_.translation().z() += pose_translation_noise(gen);
    
        Eigen::Quaterniond q(frame->pose_.unit_quaternion());
        q.x() += pose_rotation_noise(gen);
        q.y() += pose_rotation_noise(gen);
        q.z() += pose_rotation_noise(gen);
        q.w() += pose_rotation_noise(gen);
        frame->pose_.setQuaternion(q);
    }
    
    for (auto& landmark : map_cpy->landmarks_)
    {
        landmark->pose_.x() += landmark_noise(gen);
        landmark->pose_.y() += landmark_noise(gen);
        landmark->pose_.z() += landmark_noise(gen);
    }

    return map_cpy;
}

void optimize(const std::shared_ptr<Map>& map, const Eigen::Matrix3d& intrinsics)
{
    ceres::Problem problem;

    std::vector<Sophus::SE3d> inverse_poses;
    inverse_poses.reserve(map->key_frames_.size());
    for (std::size_t i=0; i<map->key_frames_.size(); ++i)
    {
        std::shared_ptr<Frame>& frame = map->key_frames_[i];
        inverse_poses.push_back(frame->pose_.inverse());
        for (const auto& feature : frame->features_left_)
        {
            ReprojectionError* constraint = new ReprojectionError(
                static_cast<double>(feature->point_2d_.x),
                static_cast<double>(feature->point_2d_.y),
                intrinsics);
            ceres::CostFunction* cost_function = 
                new ceres::AutoDiffCostFunction<ReprojectionError, 2, 7, 3>(constraint);
            problem.AddResidualBlock(cost_function,
                nullptr,
                inverse_poses[i].data(),
                feature->landmark_->pose_.data());
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;

    for (std::size_t i=0; i<inverse_poses.size(); ++i)
    {
        map->key_frames_[i]->pose_ = inverse_poses[i].inverse();
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    Eigen::Matrix3d intrinsics;
    intrinsics << fx, 0.0, cx,      
        0.0, fy, cy, 
        0.0, 0.0, 1.0;

    std::string data_filename = "/home/dev_ws/src/visual_odometry/test/ba_data.txt";

    std::shared_ptr<Visualizer> visualizer = std::make_shared<Visualizer>();
    std::shared_ptr<Map> map = std::make_shared<Map>();
    makeMapFromData(data_filename, map);
    makeFeaturesAndCorrespondence(map, intrinsics);
    std_msgs::msg::ColorRGBA color1, color2, color3;
    color1.a = 1.0; color1.r = 1.0; color1.g = 0.0; color1.b = 0.0;
    color2.a = 1.0; color2.r = 1.0; color2.g = 0.5; color2.b = 0.0;
    color3.a = 1.0; color3.r = 1.0; color3.g = 0.5; color3.b = 1.0;
    visualizer->visualizeAll(map, color1, color2, color3);

    std::shared_ptr<Map> map_cpy = copyMapAndAddNoise(map);
    std_msgs::msg::ColorRGBA color4, color5, color6;
    color4.a = 1.0; color4.r = 0.0; color4.g = 0.0; color4.b = 1.0;
    color5.a = 1.0; color5.r = 1.0; color5.g = 1.0; color5.b = 1.0;
    color6.a = 1.0; color6.r = 0.0; color6.g = 0.5; color6.b = 1.0;
    visualizer->visualizeAll(map_cpy, color4, color5, color6);

    // visualizer->clearAllMarkers();
    optimize(map_cpy, intrinsics);

    std_msgs::msg::ColorRGBA color7, color8, color9;
    color7.a = 1.0; color7.r = 1.0; color7.g = 0.0; color7.b = 1.0;
    color8.a = 1.0; color8.r = 0.5; color8.g = 1.0; color8.b = 0.0;
    color9.a = 1.0; color9.r = 5.0; color9.g = 0.5; color9.b = 0.5;
    visualizer->visualizeAll(map_cpy, color7, color8, color9);

    rclcpp::spin(visualizer);
    rclcpp::shutdown();
    return 0;
}