#include <fstream>
#include <sstream>
#include <iomanip>

#include "visual_odometry/parse_KITTI.hpp"

// this code is very specific to the file structure of the KITTI dataset

namespace VO
{
ParseKITTI::ParseKITTI()
{}

void ParseKITTI::loadFrames(const std::string& frame_filename)
{
    std::string time_file = frame_filename + "/times.txt";
    std::ifstream file;
    file.open(time_file);
    if (!file)
    {
        throw std::runtime_error("Could not open file: " + time_file);
    }
    frames_.clear();
    int frame_idx = 0;
    std::string line;
    while (std::getline(file, line))
    {
        StereoPair pair;
        pair.time_ = std::stod(line);
        pair.left_frame_ = getFrame(frame_filename + "/image_0/", frame_idx);
        pair.right_frame_ = getFrame(frame_filename + "/image_1/", frame_idx);
        frames_.push_back(pair);
        ++frame_idx;
    }
    file.close();
}

void ParseKITTI::loadCameras(const std::string& calib_filename)
{
    std::ifstream file;
    file.open(calib_filename);
    if (!file)
    {
        throw std::runtime_error("Could not open file: " + calib_filename);
    }

    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        char _[3];
        for (int i=0; i<3; ++i)
        {
            iss >> _[i];
        }
        double P[12];
        for (int i=0; i<12; ++i)
        {
            iss >> P[i];
        }
        cameras_.emplace_back(P[0], P[5], P[2], P[6], P[3], P[7], P[11]);
    }
    file.close();
}

std::string ParseKITTI::getFrame(const std::string& path_to_frames, int frame_idx)
{
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(6) << frame_idx;
    return path_to_frames + ss.str() + ".png";
}

void ParseKITTI::loadGroundTruth(const std::string& ground_truth_filename)
{
    std::ifstream file;
    file.open(ground_truth_filename);
    if (!file)
    {
        throw std::runtime_error("Could not open file: " + ground_truth_filename);
    }

    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        Sophus::SE3f pose;
        float T[12];
        for (int i=0; i<12; ++i)
        {
            iss >> T[i];
        }

        /**
         * It needs to be Sophus::SE3f because if you use SE3d,
         * Sophus throws error saying that the rotation matrix is not orthogonal.
         * SE3d's threshold for orthogonal = (R*R.transpose() - Identity).norm() < 1e-10.
         * But KITTI dataset has 1e-7.
         * SE3f's threshold for orthogonal is 1e-5.
         */
        Sophus::Matrix3f R;
        R(0,0) = T[0]; R(0,1) = T[1]; R(0,2) = T[2];
        R(1,0) = T[4]; R(1,1) = T[5]; R(1,2) = T[6];
        R(2,0) = T[8]; R(2,1) = T[9]; R(2,2) = T[10];
        pose.setRotationMatrix(R);
        
        Sophus::Vector3f t;
        t.x() = T[3];
        t.y() = T[7];
        t.z() = T[11];
        pose.translation() = t;

        ground_truth_poses_.push_back(pose);
    }
}
}