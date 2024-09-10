#ifndef PARSE_KITTI_
#define PARSE_KITTI_

#include <string>
#include <vector>

#include <sophus/se3.hpp>

#include "visual_odometry/sensor/camera.hpp"

namespace VO
{
struct StereoPair
{
    std::string left_frame_;
    std::string right_frame_;
    double time_;
};

class ParseKITTI
{
public:
    ParseKITTI();
    void loadFrames(const std::string& frame_filename);
    void loadCameras(const std::string& calib_filename);
    void loadGroundTruth(const std::string& ground_truth_filename);
    std::vector<StereoPair> frames_;
    std::vector<Camera> cameras_;
    std::vector<Sophus::SE3f> ground_truth_poses_;

private:
    std::string getFrame(const std::string& filename, int frame_idx);
};
}

#endif