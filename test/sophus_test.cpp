#include <sophus/se3.hpp>

int main(void)
{
    Sophus::SE3d pose;

    Eigen::Quaterniond q;
    q.x() = 0.0;
    q.y() = 0.0;
    q.z() = 0.2588190451025207;
    q.w() = 0.9659258262890684;
    pose.setQuaternion(q);
    pose.translation().x() = 1.0;
    pose.translation().y() = 1.0;
    pose.translation().z() = 0.0;

    Sophus::SE3d inverse_pose = pose.inverse();
    return 0;
}