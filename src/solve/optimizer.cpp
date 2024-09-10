#include <iostream>

#include <ceres/numeric_diff_cost_function.h>
#include <ceres/problem.h>
#include <ceres/solver.h>

#include <sophus/ceres_manifold.hpp>

#include "visual_odometry/solve/optimizer.hpp"

namespace VO
{
void Optimizer::optimize(const std::shared_ptr<Map>& map, const cv::Matx33d& intrinsics)
{
    ceres::Problem problem;
    ceres::Manifold* se3Parametrization = new Sophus::Manifold<Sophus::SE3>();
    
    for (const std::shared_ptr<MapPoint>& landmark : map->landmarks_)
    {
        problem.AddParameterBlock(landmark->pose_.data(), 3);
    }

    for (std::size_t i=0; i<map->frames_.size(); ++i)
    {
        std::shared_ptr<Frame>& frame = map->frames_[i];
        problem.AddParameterBlock(frame->pose_.data(), Sophus::SE3d::num_parameters, se3Parametrization);
        for (const std::shared_ptr<Feature>& feature : frame->features_left_)
        {
            ReprojectionError* constraint = new ReprojectionError(
                static_cast<double>(feature->pixel.x),
                static_cast<double>(feature->pixel.y),
                intrinsics);
            ceres::CostFunction* cost_function = 
                new ceres::NumericDiffCostFunction<
                    ReprojectionError, ceres::CENTRAL, 2, Sophus::SE3d::num_parameters, 3>(
                        constraint);
            problem.AddResidualBlock(cost_function,
                nullptr,
                frame->pose_.data(),
                feature->landmark_->pose_.data());
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;
    options.num_threads = 6;
    options.max_num_iterations = 300;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;
}
}