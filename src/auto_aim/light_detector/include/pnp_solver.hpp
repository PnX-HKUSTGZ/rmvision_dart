#ifndef PNP_SOLVER_HPP
#define PNP_SOLVER_HPP
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>
#include <iostream>
#include <Eigen/Core>
#include <array>
#include "detector.hpp"
namespace rm_auto_aim_dart
{
    class PnPSolver
    {
    public:
        PnPSolver(
            const std::array<double, 9> &camera_matrix,
            const std::vector<double> &distortion_coefficients);
        bool solvePnP(const Detector::Light &light, std::vector<cv::Mat> &rvec, std::vector<cv::Mat> &tvec);
        float calculateDistanceToCenter(const cv::Point2f &center);
        double getDistance(const Detector::Light &light, cv::Mat &rvec, cv::Mat &tvec);
        double getAngle(const Detector::Light &light, cv::Mat &rvec, cv::Mat &tvec);
        /** 返回相对于光心的水平夹角，单位：度 */
        float calculateHorizontalAngleDeg(const cv::Point2f &center) const;
        double distance;
        float angle;

    private:
        cv::Mat camera_matrix;
        cv::Mat distortion_coefficients;
        static constexpr float CIRCLE_RADIUS = 150;
        std::vector<cv::Point3f> circle_points;
    };
}

#endif