#include "pnp_solver.hpp"
#include "detector.hpp"
#include <vector>
#include <rclcpp/logging.hpp>
namespace rm_auto_aim_dart
{
    PnPSolver::PnPSolver(const std::array<double, 9> &camera_matrix, const std::vector<double> &dist_coeffs)
        : camera_matrix(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()),
          distortion_coefficients(cv::Mat(1, 5, CV_64F, const_cast<double *>(dist_coeffs.data())).clone())
    {
        constexpr double radius = CIRCLE_RADIUS / 1000; // convert to meters

        // start from the top of the circle and go clockwise
        // Model coordinate system is x-forward, y-left, z-up
        circle_points.emplace_back(cv::Point3f(0, 0, 0));
        circle_points.emplace_back(cv::Point3f(0, 0, radius));
        circle_points.emplace_back(cv::Point3f(0, radius, 0));
        circle_points.emplace_back(cv::Point3f(0, 0, -radius));
        circle_points.emplace_back(cv::Point3f(0, -radius, 0));
    }

    bool PnPSolver::solvePnP(const Detector::Light &light,
                             std::vector<cv::Mat> &rvecs,
                             std::vector<cv::Mat> &tvecs)
    {
        // 准备图像点：中心及四个边界点
        std::vector<cv::Point2f> image_points{
            light.center,
            light.top,
            light.right,
            light.bottom,
            light.left};

        try
        {
            // 使用 solvePnPGeneric 输出多个解，保证返回的 Mat 为 FIXED_TYPE
            bool ok = cv::solvePnPGeneric(
                circle_points,           // 三维模型点
                image_points,            // 二维像素点
                camera_matrix,           // 相机内参
                distortion_coefficients, // 畸变系数
                rvecs,                   // 输出：旋转向量列表
                tvecs,                   // 输出：平移向量列表
                false,                   // 不使用初始猜测
                cv::SOLVEPNP_ITERATIVE,  // 算法类型
                cv::noArray()            // 可选：输出重投影误差
            );
            if (!ok || rvecs.empty() || tvecs.empty())
            {
                RCLCPP_WARN(rclcpp::get_logger("pnp_solver"),
                            "PnP 求解失败或未返回合法向量");
                return false;
            }
            return true;
        }
        catch (const cv::Exception &e)
        {
            RCLCPP_WARN(rclcpp::get_logger("pnp_solver"),
                        "cv::solvePnPGeneric 异常: %s", e.what());
            return false;
        }
    }

    float PnPSolver::calculateDistanceToCenter(const cv::Point2f &center)
    {
        float cx = camera_matrix.at<double>(0, 2);
        float cy = camera_matrix.at<double>(1, 2);
        return cv::norm(center - cv::Point2f(cx, cy));
    }

    double PnPSolver::getDistance(const Detector::Light &light, cv::Mat &rvec, cv::Mat &tvec)
    {
        std::vector<cv::Point2f> image_points;
        image_points.emplace_back(light.center);
        image_points.emplace_back(light.top);
        image_points.emplace_back(light.right);
        image_points.emplace_back(light.bottom);
        image_points.emplace_back(light.left);
        if (cv::solvePnP(
                circle_points, image_points, camera_matrix, distortion_coefficients, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE))
        {
            std::vector<double> tvecVec;
            tvec.copyTo(tvecVec);
            double distance = std::sqrt(tvecVec[0] * tvecVec[0] +
                                        tvecVec[1] * tvecVec[1] +
                                        tvecVec[2] * tvecVec[2]);
            std::cout << "distance from the camera to the object: " << distance << std::endl;
            return distance;
        }
        else
        {
            std::cerr << "solvePnP failed to find a solution." << std::endl;
            return -1;
        }
    }

    double PnPSolver::getAngle(const Detector::Light &light, cv::Mat &rvec, cv::Mat &tvec)
    {
        std::vector<cv::Point2f> image_points;
        image_points.emplace_back(light.center);
        image_points.emplace_back(light.top);
        image_points.emplace_back(light.right);
        image_points.emplace_back(light.bottom);
        image_points.emplace_back(light.left);
        if (cv::solvePnP(
                circle_points, image_points, camera_matrix, distortion_coefficients, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE))
        {
            std::vector<double> tvecVec;
            tvec.copyTo(tvecVec);
            double angle = atan2(tvecVec[0], tvecVec[1]);
            std::cout << "angle from the to the object: " << angle << std::endl;
            return angle;
        }
        else
        {
            std::cerr << "solvePnP failed to find a solution." << std::endl;
            return -1;
        }
    }

    float PnPSolver::calculateHorizontalAngleDeg(const cv::Point2f &center) const
    {
        double cx = camera_matrix.at<double>(0, 2);
        double fx = camera_matrix.at<double>(0, 0);
        double dx = static_cast<double>(center.x) - cx;
        double rad = std::atan2(dx, fx);
        return rad * 180.0 / M_PI; // 直接返回度
    }
} // namespace rm_auto_aim_dart