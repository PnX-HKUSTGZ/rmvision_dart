#include "../include/detector.hpp"
#include <rclcpp/logging.hpp>

namespace rm_auto_aim_dart
{
    void Detector::setRadiusRange(float min_radius, float max_radius)
    {
        std::lock_guard<std::mutex> lock(params_mutex_);
        light_params_.min_radius = min_radius;
        light_params_.max_radius = max_radius;
    }

    cv::Mat Detector::binary(const cv::Mat &color_image)
    {
        if (color_image.type() != CV_8UC3 && color_image.type() != CV_8UC4)
        {
            throw std::invalid_argument("Input image must be 3 or 4 channel image, either CV_8UC3 or CV_8UC4.");
        }
        cv::Mat bgr_image;
        if (color_image.type() == CV_8UC4)
        {
            cv::cvtColor(color_image, bgr_image, cv::COLOR_BGRA2BGR);
        }
        else
        {
            bgr_image = color_image;
        }

        // 提取绿色通道，OpenCV 中为 BGR 顺序。
        std::vector<cv::Mat> channels;
        cv::split(bgr_image, channels);
        this->green_channel = channels[1];
        cv::Mat red_channel = channels[2];
        cv::Mat blue_channel = channels[0];

        cv::Mat hsv_image;
        cv::cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);
        cv::Mat hsv_green;
        cv::inRange(hsv_image, cv::Scalar(35, 45, binary_threshold),
                    cv::Scalar(95, 255, 255), hsv_green);

        cv::Mat excess_green;
        cv::addWeighted(green_channel, 2.0, red_channel, -1.0, 0.0, excess_green, CV_16S);
        cv::addWeighted(excess_green, 1.0, blue_channel, -1.0, 0.0, excess_green, CV_16S);
        cv::Mat green_dominant = excess_green > 25;
        cv::bitwise_and(green_dominant, green_channel > binary_threshold, green_dominant);

        cv::Mat green_mask;
        cv::bitwise_or(hsv_green, green_dominant, green_mask);

        cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(green_mask, green_mask, cv::MORPH_OPEN, element);
        cv::morphologyEx(green_mask, green_mask, cv::MORPH_CLOSE, element);

        Detector::LightParams params;
        {
            std::lock_guard<std::mutex> lock(params_mutex_);
            params = light_params_;
        }

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        cv::Mat circle_mask = cv::Mat::zeros(green_mask.size(), CV_8UC1);
        for (const auto &contour : contours)
        {
            if (contour.size() < 5)
            {
                continue;
            }

            double area = cv::contourArea(contour);
            if (area < params.min_area)
            {
                continue;
            }

            double perimeter = cv::arcLength(contour, true);
            if (perimeter <= 0.0)
            {
                continue;
            }

            cv::Point2f center;
            float radius = 0.0f;
            cv::minEnclosingCircle(contour, center, radius);
            if (radius < params.min_radius || radius > params.max_radius)
            {
                continue;
            }

            double circularity = 4.0 * CV_PI * area / (perimeter * perimeter);
            if (circularity < 0.65)
            {
                continue;
            }

            cv::Rect rect = cv::boundingRect(contour);
            double aspect = rect.height > 0 ? static_cast<double>(rect.width) / rect.height : 0.0;
            if (aspect < 0.65 || aspect > 1.55)
            {
                continue;
            }

            double fill_ratio = area / (CV_PI * radius * radius);
            if (fill_ratio < 0.45 || fill_ratio > 1.20)
            {
                continue;
            }

            cv::drawContours(circle_mask, std::vector<std::vector<cv::Point>>{contour},
                             -1, cv::Scalar(255), cv::FILLED);
        }

        return circle_mask;
    }

    void Detector::Light::initializeLight(const cv::Point2f &center, float radius)
    {
        top = cv::Point2f(center.x, center.y - radius); // y坐标向下为正
        bottom = cv::Point2f(center.x, center.y + radius);
        left = cv::Point2f(center.x - radius, center.y);
        right = cv::Point2f(center.x + radius, center.y);
        width = 2 * radius;
        height = 2 * radius;
        title_angle = 0;
        std::cout << "top: " << top << std::endl;       // 调试
        std::cout << "bottom: " << bottom << std::endl; // 调试
        std::cout << "left: " << left << std::endl;     // 调试
        std::cout << "right: " << right << std::endl;   // 调试
    }

    void Detector::drawResults(const cv::Mat &image, const cv::Point2f &center, double radius, double fitScore, bool found)
    {
        if (found)
        {
            // 在原图上绘制拟合度最高的圆形轮廓及其圆心
            circle(image, center, static_cast<int>(round(radius)), cv::Scalar(0, 0, 255), 2); // 红色圆边框
            circle(image, center, 3, cv::Scalar(255, 0, 0), -1);                              // 蓝色小圆点标示圆心

            // 在图像上绘制文字注释（显示半径和拟合度百分比）
            char label[100];
            snprintf(label, sizeof(label), "R=%.1f, Fit=%.1f%%", radius, fitScore);
            cv::Point textPos(static_cast<int>(center.x - 50), static_cast<int>(center.y - 10));
            putText(image, label, textPos, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
        }
        else
        {
            std::cout << "没有找到拟合度达标的圆形对象。" << std::endl;
        }
    }

    bool isLight(const Detector::Light &light, const Detector::LightParams &params, float circularity, float area)
    {
        if (light.radius < params.min_radius || light.radius > params.max_radius)
            return false;
        if (area < params.min_area)
            return false;
        if (circularity < params.min_circularity)
            return false;
        return true;
    }

    bool fitCircleLeastSquares(const std::vector<cv::Point> &contourPoints, cv::Point2f &center, float &radius)
    {
        int N = contourPoints.size();
        if (N < 3)
        {
            return false;
        }
        // 计算各类求和量（使用 double 提高精度）
        double sumX = 0, sumY = 0;
        double sumX2 = 0, sumY2 = 0, sumXY = 0;
        double sumX2Y2 = 0;   // sum(x_i^2 + y_i^2)
        double sumX_X2Y2 = 0; // sum(x_i * (x_i^2 + y_i^2))
        double sumY_X2Y2 = 0; // sum(y_i * (x_i^2 + y_i^2))

        for (size_t i = 0; i < contourPoints.size(); ++i)
        {
            double x = static_cast<double>(contourPoints[i].x);
            double y = static_cast<double>(contourPoints[i].y);
            double x2 = x * x;
            double y2 = y * y;
            sumX += x;
            sumY += y;
            sumX2 += x2;
            sumY2 += y2;
            sumXY += x * y;
            double x2y2 = x2 + y2;
            sumX2Y2 += x2y2;
            sumX_X2Y2 += x * x2y2;
            sumY_X2Y2 += y * x2y2;
        }

        // 构建正规方程 (Normal Equation) 的矩阵 M 和向量 Y
        // 方程形式: M * [A, B, C]^T = Y
        // M = | sumX2   sumXY   sumX  |
        //     | sumXY   sumY2   sumY  |
        //     | sumX    sumY    N     |
        // Y = | sumX * (x^2+y^2) |
        //     | sumY * (x^2+y^2) |
        //     | sum(x^2+y^2)     |
        // 其中 A = 2a, B = 2b, C = R^2 - a^2 - b^2 （a,b为圆心坐标, R为半径）
        cv::Mat M = (cv::Mat_<double>(3, 3) << sumX2, sumXY, sumX,
                     sumXY, sumY2, sumY,
                     sumX, sumY, N);
        cv::Mat Y = (cv::Mat_<double>(3, 1) << sumX_X2Y2,
                     sumY_X2Y2,
                     sumX2Y2);
        cv::Mat solution;
        bool solved = solve(M, Y, solution, cv::DECOMP_SVD);
        if (!solved)
        {
            return false; // 解方程失败（可能点全部共线等情况）
        }

        // 提取解并计算圆参数
        double A = solution.at<double>(0, 0);
        double B = solution.at<double>(1, 0);
        double C = solution.at<double>(2, 0);
        double a = A / 2.0;
        double b = B / 2.0;
        double R_squared = a * a + b * b + C;
        if (R_squared < 0)
        {
            return false; // 半径平方为负，说明拟合不出有效圆
        }
        double R = sqrt(R_squared);
        center = cv::Point2f(static_cast<float>(a), static_cast<float>(b));
        radius = static_cast<float>(R);
        return true;
    }

    std::vector<Detector::Light> Detector::find_lights(const cv::Mat &color_image, const cv::Mat &binary_image)
    {
        CV_Assert(binary_image.type() == CV_8UC1);
        std::vector<std::vector<cv::Point>> contours;
        cv::Mat hierarchy;
        cv::findContours(binary_image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        std::cout << "contours size: " << contours.size() << std::endl; // 调试
        if (contours.empty())
        {
            // 如果你想打印调试信息：
            std::cout << "[find_lights] no contours found, returning empty\n";

            // 清空调试数据
            this->debug_lights.data.clear();
            return {};
        }
        this->debug_lights.data.clear();

        // 全局最佳拟合记录
        int bestIndex = -1;
        float bestFit = -1.0f;
        cv::Point2f bestC;
        float bestR = 0.0f;
        float bestArea = 0.0f;
        float bestCircularity = 0.0f;
        Detector::LightParams params;
        {
            std::lock_guard<std::mutex> lock(params_mutex_);
            params = light_params_;
        }

        // 查找拟合度最高的圆
        for (size_t i = 0; i < contours.size(); ++i)
        {
            const auto &c = contours[i];
            if (c.size() < 5)
                continue; // 拟合至少需5点

            const float area = static_cast<float>(cv::contourArea(c));
            const double perimeter = cv::arcLength(c, true);
            if (perimeter <= 0.0)
                continue;
            const float circularity = static_cast<float>(4.0 * CV_PI * area / (perimeter * perimeter));

            cv::Point2f center;
            float radius;
            if (!fitCircleLeastSquares(c, center, radius))
                continue; // 拟合失败

            Detector::Light tmp(center, radius);
            tmp.area = area;
            tmp.circularity = circularity;
            if (!isLight(tmp, params, tmp.circularity, tmp.area))
                continue;

            // 创建灯内部掩码
            cv::Mat maskIn = cv::Mat::zeros(this->green_channel.size(), CV_8UC1);
            cv::circle(maskIn, tmp.center, static_cast<int>(radius), cv::Scalar(255), -1); // 填充圆内部为255

            // 创建环形外部掩码（比如圆外1.5倍半径的环形区域）
            cv::Mat maskOut = cv::Mat::zeros(this->green_channel.size(), CV_8UC1);
            cv::circle(maskOut, tmp.center, static_cast<int>(radius * 1.5), cv::Scalar(255), -1);
            cv::circle(maskOut, tmp.center, static_cast<int>(radius), cv::Scalar(0), -1); // 挖空内部

            // 计算内部和外部区域的平均亮度
            float meanIn = cv::mean(this->green_channel, maskIn)[0];
            float meanOut = cv::mean(this->green_channel, maskOut)[0];
            if (meanIn < 100.0f || meanIn < meanOut * 1.5f)
            {
                continue; // 灯光亮度不足
            }

            // 计算平均相对误差
            double sumErr = 0;
            for (const auto &p : c)
            {
                sumErr += std::abs(cv::norm(cv::Point2f(p) - center) - radius);
            }
            double meanErr = sumErr / c.size();
            double fitPct = (1.0 - meanErr / radius) * 100.0;
            fitPct = std::clamp(fitPct, 0.0, 100.0);
            float fitScore = static_cast<float>(fitPct);

            if (fitScore > bestFit)
            {
                bestFit = fitScore;
                bestIndex = static_cast<int>(i);
                bestC = center;
                bestR = radius;
                bestArea = tmp.area;
                bestCircularity = tmp.circularity;
            }
        }

        // 无有效圆，返回空
        if (bestIndex < 0)
        {
            this->debug_lights.data.clear();
            return {};
        }

        // 构造 Light 对象并筛选
        Detector::Light light(bestC, bestR);
        std::vector<Detector::Light> lights;
        if (isLight(light, params, bestCircularity, bestArea))
        {
            // 计算拟合度
            double sumErr = 0;
            for (const auto &p : contours[bestIndex])
            {
                sumErr += std::abs(cv::norm(cv::Point2f(p) - light.center) - light.radius);
            }
            double meanErr = sumErr / contours[bestIndex].size();
            double fitPct = (1.0 - meanErr / light.radius) * 100.0;
            fitPct = std::clamp(fitPct, 0.0, 100.0);
            light.circularity = static_cast<float>(fitPct);

            // 将符合条件的灯光添加到结果列表
            {
                lights.emplace_back(light);
            }

            // 填充调试数据
            this->debug_lights.data.clear();
            if (!lights.empty())
            {
                auto_aim_interfaces::msg::DebugLight dl;
                dl.center_x = lights[0].center.x;
                dl.radius = lights[0].radius;
                dl.is_light = true;
                this->debug_lights.data.emplace_back(dl);
            }
        }
        return lights;
    }

} // namespace rm_auto_aim_dart
