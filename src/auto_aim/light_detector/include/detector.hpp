#ifndef LIGHT_DETECTOR__DETECTOR_HPP_
#define LIGHT_DETECTOR__DETECTOR_HPP_
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>
#include <iostream>
#include <cstdio>

#include "auto_aim_interfaces/msg/debug_lights.hpp"

namespace rm_auto_aim_dart
{
    const int RED = 0;
    const int GREEN = 1;
    const int BLUE = 2;
    class Detector
    {
    public:
        int binary_threshold = 100; // 灰度255代表白色，0代表黑色
        Detector() = default;
        cv::Mat binary(const cv::Mat &color_image); // convert color image to binary image
        void drawResults(const cv::Mat &image, const cv::Point2f &center, double radius, double fitScore, bool found);
        // Debug msg;
        cv::Mat binary_image, gray_img;
        cv::Mat green_channel;
        auto_aim_interfaces::msg::DebugLights debug_lights;

        // 计算拟合圆的函数
        cv::Point2f best_center;
        double best_radius = 0.0f;
        double best_fit_score = -1.0f; // 最佳拟合度得分（0-100%）
        bool has_best_fit = false;

        struct Light : public cv::RotatedRect
        {
            Light() = default;
            explicit Light(cv::Point2f center, float radius) : center(center), radius(radius) { initializeLight(center, radius); };
            cv::Point2f center;
            float radius;
            double width;
            double height;
            double title_angle;
            cv::Point2f top, bottom;
            cv::Point2f left, right;
            float area;
            float circularity;

        private:
            void initializeLight(const cv::Point2f &center, float radius);
        };
        struct LightParams
        {
            float min_area = 20.0f;       // 最小面积阈值（像素）
            float min_circularity = 0.7f; // 最小圆度
            float min_radius = 10.0f;     // 初始半径下界
            float max_radius = 40.0f;    // 初始半径上界
        };

        // 动态半径滤波所需状态
        float prev_radius_ = 0.0f;
        bool has_prev_radius_ = false;

        std::vector<Light> find_lights(const cv::Mat &color_image, const cv::Mat &binary_image); // find lights in binary image
    };
}

#endif
