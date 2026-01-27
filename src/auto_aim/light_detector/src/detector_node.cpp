// Copyright 2022 Chen Jun
// Licensed under the MIT License.

// ROS
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_ros/create_timer_ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

// STD
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>
#include <limits>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "detector.hpp"
#include "detector_node.hpp"
#include "pnp_solver.hpp"
// 新增：卡尔曼滤波器
#include "kalman_filter.hpp"

namespace rm_auto_aim_dart
{
    LightDetectorNode::LightDetectorNode(const rclcpp::NodeOptions &options) : Node("light_detector", options)
    {
        RCLCPP_INFO(this->get_logger(), "Starting LightDetectorNode!");

        this->get_logger().set_level(rclcpp::Logger::Level::Debug);
        // detector
        detector_ = initDectector();

        // 获取并设置卡尔曼滤波参数
        R_angle_ = this->declare_parameter<double>("angle_filter_R", 1e-2);
        double angle_q_small = this->declare_parameter<double>("angle_filter_Q_small", Q_small_);
        double angle_q_big = this->declare_parameter<double>("angle_filter_Q_big", Q_big_);
        // 小Q用于静止阶段，大Q用于运动阶段
        Q_small_ = angle_q_small;
        Q_big_ = angle_q_big;
        angle_filter_.setParameters(Q_small_, R_angle_);

        // lights publisher
        light_pub_ = this->create_publisher<auto_aim_interfaces::msg::Light>(
            "lights", rclcpp::SensorDataQoS());
        send_pub_ = this->create_publisher<auto_aim_interfaces::msg::Send>(
            "/Send",
            rclcpp::SensorDataQoS());

        // Visualization marker publisher
        // See http://wiki.ros.org/rviz/DisplayTypes/Marker
        light_marker_.ns = "light";
        light_marker_.action = visualization_msgs::msg::Marker::ADD;
        light_marker_.type = visualization_msgs::msg::Marker::CYLINDER;
        light_marker_.scale.x = 2 * light_radius_;
        light_marker_.scale.y = 2 * light_radius_;
        light_marker_.scale.z = 0.01;
        light_marker_.color.a = 1;
        light_marker_.color.r = 0.0f;
        light_marker_.color.g = 1.0f;
        light_marker_.color.b = 0.0f;
        light_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

        marker_pub_ =
            this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "light_markers", 10);

        // Debug Publishers
        debug_ =
            this->declare_parameter<bool>("debug", true); // 先手动改debug_
        if (debug_)
        {
            createDebugPublishers();
            RCLCPP_INFO(this->get_logger(), "Debug mode is enabled");
        }

        // // +++ 新增：声明并加载 dart_angle_offset 参数 +++
        // for (int i = 1; i <= 4; ++i)
        // {
        //     this->declare_parameter<double>(
        //         "dart_angle_offset." + std::to_string(i),
        //         0.0);
        //     dart_offset_map_[i] =
        //         this->get_parameter("dart_angle_offset." + std::to_string(i))
        //             .as_double();
        // }

        // +++ 新增：订阅当前飞镖编号 +++
        dart_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            "current_dart_id",
            rclcpp::SensorDataQoS(),
            [this](const std_msgs::msg::UInt8::SharedPtr msg)
            {
                current_dart_id_ = msg->data;
                RCLCPP_DEBUG(this->get_logger(),
                             "Received current_dart_id: %u", current_dart_id_);
            });

        // 订阅比赛模式
        competition_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            "competition_mode",
            rclcpp::SensorDataQoS(),
            [this](std_msgs::msg::UInt8::SharedPtr msg)
            {
                competition_mode_ = msg->data;
                RCLCPP_INFO(this->get_logger(),
                            "Competition mode updated: %u", competition_mode_);
            });

        // <<< NEW: subscribe to serial offset >>> 将改动offset的部分交给电控，便于短时间内操作
        offset_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "offset", rclcpp::SensorDataQoS(),
            [this](const std_msgs::msg::Float32::SharedPtr msg)
            {
                offset_ = msg->data;
                RCLCPP_DEBUG(this->get_logger(), "Received offset: %f", offset_);
            });

        // Debug param change monitor
        debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
        debug_cb_handle_ =
            debug_param_sub_->add_parameter_callback("debug", [this](const rclcpp::Parameter &p)
                                                     {
          debug_ = p.as_bool();
          debug_ ? createDebugPublishers() : destroyDebugPublishers(); });
        // Camera info
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera_info", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info)
            {
                camera_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
                camera_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
                pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
                camera_info_sub_.reset(); // 取消订阅
            });
        // imageCallback when camera info is ready
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", rclcpp::SensorDataQoS(),
            std::bind(&LightDetectorNode::imageCallback, this, std::placeholders::_1));

        camera_optical_frame_ =
            this->declare_parameter<std::string>("camera_optical_frame", "camera_optical_frame");
        cloud_topic_ =
            this->declare_parameter<std::string>("cloud_topic", "/livox/accum_points");
        fused_send_topic_ =
            this->declare_parameter<std::string>("fused_send_topic", "send_fused");
        draw_cloud_ = this->declare_parameter<bool>("draw_cloud", true);
        cloud_draw_stride_ = this->declare_parameter<int>("cloud_draw_stride", 5);
        if (cloud_draw_stride_ < 1) {
            cloud_draw_stride_ = 1;
        }

        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_, rclcpp::SensorDataQoS(),
            std::bind(&LightDetectorNode::cloudCallback, this, std::placeholders::_1));

        fused_send_sub_ = this->create_subscription<auto_aim_interfaces::msg::Send>(
            fused_send_topic_, rclcpp::SensorDataQoS(),
            std::bind(&LightDetectorNode::fusedSendCallback, this, std::placeholders::_1));

        tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(), this->get_node_timers_interface());
        tf2_buffer_->setCreateTimerInterface(timer_interface);
        tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
    }

    void LightDetectorNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        last_cloud_ = msg;
    }

    void LightDetectorNode::fusedSendCallback(const auto_aim_interfaces::msg::Send::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(fused_mutex_);
        last_fused_send_ = *msg;
        last_fused_stamp_ = this->now();
        has_fused_send_ = true;
    }

    void LightDetectorNode::chooseBestPose(Detector::Light &light, const std::vector<cv::Mat> &rvecs, const std::vector<cv::Mat> &tvecs, cv::Mat &rvec, cv::Mat &tvec)
    {
        // choose the best result
        if (rvecs.empty() || tvecs.empty())
        {
            RCLCPP_WARN(this->get_logger(),
                        "chooseBestPose: 空的 rvecs/tvecs,跳过绘制");
            return;
        }
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvecs[0], rotation_matrix);
        // rotation matrix to quaternion
        Eigen::Matrix3d rotation_matrix_eigen;
        cv::cv2eigen(rotation_matrix, rotation_matrix_eigen);

        Eigen::Quaterniond q_gimbal_camera(
            Eigen::AngleAxisd(-CV_PI / 2, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(-CV_PI / 2, Eigen::Vector3d::UnitX()));
        Eigen::Quaterniond q_rotation(rotation_matrix_eigen);
        q_rotation = q_gimbal_camera * q_rotation;
        // get yaw
        Eigen::Vector3d rpy = q_rotation.toRotationMatrix().eulerAngles(0, 1, 2);
        // 限制在-pi到pi之间
        rpy(0) = std::fmod(rpy(0) + M_PI, M_PI) > M_PI / 2 ? std::fmod(rpy(0) + M_PI, M_PI) - M_PI : std::fmod(rpy(0) + M_PI, M_PI);
        rpy(1) = std::fmod(rpy(1) + M_PI, M_PI) > M_PI / 2 ? std::fmod(rpy(1) + M_PI, M_PI) - M_PI : std::fmod(rpy(1) + M_PI, M_PI);
        rpy(2) = std::fmod(rpy(2) + M_PI, M_PI) > M_PI / 2 ? std::fmod(rpy(2) + M_PI, M_PI) - M_PI : std::fmod(rpy(2) + M_PI, M_PI);

        q_rotation = Eigen::Quaterniond(Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX())) *
                     Eigen::Quaterniond(Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY())) *
                     Eigen::Quaterniond(Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ()));
        q_rotation = q_gimbal_camera.conjugate() * q_rotation;
        Eigen::Matrix3d eigen_mat = q_rotation.toRotationMatrix();

        // BaSolver 后期再加上
        // if (rpy(0) < 0.26)
        // {
        //     Eigen::Vector3d eigen_tvec;
        //     eigen_tvec << tvecs[0].at<double>(0),
        //         tvecs[0].at<double>(1),
        //         tvecs[0].at<double>(2);
        //     eigen_mat = ba_solver_->solveBa(armor, eigen_tvec, eigen_mat, imu_to_camera);
        // }
        cv::Mat rmat;
        cv::eigen2cv(eigen_mat, rmat);
        cv::Rodrigues(rmat, rvec);
        tvec = tvecs[0];
    }

    void LightDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
    {
        // 未到比赛开始，不处理
        if (competition_mode_ != 4)
        {
            RCLCPP_DEBUG(this->get_logger(),
                         "Skipping detection, mode=%u", competition_mode_);
            return;
        }
        RCLCPP_INFO(this->get_logger(),
                    "Received image with header.frame_id='%s' at time %u.%u",
                    img_msg->header.frame_id.c_str(),
                    img_msg->header.stamp.sec, img_msg->header.stamp.nanosec); // 调试用

        if (debug_)
            lights_msg_.image = *img_msg;
        cv::Mat img;
        auto lights = detectLights(img_msg, img);

        try
        {
            rclcpp::Time target_time = img_msg->header.stamp;
            auto odom_to_gimbal = tf2_buffer_->lookupTransform(
                "odom", img_msg->header.frame_id, target_time,
                rclcpp::Duration::from_seconds(0.01));
            RCLCPP_DEBUG(this->get_logger(), "Image frame_id='%s'", img_msg->header.frame_id.c_str()); // 这里的 frame_id 是相机的 frame_id,调试用
            auto msg_q = odom_to_gimbal.transform.rotation;
            tf2::Quaternion tf_q;
            tf2::fromMsg(msg_q, tf_q);
            tf2::Matrix3x3 tf2_matrix = tf2::Matrix3x3(tf_q);
            imu_to_camera << tf2_matrix.getRow(0)[0], tf2_matrix.getRow(0)[1],
                tf2_matrix.getRow(0)[2], tf2_matrix.getRow(1)[0],
                tf2_matrix.getRow(1)[1], tf2_matrix.getRow(1)[2],
                tf2_matrix.getRow(2)[0], tf2_matrix.getRow(2)[1],
                tf2_matrix.getRow(2)[2];
        }
        catch (...)
        {
            RCLCPP_ERROR(this->get_logger(), "Something Wrong when lookUpTransform");
            return;
        }

        // —— 3. 加入空检测 ——
        RCLCPP_INFO(this->get_logger(), "Detected %zu lights", lights.size());
        if (lights.empty())
        {
            RCLCPP_DEBUG_THROTTLE(
                this->get_logger(), *get_clock(), 200,
                "No lights detected, sending zero packet");
            // —— 新增：无灯时持续发 distance=1, angle=0 ——
            auto zero_msg = auto_aim_interfaces::msg::Send();
            zero_msg.header = img_msg->header;
            zero_msg.distance = 1.0;
            zero_msg.angle = 0.0;
            zero_msg.u = 0.0f;
            zero_msg.v = 0.0f;
            zero_msg.roi_radius = 0.0f;
            // 无灯时视为不稳定
            zero_msg.stability = 0;
            send_pub_->publish(zero_msg);
            return;
        }

        if (pnp_solver_ != nullptr)
        {
            lights_msg_.header = light_marker_.header = img_msg->header;
            lights_msg_.lights.clear();
            marker_array_.markers.clear();
            light_marker_.id = 0;
            text_marker_.id = 0;
            std::vector<cv::Point2f> roi_centers;
            std::vector<float> roi_radii;

            auto_aim_interfaces::msg::Light light_msg;
            for (auto &light : lights)
            {
                std::vector<cv::Mat> rvecs, tvecs;
                bool success = pnp_solver_->solvePnP(light, rvecs, tvecs);
                if (success)
                {
                    cv::Mat rvec, tvec;
                    chooseBestPose(light, rvecs, tvecs, rvec, tvec);
                    // Fill pose
                    light_msg.pose.position.x = tvec.at<double>(0);
                    light_msg.pose.position.y = tvec.at<double>(1);
                    light_msg.pose.position.z = tvec.at<double>(2);

                    // Fill the distance to image center
                    light_msg.distance_to_image_center = pnp_solver_->calculateDistanceToCenter(light.center);
                    light_msg.distance = pnp_solver_->getDistance(light, rvec, tvec);
                    light_msg.angle = pnp_solver_->calculateHorizontalAngleDeg(light.center);

                    // Fill the markers
                    light_marker_.id++;
                    light_marker_.pose = light_msg.pose;
                    lights_msg_.lights.emplace_back(light_msg);
                    marker_array_.markers.emplace_back(light_marker_);
                    roi_centers.emplace_back(light.center);
                    roi_radii.emplace_back(light.radius);
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "PnP failed!");
                }
            }
            drawResults(img_msg, img, lights);
            for (size_t i = 0; i < lights_msg_.lights.size(); ++i)
            {
                const auto &light = lights_msg_.lights[i];
                light_pub_->publish(light);
                // —— 新增：针对每个 light 同时发布 Send 消息 —— :contentReference[oaicite:2]{index=2}:contentReference[oaicite:3]{index=3}
                auto send_msg = auto_aim_interfaces::msg::Send();
                // 保留原始图像的 header，用于串口端计算延迟
                send_msg.header = lights_msg_.header;
                // 原始测量值
                double raw_angle = light.angle;
                double raw_dist = light.distance;

                // 1. 判断是否“快速运动”——如果跳变大于阈值
                if (std::abs(raw_angle - prev_angle_) > jump_threshold_)
                {
                    // 切换到大Q，几乎无平滑
                    angle_filter_.setParameters(Q_big_, R_angle_);
                }
                else
                {
                    // 静止阶段，小Q强化平滑
                    angle_filter_.setParameters(Q_small_, R_angle_);
                }

                // 2. 卡尔曼滤波更新
                double smooth_angle = angle_filter_.update(raw_angle);

                // // 3. 发布平滑后的结果

                // <<< UPDATED: use serial offset >>>
                send_msg.distance = raw_dist;
                send_msg.angle = smooth_angle + offset_;
                if (i < roi_centers.size() && i < roi_radii.size())
                {
                    send_msg.u = roi_centers[i].x;
                    send_msg.v = roi_centers[i].y;
                    send_msg.roi_radius = roi_radii[i];
                }
                else
                {
                    send_msg.u = 0.0f;
                    send_msg.v = 0.0f;
                    send_msg.roi_radius = 0.0f;
                }
                send_msg.stability = (std::abs(smooth_angle + offset_) <= 0.06) ? 1 : 0;
                send_pub_->publish(send_msg);

                prev_angle_ = raw_angle;
            }
            publishMarkers();
        }
    }
    std::unique_ptr<Detector> LightDetectorNode::initDectector()
    {
        auto detector = std::make_unique<Detector>();
        detector->binary_threshold = this->declare_parameter<int>("binary_threshold", 100);
        return detector;
    }
    std::vector<Detector::Light> LightDetectorNode::detectLights(
        const sensor_msgs::msg::Image::ConstSharedPtr &img_msg,
        cv::Mat &img)
    {
        // 1. 把 ROS 图像转为 cv::Mat
        img = cv_bridge::toCvShare(img_msg, "bgr8")->image;

        // 2. 先生成二值图，供调试与后续使用
        cv::Mat binary = detector_->binary(img);

        // 3. 捕获可能在内部越界的异常，确保安全返回
        std::vector<Detector::Light> lights;
        try
        {
            lights = detector_->find_lights(img, binary);
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(),
                        "find_lights exception: %s", e.what());
            return {}; // 发现异常，返回空列表
        }

        // 4. 计算并输出延迟（仅做调试）
        auto final_time = this->now();
        auto latency = (final_time - img_msg->header.stamp).seconds() * 1000;
        RCLCPP_DEBUG_STREAM(this->get_logger(),
                            "Latency: " << latency << "ms");

        // 5. 如果开启 debug 模式，发布二值图与调试 数据
        if (debug_)
        {
            binary_img_pub_.publish(
                cv_bridge::CvImage(img_msg->header, "mono8", binary).toImageMsg());
            lights_data_pub_->publish(detector_->debug_lights);
        }

        // 6. 如果 find_lights 没抛异常，但返回空列表，也直接返回
        if (lights.empty())
        {
            RCLCPP_DEBUG(this->get_logger(),
                         "detectLights: no lights found");
            return {};
        }

        // 7. 返回非空的 lights 列表
        return lights;
    }

    void LightDetectorNode::drawResults(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg, cv::Mat &img, const std::vector<rm_auto_aim_dart::Detector::Light> &lights)
    {
        // 计算延迟
        auto final_time = this->now();
        auto latency = (final_time - img_msg->header.stamp).seconds() * 1000;
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Latency: " << latency << "ms");
        if (!debug_)
        {
            return;
        }
        if (draw_cloud_)
        {
            drawPointCloudOnImage(img_msg, img);
        }
        // 使用 detector_ 中存储的最佳拟合结果，而不是 lights[0]
        if (detector_->has_best_fit)
        {
            detector_->drawResults(img, detector_->best_center, detector_->best_radius,
                                   detector_->best_fit_score, true);
        }
        else if (!lights.empty())
        {
            // 备选方案：如果没有存储最佳拟合，但有灯光，则使用 lights[0]
            detector_->drawResults(img, lights[0].center, lights[0].radius, 100, true);
        }
        else
        {
            // 没有找到灯光
            detector_->drawResults(img, cv::Point2f(0, 0), 0, 0, false);
        }
        // Draw camera center
        cv::circle(img, camera_center_, 5, cv::Scalar(255, 0, 0), 2);
        // Draw latency
        std::stringstream latency_ss;
        latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
        auto latency_s = latency_ss.str();
        cv::putText(
            img, latency_s, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
        // 新增：如果已经计算出了 PnP 的 distance/angle，就把它们也画上去
        if (!lights_msg_.lights.empty())
        {
            // 只取第一个 light 的信息
            const auto &lm = lights_msg_.lights[0];
            double draw_distance = lm.distance;
            double draw_angle = lm.angle;
            bool using_lidar = false;
            {
                std::lock_guard<std::mutex> lock(fused_mutex_);
                if (has_fused_send_)
                {
                    draw_distance = last_fused_send_.distance;
                    draw_angle = last_fused_send_.angle;
                    using_lidar = true;
                }
            }
            char info[128];
            // 距离单位是米，角度单位是度
            std::snprintf(info, sizeof(info),
                          "Dist=%.2fm, Ang=%.2fdeg%s",
                          draw_distance, draw_angle,
                          using_lidar ? " [lidar]" : "");
            cv::putText(img, info, cv::Point(10, 60),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(0, 255, 255), 2);
        }
        result_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
    }

    void LightDetectorNode::drawPointCloudOnImage(
        const sensor_msgs::msg::Image::ConstSharedPtr &img_msg,
        cv::Mat &img)
    {
        if (!camera_info_) {
            return;
        }

        sensor_msgs::msg::PointCloud2::SharedPtr cloud;
        {
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            cloud = last_cloud_;
        }
        if (!cloud) {
            return;
        }

        rclcpp::Time lookup_time = rclcpp::Time(img_msg->header.stamp);
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf2_buffer_->lookupTransform(
                camera_optical_frame_, cloud->header.frame_id, lookup_time,
                rclcpp::Duration::from_seconds(0.05));
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Cloud TF lookup failed: %s", ex.what());
            return;
        }

        sensor_msgs::msg::PointCloud2 cloud_tf;
        try {
            tf2::doTransform(*cloud, cloud_tf, transform);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Cloud transform failed: %s", ex.what());
            return;
        }

        double fx = camera_info_->k[0];
        double fy = camera_info_->k[4];
        double cx = camera_info_->k[2];
        double cy = camera_info_->k[5];
        if (!(fx > 0.0 && fy > 0.0)) {
            return;
        }

        int stride = cloud_draw_stride_ < 1 ? 1 : cloud_draw_stride_;
        size_t idx = 0;
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_tf, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_tf, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_tf, "z");
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++idx) {
            if (idx % static_cast<size_t>(stride) != 0) {
                continue;
            }
            float x = *iter_x;
            float y = *iter_y;
            float z = *iter_z;
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                continue;
            }
            if (z <= 0.0f) {
                continue;
            }
            double u = fx * static_cast<double>(x) / static_cast<double>(z) + cx;
            double v = fy * static_cast<double>(y) / static_cast<double>(z) + cy;
            if (u < 0.0 || v < 0.0 || u >= img.cols || v >= img.rows) {
                continue;
            }
            cv::circle(img, cv::Point(static_cast<int>(u), static_cast<int>(v)),
                       1, cv::Scalar(0, 255, 0), -1);
        }
    }

    void LightDetectorNode::createDebugPublishers()
    {
        RCLCPP_INFO(this->get_logger(), "Debug mode enabled, creating debug publishers");
        lights_data_pub_ =
            this->create_publisher<auto_aim_interfaces::msg::DebugLights>("/detector/debug_lights", 10);

        binary_img_pub_ = image_transport::create_publisher(this, "/detector/binary_img");
        result_img_pub_ = image_transport::create_publisher(this, "/detector/result_img");
    }

    void LightDetectorNode::destroyDebugPublishers()
    {
        lights_data_pub_.reset();

        binary_img_pub_.shutdown();
        result_img_pub_.shutdown();
    }

    void LightDetectorNode::publishMarkers()
    {
        using Marker = visualization_msgs::msg::Marker;
        light_marker_.action = lights_msg_.lights.empty() ? Marker::DELETE : Marker::ADD;
        marker_array_.markers.emplace_back(light_marker_);
        marker_pub_->publish(marker_array_);
    }
} // namespace rm_auto_aim_dart

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim_dart::LightDetectorNode)
