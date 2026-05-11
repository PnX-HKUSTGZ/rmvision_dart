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
#include <rcl_interfaces/msg/set_parameters_result.hpp>
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
    namespace
    {
        void normalizeRadiusRange(double &min_radius, double &max_radius,
                                  const rclcpp::Logger &logger, const char *label)
        {
            if (min_radius <= max_radius)
            {
                return;
            }

            std::swap(min_radius, max_radius);
            RCLCPP_WARN(logger,
                        "%s min_radius > max_radius, swapping to %.2f ~ %.2f",
                        label, min_radius, max_radius);
        }
    } // namespace

    LightDetectorNode::LightDetectorNode(const rclcpp::NodeOptions &options) : Node("light_detector", options)
    {
        RCLCPP_INFO(this->get_logger(), "Starting LightDetectorNode!");

        this->get_logger().set_level(rclcpp::Logger::Level::Debug);
        // detector
        detector_ = initDectector();

        use_target_id_ = this->declare_parameter<bool>("use_target_id", true);
        manual_min_radius_ = this->declare_parameter<double>("manual_min_radius", 20.0);
        manual_max_radius_ = this->declare_parameter<double>("manual_max_radius", 50.0);
        target_id_0_min_radius_ =
            this->declare_parameter<double>("target_id_0_min_radius", 50.0);
        target_id_0_max_radius_ =
            this->declare_parameter<double>("target_id_0_max_radius", 80.0);
        target_id_1_min_radius_ =
            this->declare_parameter<double>("target_id_1_min_radius", 10.0);
        target_id_1_max_radius_ =
            this->declare_parameter<double>("target_id_1_max_radius", 50.0);
        pnp_circle_radius_mm_ =
            this->declare_parameter<double>("pnp_circle_radius_mm", 30.0);
        if (pnp_circle_radius_mm_ <= 0.0)
        {
            RCLCPP_WARN(this->get_logger(),
                        "pnp_circle_radius_mm must be positive, fallback to 30.0mm");
            pnp_circle_radius_mm_ = 30.0;
        }
        light_radius_ = pnp_circle_radius_mm_ / 1000.0;
        normalizeRadiusRange(
            manual_min_radius_, manual_max_radius_, this->get_logger(), "manual");
        normalizeRadiusRange(
            target_id_0_min_radius_, target_id_0_max_radius_, this->get_logger(), "target_id_0");
        normalizeRadiusRange(
            target_id_1_min_radius_, target_id_1_max_radius_, this->get_logger(), "target_id_1");
        applyManualRadius();

        // 获取并设置卡尔曼滤波参数
        R_angle_ = this->declare_parameter<double>("angle_filter_R", 1e-2);
        double angle_q_small = this->declare_parameter<double>("angle_filter_Q_small", Q_small_);
        double angle_q_big = this->declare_parameter<double>("angle_filter_Q_big", Q_big_);
        // 小Q用于静止阶段，大Q用于运动阶段
        Q_small_ = angle_q_small;
        Q_big_ = angle_q_big;
        angle_filter_.setParameters(Q_small_, R_angle_);

        dart_input_mode_ = this->declare_parameter<std::string>("dart_input_mode", "serial");
        serial_dart_topic_ =
            this->declare_parameter<std::string>("serial_dart_topic", "current_dart_id");
        serial_offset_topic_ =
            this->declare_parameter<std::string>("serial_offset_topic", "offset");
        barcode_profile_topic_ =
            this->declare_parameter<std::string>("barcode_profile_topic", "barcode/scan_profile");
        total_latency_topic_ =
            this->declare_parameter<std::string>("total_latency_topic", "/latency");
        barcode_slot_count_ = this->declare_parameter<int>("barcode_slot_count", 4);
        barcode_require_full_slots_ =
            this->declare_parameter<bool>("barcode_require_full_slots", true);
        if (barcode_slot_count_ < 1)
        {
            RCLCPP_WARN(this->get_logger(),
                        "barcode_slot_count must be >= 1, fallback to 1");
            barcode_slot_count_ = 1;
        }
        barcode_slots_.assign(static_cast<size_t>(barcode_slot_count_), BarcodeSlot{});
        next_scan_slot_ = 1;
        active_scan_slot_ = 1;
        has_active_barcode_slot_ = false;
        if (dart_input_mode_ != "serial" && dart_input_mode_ != "barcode")
        {
            RCLCPP_WARN(this->get_logger(),
                        "Invalid dart_input_mode='%s', fallback to 'serial'",
                        dart_input_mode_.c_str());
            dart_input_mode_ = "serial";
        }

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

        // 串口发次索引（1~4循环）
        dart_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            serial_dart_topic_,
            rclcpp::SensorDataQoS(),
            [this](const std_msgs::msg::UInt8::SharedPtr msg)
            {
                serial_shot_index_ = msg->data;
                RCLCPP_DEBUG(this->get_logger(),
                             "Received serial shot index: %u", serial_shot_index_);
                if (isBarcodeMode())
                {
                    updateActiveBarcodeProfile();
                }
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

        // 订阅目标 ID，用于动态设置半径阈值
        target_id_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            "target_id",
            rclcpp::SensorDataQoS(),
            [this](const std_msgs::msg::UInt8::SharedPtr msg)
            {
                target_id_ = msg->data;
                RCLCPP_INFO(this->get_logger(),
                            "Received target_id: %u", target_id_);
                if (!use_target_id_)
                {
                    RCLCPP_DEBUG(this->get_logger(),
                                 "target_id ignored (use_target_id=false)");
                    return;
                }
                applyTargetIdRadius();
            });

        // 串口 offset（serial 模式生效）
        offset_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            serial_offset_topic_, rclcpp::SensorDataQoS(),
            [this](const std_msgs::msg::Float32::SharedPtr msg)
            {
                serial_offset_deg_ = static_cast<double>(msg->data);
                RCLCPP_DEBUG(this->get_logger(), "Received serial offset: %.3f", serial_offset_deg_);
            });

        barcode_profile_sub_ = this->create_subscription<auto_aim_interfaces::msg::DartProfile>(
            barcode_profile_topic_, rclcpp::SensorDataQoS(),
            std::bind(&LightDetectorNode::barcodeProfileCallback, this, std::placeholders::_1));

        total_latency_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            total_latency_topic_, rclcpp::SensorDataQoS(),
            [this](const std_msgs::msg::Float64::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lock(total_latency_mutex_);
                total_latency_ms_ = msg->data;
                has_total_latency_ = true;
            });

        on_set_params_cb_handle_ =
            this->add_on_set_parameters_callback(
                [this](const std::vector<rclcpp::Parameter> &params)
                {
                    for (const auto &param : params)
                    {
                        const auto &name = param.get_name();
                        if (name == "use_target_id")
                        {
                            use_target_id_ = param.as_bool();
                        }
                        else if (name == "manual_min_radius")
                        {
                            manual_min_radius_ = param.as_double();
                        }
                        else if (name == "manual_max_radius")
                        {
                            manual_max_radius_ = param.as_double();
                        }
                        else if (name == "target_id_0_min_radius")
                        {
                            target_id_0_min_radius_ = param.as_double();
                        }
                        else if (name == "target_id_0_max_radius")
                        {
                            target_id_0_max_radius_ = param.as_double();
                        }
                        else if (name == "target_id_1_min_radius")
                        {
                            target_id_1_min_radius_ = param.as_double();
                        }
                        else if (name == "target_id_1_max_radius")
                        {
                            target_id_1_max_radius_ = param.as_double();
                        }
                        else if (name == "dart_input_mode")
                        {
                            const auto mode = param.as_string();
                            if (mode != "serial" && mode != "barcode")
                            {
                                rcl_interfaces::msg::SetParametersResult result;
                                result.successful = false;
                                result.reason = "dart_input_mode must be 'serial' or 'barcode'";
                                return result;
                            }
                            dart_input_mode_ = mode;
                        }
                        else if (name == "barcode_require_full_slots")
                        {
                            barcode_require_full_slots_ = param.as_bool();
                        }
                    }

                    normalizeRadiusRange(
                        manual_min_radius_, manual_max_radius_, this->get_logger(), "manual");
                    normalizeRadiusRange(
                        target_id_0_min_radius_, target_id_0_max_radius_,
                        this->get_logger(), "target_id_0");
                    normalizeRadiusRange(
                        target_id_1_min_radius_, target_id_1_max_radius_,
                        this->get_logger(), "target_id_1");

                    if (use_target_id_)
                    {
                        applyTargetIdRadius();
                    }
                    else
                    {
                        applyManualRadius();
                    }

                    if (isBarcodeMode())
                    {
                        updateActiveBarcodeProfile();
                    }

                    rcl_interfaces::msg::SetParametersResult result;
                    result.successful = true;
                    return result;
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
                pnp_solver_ =
                    std::make_unique<PnPSolver>(
                        camera_info->k, camera_info->d, pnp_circle_radius_mm_);
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

    void LightDetectorNode::applyManualRadius()
    {
        float min_radius = static_cast<float>(manual_min_radius_);
        float max_radius = static_cast<float>(manual_max_radius_);
        detector_->setRadiusRange(min_radius, max_radius);
        RCLCPP_INFO(this->get_logger(),
                    "Using manual radius range: %.2f ~ %.2f",
                    min_radius, max_radius);
    }

    void LightDetectorNode::applyTargetIdRadius()
    {
        double min_radius = 0.0;
        double max_radius = 0.0;
        if (target_id_ == 0)
        {
            min_radius = target_id_0_min_radius_;
            max_radius = target_id_0_max_radius_;
        }
        else if (target_id_ == 1)
        {
            min_radius = target_id_1_min_radius_;
            max_radius = target_id_1_max_radius_;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(),
                        "Unknown target_id: %u (expected 0-outpost, 1-base)",
                        target_id_);
            return;
        }
        detector_->setRadiusRange(
            static_cast<float>(min_radius), static_cast<float>(max_radius));
        RCLCPP_INFO(this->get_logger(),
                    "Using target_id=%u radius range: %.2f ~ %.2f",
                    target_id_, min_radius, max_radius);
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

    bool LightDetectorNode::isBarcodeMode() const
    {
        return dart_input_mode_ == "barcode";
    }

    int LightDetectorNode::normalizeShotIndex(uint8_t shot_index) const
    {
        if (barcode_slot_count_ <= 1)
        {
            return 1;
        }
        if (shot_index == 0)
        {
            return 1;
        }
        const int normalized =
            (static_cast<int>(shot_index) - 1) % barcode_slot_count_;
        return normalized + 1;
    }

    bool LightDetectorNode::isBarcodeSlotsReady() const
    {
        return std::all_of(
            barcode_slots_.begin(), barcode_slots_.end(),
            [](const BarcodeSlot &slot)
            { return slot.valid; });
    }

    void LightDetectorNode::resetBarcodeSlots()
    {
        for (auto &slot : barcode_slots_)
        {
            slot.valid = false;
            slot.dart_id = 0;
            slot.offset_deg = 0.0;
        }
        next_scan_slot_ = 1;
        active_scan_slot_ = 1;
        has_active_barcode_slot_ = false;
    }

    void LightDetectorNode::barcodeProfileCallback(
        const auto_aim_interfaces::msg::DartProfile::SharedPtr msg)
    {
        if (!msg)
        {
            return;
        }

        if (next_scan_slot_ > barcode_slot_count_)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Barcode slots are full, restarting from slot 1 with a new scan round");
            resetBarcodeSlots();
        }

        const int target_slot = next_scan_slot_;
        auto &slot = barcode_slots_[static_cast<size_t>(target_slot - 1)];
        slot.dart_id = msg->dart_id;
        slot.offset_deg = static_cast<double>(msg->offset_deg);
        slot.valid = true;
        next_scan_slot_++;

        RCLCPP_INFO(this->get_logger(),
                    "Stored barcode slot %d/%d: dart_id=%u offset=%.3f",
                    target_slot, barcode_slot_count_, slot.dart_id, slot.offset_deg);

        if (isBarcodeMode())
        {
            updateActiveBarcodeProfile();
        }
    }

    void LightDetectorNode::updateActiveBarcodeProfile()
    {
        if (!isBarcodeMode())
        {
            return;
        }

        if (barcode_require_full_slots_ && !isBarcodeSlotsReady())
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Barcode mode waiting for full scan set (%d slots required)",
                barcode_slot_count_);
            return;
        }

        const int requested_slot = normalizeShotIndex(serial_shot_index_);
        if (requested_slot < 1 || requested_slot > barcode_slot_count_)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Shot index %u maps to invalid slot %d", serial_shot_index_, requested_slot);
            return;
        }

        const auto &slot = barcode_slots_[static_cast<size_t>(requested_slot - 1)];
        if (!slot.valid)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Requested slot %d is not ready, keeping last valid slot", requested_slot);
            return;
        }

        active_scan_slot_ = requested_slot;
        active_offset_deg_ = slot.offset_deg;
        has_active_barcode_slot_ = true;
    }

    double LightDetectorNode::getEffectiveOffsetDeg()
    {
        if (!isBarcodeMode())
        {
            return serial_offset_deg_;
        }

        updateActiveBarcodeProfile();
        if (has_active_barcode_slot_)
        {
            return active_offset_deg_;
        }

        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Barcode mode has no valid active slot, fallback offset=0");
        return 0.0;
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

        if (lights.empty())
        {
            RCLCPP_DEBUG_THROTTLE(
                this->get_logger(), *get_clock(), 200,
                "No lights detected, sending no-light packet");
            publishNoLightSend(img_msg);
            return;
        }

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
            publishNoLightSend(img_msg);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Detected %zu lights", lights.size());

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
            if (lights_msg_.lights.empty())
            {
                RCLCPP_DEBUG_THROTTLE(
                    this->get_logger(), *get_clock(), 200,
                    "No valid PnP result, sending no-light packet");
                publishNoLightSend(img_msg);
                return;
            }
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

                const double effective_offset = getEffectiveOffsetDeg();
                const double pixel_angle = smooth_angle + effective_offset;
                send_msg.distance = raw_dist;
                send_msg.pixel_angle = static_cast<float>(pixel_angle);
                // /Send_pnp 阶段保持 angle 与 pixel_angle 相同，真实角由 range_fusion 覆盖
                send_msg.angle = send_msg.pixel_angle;
                send_msg.longitudinal_distance = 0.0f;
                send_msg.lateral_distance = 0.0f;
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
                send_msg.stability = (std::abs(pixel_angle) <= 0.02) ? 1 : 0;
                send_pub_->publish(send_msg);

                prev_angle_ = raw_angle;
            }
            publishMarkers();
        }
        else
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *get_clock(), 1000,
                "PnP solver is not ready, sending no-light packet");
            publishNoLightSend(img_msg);
        }
    }

    void LightDetectorNode::publishNoLightSend(
        const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
    {
        auto no_light_msg = auto_aim_interfaces::msg::Send();
        no_light_msg.header = img_msg->header;
        no_light_msg.distance = 666.0f;
        no_light_msg.angle = 1234.0f;
        no_light_msg.pixel_angle = 1234.0f;
        no_light_msg.longitudinal_distance = 1111.0f;
        no_light_msg.lateral_distance = 2222.0f;
        no_light_msg.u = 0.0f;
        no_light_msg.v = 0.0f;
        no_light_msg.roi_radius = 0.0f;
        no_light_msg.stability = 0;
        send_pub_->publish(no_light_msg);
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

        double total_latency_ms = 0.0;
        bool has_total_latency = false;
        {
            std::lock_guard<std::mutex> lock(total_latency_mutex_);
            total_latency_ms = total_latency_ms_;
            has_total_latency = has_total_latency_;
        }
        std::stringstream total_latency_ss;
        if (has_total_latency)
        {
            total_latency_ss << "Total latency: " << std::fixed << std::setprecision(2)
                             << total_latency_ms << "ms";
        }
        else
        {
            total_latency_ss << "Total latency: N/A";
        }
        cv::putText(
            img, total_latency_ss.str(), cv::Point(10, 115), cv::FONT_HERSHEY_SIMPLEX, 0.7,
            cv::Scalar(255, 255, 0), 2);
        // 新增：如果已经计算出了 PnP 的 distance/angle，就把它们也画上去
        if (!lights_msg_.lights.empty())
        {
            // 只取第一个 light 的信息
            const auto &lm = lights_msg_.lights[0];
            double draw_distance = lm.distance;
            double draw_pixel_angle = lm.angle;
            double draw_real_angle = lm.angle;
            double draw_longitudinal = 0.0;
            double draw_lateral = 0.0;
            bool using_lidar = false;
            {
                std::lock_guard<std::mutex> lock(fused_mutex_);
                const bool has_valid_lidar_result =
                    has_fused_send_ &&
                    (std::abs(last_fused_send_.longitudinal_distance) > 1e-3 ||
                     std::abs(last_fused_send_.lateral_distance) > 1e-3);
                if (has_valid_lidar_result)
                {
                    draw_distance = last_fused_send_.distance;
                    draw_pixel_angle = last_fused_send_.pixel_angle;
                    draw_real_angle = last_fused_send_.angle;
                    draw_longitudinal = last_fused_send_.longitudinal_distance;
                    draw_lateral = last_fused_send_.lateral_distance;
                    using_lidar = true;
                }
            }
            char info_angle[192];
            char info_dist[192];
            std::snprintf(info_angle, sizeof(info_angle),
                          "Dist=%.2fm PixelAng=%.2fdeg RealAng=%.2fdeg%s",
                          draw_distance, draw_pixel_angle, draw_real_angle,
                          using_lidar ? " [lidar]" : "");
            std::snprintf(info_dist, sizeof(info_dist),
                          "LongDist=%.2fm LatDist=%.2fm",
                          draw_longitudinal, draw_lateral);
            cv::putText(img, info_angle, cv::Point(10, 60),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(0, 255, 255), 2);
            cv::putText(img, info_dist, cv::Point(10, 85),
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
