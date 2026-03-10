#ifndef LIGHT_DETECTOR__DETECTOR_NODE_HPP_
#define LIGHT_DETECTOR__DETECTOR_NODE_HPP_
// ROS
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// STD
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <cstddef>

#include "./detector.hpp"
#include "pnp_solver.hpp"
#include "kalman_filter.hpp"
#include "auto_aim_interfaces/msg/light.hpp"
#include "auto_aim_interfaces/msg/lights.hpp"
#include "auto_aim_interfaces/msg/send.hpp"

namespace rm_auto_aim_dart
{
    class LightDetectorNode : public rclcpp::Node
    {
    public:
        explicit LightDetectorNode(const rclcpp::NodeOptions &options);

    private:
        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg);

        std::unique_ptr<Detector> initDectector();
        void chooseBestPose(Detector::Light &light, const std::vector<cv::Mat> &rvecs, const std::vector<cv::Mat> &tvecs, cv::Mat &rvec, cv::Mat &tvec);
        std::vector<Detector::Light> detectLights(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg, cv::Mat &img);
        void drawResults(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg, cv::Mat &img, const std::vector<Detector::Light> &lights);

        void createDebugPublishers();
        void destroyDebugPublishers();

        void publishMarkers();
        void updateLatencyStats(const builtin_interfaces::msg::Time &stamp);

        // Light Detector
        std::unique_ptr<Detector> detector_;

        // Detected armors publisher
        auto_aim_interfaces::msg::Lights lights_msg_;
        rclcpp::Publisher<auto_aim_interfaces::msg::Light>::SharedPtr light_pub_;
        // 新增：Send 消息发布者
        rclcpp::Publisher<auto_aim_interfaces::msg::Send>::SharedPtr send_pub_;
        // **新增：飞镖编号订阅 & 偏移映射**
        // 当前飞镖编号（dart_id）
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr dart_sub_;
        uint8_t current_dart_id_{1}; // 默认 1
        // <<< NEW: subscription for serial offset >>>
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr offset_sub_;
        float offset_{0.0f};
        // std::map<int, double> dart_offset_map_;
        // std::map<int, double> dart_stability_threshold_map_;

        // Visualization marker publisher
        visualization_msgs::msg::Marker light_marker_;
        visualization_msgs::msg::Marker text_marker_;
        visualization_msgs::msg::MarkerArray marker_array_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
        double current_latency_ms_{0.0};
        double total_latency_ms_{0.0};
        double max_latency_ms_{0.0};
        std::size_t latency_samples_{0};

        // Camera info
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
        cv::Point2f camera_center_;
        std::shared_ptr<sensor_msgs::msg::CameraInfo> camera_info_;
        std::unique_ptr<PnPSolver> pnp_solver_;

        // Image subscription
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

        // tf2
        std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
        Eigen::Matrix3d imu_to_camera;

        // Kalman filter
        KalmanFilter angle_filter_;
        float prev_angle_{0.0};
        float jump_threshold_{0.03}; // 运动切换阈值（rad）
        float Q_big_{1.0}, Q_small_{1e-3};
        float R_angle_;

        // --- 新增：比赛模式开关 ---
        uint8_t competition_mode_{0};
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr competition_sub_;

        // Debug
        bool debug_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_cb_handle_;
        std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
        rclcpp::Publisher<auto_aim_interfaces::msg::DebugLights>::SharedPtr lights_data_pub_;
        image_transport::Publisher binary_img_pub_;
        image_transport::Publisher result_img_pub_;

        double light_radius_; // 用于Rviz，需要直接给出数值
    };
} // namespace rm_auto_aim_dart
#endif
