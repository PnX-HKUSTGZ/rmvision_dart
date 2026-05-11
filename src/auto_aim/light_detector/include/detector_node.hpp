#ifndef LIGHT_DETECTOR__DETECTOR_NODE_HPP_
#define LIGHT_DETECTOR__DETECTOR_NODE_HPP_
// ROS
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
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
#include <mutex>

#include "./detector.hpp"
#include "pnp_solver.hpp"
#include "kalman_filter.hpp"
#include "auto_aim_interfaces/msg/light.hpp"
#include "auto_aim_interfaces/msg/lights.hpp"
#include "auto_aim_interfaces/msg/send.hpp"
#include "auto_aim_interfaces/msg/dart_profile.hpp"

namespace rm_auto_aim_dart
{
    class LightDetectorNode : public rclcpp::Node
    {
    public:
        explicit LightDetectorNode(const rclcpp::NodeOptions &options);

    private:
        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg);

        std::unique_ptr<Detector> initDectector();
        void applyManualRadius();
        void applyTargetIdRadius();
        void chooseBestPose(Detector::Light &light, const std::vector<cv::Mat> &rvecs, const std::vector<cv::Mat> &tvecs, cv::Mat &rvec, cv::Mat &tvec);
        std::vector<Detector::Light> detectLights(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg, cv::Mat &img);
        void drawResults(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg, cv::Mat &img, const std::vector<Detector::Light> &lights);
        void publishNoLightSend(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg);

        void createDebugPublishers();
        void destroyDebugPublishers();

        void publishMarkers();
        void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void fusedSendCallback(const auto_aim_interfaces::msg::Send::SharedPtr msg);
        void barcodeProfileCallback(const auto_aim_interfaces::msg::DartProfile::SharedPtr msg);
        void updateActiveBarcodeProfile();
        int normalizeShotIndex(uint8_t shot_index) const;
        bool isBarcodeSlotsReady() const;
        bool isBarcodeMode() const;
        void resetBarcodeSlots();
        double getEffectiveOffsetDeg();
        void drawPointCloudOnImage(
            const sensor_msgs::msg::Image::ConstSharedPtr &img_msg,
            cv::Mat &img);

        // Light Detector
        std::unique_ptr<Detector> detector_;

        // Detected armors publisher
        auto_aim_interfaces::msg::Lights lights_msg_;
        rclcpp::Publisher<auto_aim_interfaces::msg::Light>::SharedPtr light_pub_;
        // 新增：Send 消息发布者
        rclcpp::Publisher<auto_aim_interfaces::msg::Send>::SharedPtr send_pub_;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr dart_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr offset_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr total_latency_sub_;
        rclcpp::Subscription<auto_aim_interfaces::msg::DartProfile>::SharedPtr barcode_profile_sub_;

        std::string dart_input_mode_{"serial"};
        std::string serial_dart_topic_{"current_dart_id"};
        std::string serial_offset_topic_{"offset"};
        std::string barcode_profile_topic_{"barcode/scan_profile"};
        std::string total_latency_topic_{"/latency"};
        int barcode_slot_count_{4};
        bool barcode_require_full_slots_{true};

        uint8_t serial_shot_index_{1};
        double serial_offset_deg_{0.0};
        double active_offset_deg_{0.0};
        std::mutex total_latency_mutex_;
        double total_latency_ms_{0.0};
        bool has_total_latency_{false};

        struct BarcodeSlot
        {
            uint8_t dart_id{0};
            double offset_deg{0.0};
            bool valid{false};
        };
        std::vector<BarcodeSlot> barcode_slots_;
        int next_scan_slot_{1};
        int active_scan_slot_{1};
        bool has_active_barcode_slot_{false};

        // Visualization marker publisher
        visualization_msgs::msg::Marker light_marker_;
        visualization_msgs::msg::Marker text_marker_;
        visualization_msgs::msg::MarkerArray marker_array_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

        // Camera info
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
        cv::Point2f camera_center_;
        std::shared_ptr<sensor_msgs::msg::CameraInfo> camera_info_;
        std::unique_ptr<PnPSolver> pnp_solver_;

        // Image subscription
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
        rclcpp::Subscription<auto_aim_interfaces::msg::Send>::SharedPtr fused_send_sub_;
        std::mutex cloud_mutex_;
        sensor_msgs::msg::PointCloud2::SharedPtr last_cloud_;
        std::mutex fused_mutex_;
        auto_aim_interfaces::msg::Send last_fused_send_;
        rclcpp::Time last_fused_stamp_;
        bool has_fused_send_{false};
        std::string camera_optical_frame_;
        std::string cloud_topic_;
        std::string fused_send_topic_;
        bool draw_cloud_{false};
        int cloud_draw_stride_{5};

        // tf2
        std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
        Eigen::Matrix3d imu_to_camera;

        // Kalman filter
        KalmanFilter angle_filter_;
        double prev_angle_{0.0};
        double jump_threshold_{0.03}; // 运动切换阈值（rad）
        double Q_big_{1.0}, Q_small_{1e-3};
        double R_angle_;

        // --- 新增：比赛模式开关 ---
        uint8_t competition_mode_{0};
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr competition_sub_;

        // --- 新增：目标ID订阅，用于动态设置半径阈值 ---
        uint8_t target_id_{1};
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr target_id_sub_;
        bool use_target_id_{true};
        double manual_min_radius_{20.0};
        double manual_max_radius_{50.0};
        double target_id_0_min_radius_{50.0};
        double target_id_0_max_radius_{80.0};
        double target_id_1_min_radius_{10.0};
        double target_id_1_max_radius_{50.0};
        double pnp_circle_radius_mm_{30.0};

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
