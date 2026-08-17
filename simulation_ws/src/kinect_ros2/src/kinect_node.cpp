#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <libfreenect_sync.h>
#include <chrono>
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>

class KinectNode : public rclcpp::Node {
public:
    KinectNode() : Node("kinect_node") {
        const int requested_tilt = declare_parameter<int>("tilt_degrees", 8);
        tilt_degrees_ = load_saved_tilt(requested_tilt);
        pub_rgb_   = create_publisher<sensor_msgs::msg::Image>("/camera/rgb/image_raw", 5);
        pub_depth_ = create_publisher<sensor_msgs::msg::Image>("/camera/depth/image_raw", 5);
        pub_rgb_info_ = create_publisher<sensor_msgs::msg::CameraInfo>("/camera/rgb/camera_info", 5);
        pub_depth_info_ = create_publisher<sensor_msgs::msg::CameraInfo>("/camera/depth/camera_info", 5);
        pub_points_ = create_publisher<sensor_msgs::msg::PointCloud2>("/camera/depth/points", 2);
        rgb_fx_ = declare_parameter<double>("rgb_fx", 529.215);
        rgb_fy_ = declare_parameter<double>("rgb_fy", 525.564);
        rgb_cx_ = declare_parameter<double>("rgb_cx", 328.943);
        rgb_cy_ = declare_parameter<double>("rgb_cy", 267.481);
        depth_fx_ = declare_parameter<double>("depth_fx", 594.214);
        depth_fy_ = declare_parameter<double>("depth_fy", 591.041);
        depth_cx_ = declare_parameter<double>("depth_cx", 339.308);
        depth_cy_ = declare_parameter<double>("depth_cy", 242.739);
        point_cloud_stride_ = std::max(
            1, static_cast<int>(declare_parameter<int>("point_cloud_stride", 2)));
        point_cloud_every_n_frames_ = std::max(
            1, static_cast<int>(declare_parameter<int>("point_cloud_every_n_frames", 3)));
        pub_tilt_ = create_publisher<std_msgs::msg::Int32>("/kinect/tilt/state", 5);
        sub_tilt_set_ = create_subscription<std_msgs::msg::Int32>(
            "/kinect/tilt/set", 5,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                set_tilt(msg->data);
            });
        sub_tilt_save_ = create_subscription<std_msgs::msg::Bool>(
            "/kinect/tilt/save", 5,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data) {
                    save_tilt();
                }
            });
        const int tilt_result = freenect_sync_set_tilt_degs(tilt_degrees_, 0);
        RCLCPP_INFO(
            get_logger(),
            "Kinect node iniciado (libfreenect_sync), inclinacion=%d deg, resultado=%d",
            tilt_degrees_, tilt_result
        );
        timer_ = create_wall_timer(std::chrono::milliseconds(66),  // ~15 fps
                                   std::bind(&KinectNode::capture, this));
        tilt_timer_ = create_wall_timer(
            std::chrono::seconds(1), std::bind(&KinectNode::publish_tilt, this));
        publish_tilt();
    }
    ~KinectNode() { freenect_sync_stop(); }

private:
    std::filesystem::path tilt_file() const {
        const char* home = std::getenv("HOME");
        const std::filesystem::path base =
            home ? std::filesystem::path(home) : std::filesystem::path("/tmp");
        return base / ".config" / "smart_trolley" / "kinect_tilt_deg";
    }

    int load_saved_tilt(int fallback) const {
        int saved = fallback;
        std::ifstream input(tilt_file());
        if (input >> saved) {
            return std::clamp(saved, -30, 30);
        }
        return std::clamp(fallback, -30, 30);
    }

    void publish_tilt() {
        std_msgs::msg::Int32 msg;
        msg.data = tilt_degrees_;
        pub_tilt_->publish(msg);
    }

    void set_tilt(int requested) {
        const int next = std::clamp(requested, -30, 30);
        if (next == tilt_degrees_) {
            publish_tilt();
            return;
        }
        if (freenect_sync_set_tilt_degs(next, 0) == 0) {
            tilt_degrees_ = next;
            publish_tilt();
            RCLCPP_INFO(get_logger(), "Inclinacion Kinect ajustada a %d deg", next);
        } else {
            RCLCPP_WARN(get_logger(), "No se pudo ajustar Kinect a %d deg", next);
        }
    }

    void save_tilt() {
        const auto path = tilt_file();
        std::error_code error;
        std::filesystem::create_directories(path.parent_path(), error);
        std::ofstream output(path, std::ios::trunc);
        if (!output) {
            RCLCPP_ERROR(get_logger(), "No se pudo guardar inclinacion Kinect");
            return;
        }
        output << tilt_degrees_ << '\n';
        RCLCPP_INFO(
            get_logger(), "Centro teorico Kinect guardado: %d deg", tilt_degrees_);
        publish_tilt();
    }

    void capture() {
        const auto steady_now = std::chrono::steady_clock::now();
        if (steady_now < retry_after_) {
            return;
        }

        bool rgb_ok = false;
        bool depth_ok = false;
        // RGB frame
        void* rgb_data = nullptr; uint32_t ts = 0;
        if (freenect_sync_get_video(&rgb_data, &ts, 0, FREENECT_VIDEO_RGB) == 0) {
            rgb_ok = true;
            auto msg = sensor_msgs::msg::Image();
            msg.header.stamp = now();
            msg.header.frame_id = "camera_rgb_optical_frame";
            msg.height = 480; msg.width = 640;
            msg.encoding = "rgb8"; msg.step = 640 * 3;
            msg.data.resize(640 * 480 * 3);
            std::memcpy(msg.data.data(), rgb_data, msg.data.size());
            pub_rgb_->publish(msg);
            pub_rgb_info_->publish(camera_info(msg.header, rgb_fx_, rgb_fy_, rgb_cx_, rgb_cy_));
        }
        // Depth frame
        void* dep_data = nullptr;
        // Publish metric millimetres. FREENECT_DEPTH_11BIT contains raw
        // disparity codes (2047 means invalid), which must not be labelled and
        // consumed as 16-bit millimetres.
        if (freenect_sync_get_depth(&dep_data, &ts, 0, FREENECT_DEPTH_MM) == 0) {
            depth_ok = true;
            auto msg = sensor_msgs::msg::Image();
            msg.header.stamp = now();
            msg.header.frame_id = "camera_depth_optical_frame";
            msg.height = 480; msg.width = 640;
            msg.encoding = "16UC1"; msg.step = 640 * 2;
            msg.data.resize(640 * 480 * 2);
            std::memcpy(msg.data.data(), dep_data, msg.data.size());
            pub_depth_->publish(msg);
            pub_depth_info_->publish(camera_info(
                msg.header, depth_fx_, depth_fy_, depth_cx_, depth_cy_));
            if ((depth_frame_count_++ % point_cloud_every_n_frames_) == 0) {
                publish_point_cloud(static_cast<const uint16_t*>(dep_data), msg.header);
            }
        }

        if (rgb_ok || depth_ok) {
            consecutive_failures_ = 0;
            return;
        }

        ++consecutive_failures_;
        if (consecutive_failures_ >= 3) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Kinect sin frames; reiniciando libfreenect y reintentando"
            );
            freenect_sync_stop();
            consecutive_failures_ = 0;
            retry_after_ = steady_now + std::chrono::seconds(2);
        }
    }

    sensor_msgs::msg::CameraInfo camera_info(
        const std_msgs::msg::Header& header, double fx, double fy, double cx, double cy) const {
        sensor_msgs::msg::CameraInfo info;
        info.header = header;
        info.height = 480;
        info.width = 640;
        info.distortion_model = "plumb_bob";
        info.d.assign(5, 0.0);
        info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
        info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        info.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
        return info;
    }

    void publish_point_cloud(
        const uint16_t* depth_mm, const std_msgs::msg::Header& header) {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header = header;
        cloud.height = 480 / point_cloud_stride_;
        cloud.width = 640 / point_cloud_stride_;
        cloud.is_dense = false;
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(cloud.height * cloud.width);
        sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");
        for (int v = 0; v < 480; v += point_cloud_stride_) {
            for (int u = 0; u < 640; u += point_cloud_stride_, ++out_x, ++out_y, ++out_z) {
                const uint16_t millimetres = depth_mm[v * 640 + u];
                if (millimetres == 0 || millimetres >= 10000) {
                    const float nan = std::numeric_limits<float>::quiet_NaN();
                    *out_x = nan; *out_y = nan; *out_z = nan;
                    continue;
                }
                const float z = static_cast<float>(millimetres) * 0.001f;
                *out_x = static_cast<float>((u - depth_cx_) * z / depth_fx_);
                *out_y = static_cast<float>((v - depth_cy_) * z / depth_fy_);
                *out_z = z;
            }
        }
        pub_points_->publish(cloud);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_rgb_, pub_depth_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_rgb_info_, pub_depth_info_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_points_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_tilt_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_tilt_set_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_tilt_save_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr tilt_timer_;
    int consecutive_failures_{0};
    int tilt_degrees_{8};
    int point_cloud_stride_{2};
    int point_cloud_every_n_frames_{3};
    uint64_t depth_frame_count_{0};
    double rgb_fx_{0.0}, rgb_fy_{0.0}, rgb_cx_{0.0}, rgb_cy_{0.0};
    double depth_fx_{0.0}, depth_fy_{0.0}, depth_cx_{0.0}, depth_cy_{0.0};
    std::chrono::steady_clock::time_point retry_after_{};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinectNode>());
    rclcpp::shutdown();
}
