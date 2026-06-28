#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <libfreenect_sync.h>
#include <cstring>

class KinectNode : public rclcpp::Node {
public:
    KinectNode() : Node("kinect_node") {
        pub_rgb_   = create_publisher<sensor_msgs::msg::Image>("/camera/rgb/image_raw", 5);
        pub_depth_ = create_publisher<sensor_msgs::msg::Image>("/camera/depth/image_raw", 5);
        RCLCPP_INFO(get_logger(), "Kinect node iniciado (libfreenect_sync)");
        timer_ = create_wall_timer(std::chrono::milliseconds(66),  // ~15 fps
                                   std::bind(&KinectNode::capture, this));
    }
    ~KinectNode() { freenect_sync_stop(); }

private:
    void capture() {
        // RGB frame
        void* rgb_data = nullptr; uint32_t ts = 0;
        if (freenect_sync_get_video(&rgb_data, &ts, 0, FREENECT_VIDEO_RGB) == 0) {
            auto msg = sensor_msgs::msg::Image();
            msg.header.stamp = now();
            msg.header.frame_id = "camera_rgb_optical_frame";
            msg.height = 480; msg.width = 640;
            msg.encoding = "rgb8"; msg.step = 640 * 3;
            msg.data.resize(640 * 480 * 3);
            std::memcpy(msg.data.data(), rgb_data, msg.data.size());
            pub_rgb_->publish(msg);
        }
        // Depth frame
        void* dep_data = nullptr;
        if (freenect_sync_get_depth(&dep_data, &ts, 0, FREENECT_DEPTH_11BIT) == 0) {
            auto msg = sensor_msgs::msg::Image();
            msg.header.stamp = now();
            msg.header.frame_id = "camera_depth_optical_frame";
            msg.height = 480; msg.width = 640;
            msg.encoding = "16UC1"; msg.step = 640 * 2;
            msg.data.resize(640 * 480 * 2);
            std::memcpy(msg.data.data(), dep_data, msg.data.size());
            pub_depth_->publish(msg);
        }
    }
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_rgb_, pub_depth_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinectNode>());
    rclcpp::shutdown();
}
