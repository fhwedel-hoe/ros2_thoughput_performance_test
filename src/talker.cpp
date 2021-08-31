#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_calibration_parsers/parse.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>

namespace throughput_performance_test {

class Talker : public rclcpp::Node {

private:
    std::shared_ptr<sensor_msgs::msg::Image> image_ptr;
    std::shared_ptr<sensor_msgs::msg::CameraInfo> camera_info_msg_ptr;
    std::string camera_info_path;
    int64_t rate_cycle_milliseconds = 22; // 45 fps by default
    image_transport::CameraPublisher image_publisher;
    std::unique_ptr<std::thread> producer_thread;

public:
    Talker(const rclcpp::NodeOptions & options) : Node("throughput_performance_test", options) {
        rate_cycle_milliseconds = this->declare_parameter("rate_millis", rate_cycle_milliseconds);
        camera_info_path = this->declare_parameter("camera_info_yaml", camera_info_path);
        if (camera_info_path.empty()) {
            camera_info_path = ament_index_cpp::get_package_prefix("throughput_performance_test")
                               + "/share/throughput_performance_test/config/"
                               + "camera_calibration.yaml";
        }
        camera_info_msg_ptr = std::make_shared<sensor_msgs::msg::CameraInfo>();
        std::string camera_name;
        if (!camera_calibration_parsers::readCalibration(camera_info_path, camera_name, *camera_info_msg_ptr)) {
            RCLCPP_ERROR(get_logger(), "camera_info was not loaded");
            throw std::runtime_error("camera_info was not loaded");
        }
        image_ptr = std::make_shared<sensor_msgs::msg::Image>();
        const size_t width = 1280;
        const size_t height = 960;
        const size_t channels = 3;
        image_ptr->width = width;
        image_ptr->height = height;
        image_ptr->encoding = sensor_msgs::image_encodings::RGB8;
        image_ptr->step = width * channels;
        image_ptr->data.resize(height * width * channels); // allocate memory once
        // have a test pattern
        for (size_t y = 0; y < height; y++) {
            for (size_t x = 0; x < width; x++) {
                for (size_t c = 0; c < channels; c++) {
                    image_ptr->data[y*width*channels+x*channels+c] = (x % 16) * (y % 16);
                }
            }
        }
        image_publisher = image_transport::create_camera_publisher(this, "image", rclcpp::QoS(1).get_rmw_qos_profile());
        producer_thread = std::make_unique<std::thread>([this](){
            // have the main loop in a thread since blocking functions and rclcpp::spin() are mutually exclusive
            rclcpp::Rate rate(std::chrono::milliseconds(22));
            while(rclcpp::ok()) {
                publish();
                rate.sleep();
            }
        });
    }
    void publish() {
        image_ptr->header.stamp = this->now();
        image_ptr->header.frame_id = "image_frame";
        camera_info_msg_ptr->header.stamp = image_ptr->header.stamp;
        camera_info_msg_ptr->header.frame_id = image_ptr->header.frame_id;
        image_publisher.publish(image_ptr, camera_info_msg_ptr);
    }
};

} // end namespace throughput_performance_test

RCLCPP_COMPONENTS_REGISTER_NODE(throughput_performance_test::Talker)
