#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <image_transport/image_transport.hpp>

namespace throughput_performance_test {

class Listener : public rclcpp::Node {
private:
    std::string topic = "image_rect";
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    int msg_count = 0;
    int32_t prev_sec;
    uint32_t checksum;
    
public:
    Listener(const rclcpp::NodeOptions & options) : Node("throughput_performance_test", options) {
        topic = this->declare_parameter("topic", topic);
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(topic, 1,
            std::bind(&Listener::imageCb, this, std::placeholders::_1)
        );
    }
    
    void imageCb(const sensor_msgs::msg::Image::SharedPtr msg) {
        msg_count++;
        for (auto i : msg->data) {
            checksum += i;
        }
        if (prev_sec != msg->header.stamp.sec) {
            RCLCPP_INFO(this->get_logger(), "Got % 2d messages per second, checksum is %u.", msg_count, checksum);
            prev_sec = msg->header.stamp.sec;
            msg_count = 0;
        }
    }
};

} // end namespace throughput_performance_test

RCLCPP_COMPONENTS_REGISTER_NODE(throughput_performance_test::Listener)
