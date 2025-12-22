#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class SubscriberNode : public rclcpp::Node
{
public:
    SubscriberNode() : Node("dev_subscriber_node")
    {
        // QoS policy for subscribers
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

        // Subscriber for std msg
        subcriber_ = this->create_subscription<std_msgs::msg::Float32>("simplerosrpi", qos_profile,
                                                                      std::bind(&SubscriberNode::string_callback, this, std::placeholders::_1));

        }

private:
    // Callback for std msg
    void string_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received Distance'%f'", msg->data);
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subcriber_;
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubscriberNode>());
    rclcpp::shutdown();
    return 0;
}