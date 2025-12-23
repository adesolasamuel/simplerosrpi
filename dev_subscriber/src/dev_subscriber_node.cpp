#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

class SubscriberNode : public rclcpp::Node
{
public:
    SubscriberNode() : Node("dev_subscriber_node"),
                       sock(-1)
    {
        // QoS policy for subscribers
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

        // Subscriber for std msg
        subcriber_ = this->create_subscription<std_msgs::msg::Float32>("simplerosrpi", qos_profile,
                                                                       std::bind(&SubscriberNode::string_callback, this, std::placeholders::_1));

        connectToGui();
    }

private:
    // Connect to the GUI over the socket
    void connectToGui()
    {
        sock = socket(AF_INET, SOCK_STREAM, 0);
        struct sockaddr_in serv_addr;
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(5000);

        // We need to convert IPv4 and IPv6 from text to binary form
        inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr);

        if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "GUI Connection Failed!");
            sock = -1;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "GUI Connected");
        }
    }
    // Callback for std msg
    void string_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        if (sock != -1)
        {
            RCLCPP_INFO(this->get_logger(), "Received Distance'%f'", msg->data);
            std::string data = std::to_string(msg->data);
            send(sock, data.c_str(), data.length(), 0);
        }
    }

    int sock;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subcriber_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubscriberNode>());
    rclcpp::shutdown();
    return 0;
}