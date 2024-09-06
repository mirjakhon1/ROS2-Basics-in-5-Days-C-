#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using std::placeholders::_1;

class SimpleSubscriber : public rclcpp::Node{
    public:
    SimpleSubscriber()
    : Node("topic_subscriber_node")
    {
        subsriber_ = this->create_subscription<std_msgs::msg::Int32>(
            "counter", 10, std::bind(&SimpleSubscriber::topic_callback, this, _1));
    }

    private:
    void topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->data);
    }
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subsriber_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleSubscriber>());
    rclcpp::shutdown();
    return 0;
}