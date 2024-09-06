#include "custom_interfaces/msg/age.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"

using namespace std::chrono_literals;

class CustomPublisher : public rclcpp::Node
{
    public:
    CustomPublisher() 
    : rclcpp::Node("age_publisher_node")
    {
        publisher_ = this->create_publisher<custom_interfaces::msg::Age>("robot_age", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
            std::bind(&CustomPublisher::age_callback, this));
    }

    private:
    void age_callback()
    {
        custom_interfaces::msg::Age msg; 
        msg.years = 2.0f;
        msg.months = 4.0f;
        msg.days = 14.0f;

        publisher_->publish(msg);
    }
    rclcpp::Publisher<custom_interfaces::msg::Age>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CustomPublisher>());
    rclcpp::shutdown();
    return 0;
}