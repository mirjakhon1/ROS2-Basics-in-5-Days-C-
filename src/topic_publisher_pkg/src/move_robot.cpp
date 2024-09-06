#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>

using namespace std::chrono_literals;

class Move_robot : public rclcpp::Node {
public:
  Move_robot() 
  : rclcpp::Node("move_robot") {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&Move_robot::timer_callback, this));
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  void timer_callback() {
    geometry_msgs::msg::Twist message;
    message.linear.x = 2;
    message.angular.z = 1;
    publisher_->publish(message);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Move_robot>());
  rclcpp::shutdown();
  return 0;
}
