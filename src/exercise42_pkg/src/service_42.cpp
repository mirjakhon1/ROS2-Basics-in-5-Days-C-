#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

using SetBool = std_srvs::srv::SetBool;
using std::placeholders::_1;
using std::placeholders::_2;

class ServerNode : public rclcpp::Node {
public:
  ServerNode() : rclcpp::Node("service_42_node") {
    service_ = this->create_service<SetBool>(
        "custom_move_srv",
        std::bind(&ServerNode::service_callback, this, _1, _2));
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  void service_callback(const SetBool::Request::SharedPtr request,
                        const SetBool::Response::SharedPtr response) {
    geometry_msgs::msg::Twist move_command;

    if (request->data == true) {
      RCLCPP_INFO(this->get_logger(), "The robot is moving!");

      move_command.linear.x = 0.2;
      move_command.angular.z = -0.2;
      pub_->publish(move_command);
      response->message = "Turning to the right right right!";
    } else {
      RCLCPP_INFO(this->get_logger(), "Stopping the robot...");

      move_command.linear.x = 0.0;
      move_command.angular.z = 0.0;
      pub_->publish(move_command);
      response->message = "It is time to stop!";
    }
  }
  rclcpp::Service<SetBool>::SharedPtr service_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServerNode>());
  rclcpp::shutdown();
  return 0;
}
