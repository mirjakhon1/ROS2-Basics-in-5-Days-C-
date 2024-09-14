#include "rclcpp/timer.hpp"
#include "services_quiz_srv/srv/spin.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <functional>

using Spin = services_quiz_srv::srv::Spin;
using std::placeholders::_1;
using std::placeholders::_2;

class QuizService : public rclcpp::Node{
public:
    QuizService() 
    : rclcpp::Node("quiz_service_node")
    {
        service_ = this->create_service<Spin>("/rotate", 
            std::bind(&QuizService::service_callback, this, _1, _2));
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", 10);
    }

private:
    void service_callback(const std::shared_ptr<Spin::Request> request,
                          const std::shared_ptr<Spin::Response> response)
    {
        geometry_msgs::msg::Twist move_command;
        if(request->direction == "right" || request->direction == "left")
        {
            response->success = true;
            move_command.angular.z = (request->direction == "right") ? -request->angular_velocity : request->angular_velocity;
            pub_->publish(move_command);
            timer_ = this->create_wall_timer(std::chrono::seconds(request->time), std::bind(&QuizService::timer_callback, this));
        }
        
        else {
            response->success = false;
        }
    }

    void timer_callback()
    {
        geometry_msgs::msg::Twist move_command;
        move_command.angular.z = 0.0;
        pub_->publish(move_command);
    }

    rclcpp::Service<Spin>::SharedPtr service_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QuizService>());
  rclcpp::shutdown();
  return 0;
}