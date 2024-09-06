#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <chrono>
#include <vector>

using std::placeholders::_1;

class AvoidSphere : public rclcpp::Node {
public:
  AvoidSphere() : rclcpp::Node("topics_quiz_node") {
    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&AvoidSphere::scan_callback, this, _1));
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(500),
                                std::bind(&AvoidSphere::pub_callback, this));
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

private:

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_data) {
    // Constants
    const float min_distance = 0.7;                                    // min distance to an obstacle
    const float turn_speed = 2;
    const float move_speed = 1;
    const int source_array_size = 719;                                 // LaserScan ranges array size 
    const int target_array_size = 80;                                  // segmented array size

    std::vector<float> front_scan(target_array_size);               // array for front
    std::vector<float> left_scan(target_array_size);
    std::vector<float> right_scan(target_array_size);
    int startingIndexFront = (source_array_size - target_array_size) / 2;  // index of target array withing source array
    int startingIndexRight = 0;
    int startingIndexLeft = source_array_size - target_array_size;

    create_sub_scan(scan_data, front_scan, startingIndexFront, target_array_size);
    create_sub_scan(scan_data, left_scan, startingIndexLeft, target_array_size);
    create_sub_scan(scan_data, right_scan, startingIndexRight, target_array_size);


    // Find minimum value in the target array
    auto minValFront = std::min_element(std::begin(front_scan), std::end(front_scan));
    auto minValRight = std::min_element(std::begin(right_scan), std::end(right_scan));
    auto minValLeft = std::min_element(std::begin(left_scan), std::end(left_scan));

    // Turn
    if (*minValFront < min_distance) 
    {
      move_commands.linear.x = 0;
      move_commands.angular.z = turn_speed;
      RCLCPP_INFO(this->get_logger(), "Left-center");
    } 
    else if(*minValRight < min_distance)
    {
      move_commands.linear.x = 0;
      move_commands.angular.z = -turn_speed;
      RCLCPP_INFO(this->get_logger(), "Left");
    } 
    else if(*minValLeft < min_distance)
    {
      move_commands.linear.x = 0;
      move_commands.angular.z = turn_speed;
      RCLCPP_INFO(this->get_logger(), "Right");
    } 
    else 
    {
      move_commands.linear.x = move_speed;
      move_commands.angular.z = 0;
    }
  }
  
  void create_sub_scan(const sensor_msgs::msg::LaserScan::SharedPtr &source_array, std::vector<float> &output_array, int sub_index, int target_array_size)
  {
    // Create target array
    for (int i = 0; i < target_array_size; ++i) {
      float range = source_array->ranges.at(sub_index + i);
      // Check for invalid (NaN, inf) ranges
      if (!std::isfinite(range)) {
        // RCLCPP_INFO(this->get_logger(), "Infinite value");
        output_array[i] = std::numeric_limits<float>::max();
      } else {
        output_array[i] = range;
      }
    }
  }

  void pub_callback() { publisher_->publish(move_commands); }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Twist move_commands;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AvoidSphere>());
  rclcpp::shutdown();
  return 0;
}