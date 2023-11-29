#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdlib.h>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class SimulationNode : public rclcpp::Node
{
  public:
    SimulationNode()
    : Node("simulation_node"), count_ms(0)
    {

      declare_parameter("rate", 200.);
      declare_parameter("x0", 0.);
      declare_parameter("y0", 0.);
      declare_parameter("theta0", 0.);

      rate_hz = get_parameter("rate").as_double();

      tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      timestep_publisher = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
      std::chrono::milliseconds rate_ms = (std::chrono::milliseconds) ((int)(1000. / rate_hz));
      timer = this->create_wall_timer(rate_ms, std::bind(&SimulationNode::timer_callback, this));
    }

  private:
    void reset(std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
      RCLCPP_INFO_STREAM(get_logger(), "Resetting!");
      count_ms = 0;
    }

    void timer_callback()
    {
      auto message = std_msgs::msg::UInt64();
      message.data = count_ms;
      timestep_publisher->publish(message);
      count_ms++;
    }

    double rate_hz;
    double x0, y0, theta0;
    size_t count_ms;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service = create_service<std_srvs::srv::Empty>("~/reset", std::bind(&SimulationNode::reset, this, std::placeholders::_1, std::placeholders::_2));
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimulationNode>());
  rclcpp::shutdown();
  return 0;
}
