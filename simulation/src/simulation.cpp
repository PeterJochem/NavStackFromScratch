#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdlib.h>
#include <memory>
#include <vector>
#include <exception>
#include "turtlelib/rigid2d.hpp"


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"


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
      declare_parameter("obstacles/x", std::vector<double>{});
      declare_parameter("obstacles/y", std::vector<double>{});
      declare_parameter("obstacles/radius", 1.0);

      rate_hz = get_parameter("rate").as_double();
      x0 = get_parameter("x0").as_double();
      y0 = get_parameter("y0").as_double();
      theta0 = get_parameter("theta0").as_double();

      std::vector<double> obstacles_x = get_parameter("obstacles/x").as_double_array();
      std::vector<double> obstacles_y = get_parameter("obstacles/y").as_double_array();
      if (obstacles_x.size() != obstacles_y.size()) {
          throw std::runtime_error("The obstacles must have the same length.");
      }
      for (int i = 0; i < obstacles_x.size(); i++) {
        obstacle_locations.emplace_back(turtlelib::Vector2D(obstacles_x[i], obstacles_y[i]));
      }

      obstacle_radius = get_parameter("obstacles/radius").as_double();

      tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      timestep_publisher = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
      std::chrono::milliseconds rate_ms = (std::chrono::milliseconds) ((int)(1000. / rate_hz));
      timer = this->create_wall_timer(rate_ms, std::bind(&SimulationNode::timer_callback, this));
      world_to_base_ground_truth = turtlelib::Transform2D(turtlelib::Vector2D(x0, y0), theta0);
    }

  private:
    void reset(std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
      RCLCPP_INFO_STREAM(get_logger(), "Resetting!");
      count_ms = 0;
      world_to_base_ground_truth = turtlelib::Transform2D(turtlelib::Vector2D(x0, y0), theta0);
    }

    void timer_callback()
    {
      // Publish and update the timer.
      auto message = std_msgs::msg::UInt64();
      message.data = count_ms;
      timestep_publisher->publish(message);
      count_ms++;

      // Publish and update the ground truth transform.
      publish_transform();

      // Publish the obstacles.
      // visualization_msgs/MarkerArray

    }

    void publish_transform() {

      geometry_msgs::msg::TransformStamped transform;

      transform.header.stamp = this->get_clock()->now();
      transform.header.frame_id = "world";
      transform.child_frame_id = "turtle_ground_truth";

      auto translation = world_to_base_ground_truth.translation();
      transform.transform.translation.x = translation.x;
      transform.transform.translation.y = translation.y;
      transform.transform.translation.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0, 0, world_to_base_ground_truth.rotation());
      transform.transform.rotation.x = q.x();
      transform.transform.rotation.y = q.y();
      transform.transform.rotation.z = q.z();
      transform.transform.rotation.w = q.w();

      tf_broadcaster->sendTransform(transform);
    }


    double rate_hz;
    double x0, y0, theta0;
    size_t count_ms;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service = create_service<std_srvs::srv::Empty>("~/reset", std::bind(&SimulationNode::reset, this, std::placeholders::_1, std::placeholders::_2));
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    turtlelib::Transform2D world_to_base_ground_truth;
    std::vector<turtlelib::Vector2D> obstacle_locations;
    double obstacle_radius;
    double obstacle_height = 0.25;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimulationNode>());
  rclcpp::shutdown();
  return 0;
}
