#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;

// Simulates the ground-truth ballistic trajectory of the target and
// broadcasts its pose as a tf frame (world -> target).
class TargetSimNode : public rclcpp::Node {
public:
  TargetSimNode() : Node("target_sim_node") {
    // Parameters (overridable from launch file / YAML)
    initial_position_ = declare_parameter<std::vector<double>>(
        "initial_position", {0.0, 0.0, 0.0});
    initial_velocity_ = declare_parameter<std::vector<double>>(
        "initial_velocity", {10.0, 0.0, 30.0});
    gravity_ = declare_parameter<double>("gravity", 9.81);
    update_rate_hz_ = declare_parameter<double>("update_rate", 50.0);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
        "target_marker", 10);

    start_time_ = now();
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / update_rate_hz_),
        std::bind(&TargetSimNode::update, this));
  }

private:
  void update() {
    const double t = (now() - start_time_).seconds();

    // TODO(you) Phase 1, step 1: compute the ballistic position at time t
    //   p(t) = p0 + v0 * t + 0.5 * a * t^2   with a = (0, 0, -gravity_)
    //   Store it in x, y, z below.
    double x = 0.0, y = 0.0, z = 0.0;

    // TODO(you) Phase 1, step 2: stop (or reset) the simulation when the
    //   target hits the ground (z <= 0).

    // Broadcast world -> target
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = now();
    tf_msg.header.frame_id = "world";
    tf_msg.child_frame_id = "target";
    tf_msg.transform.translation.x = x;
    tf_msg.transform.translation.y = y;
    tf_msg.transform.translation.z = z;
    // Identity rotation is fine for a point target; revisit if you want the
    // frame oriented along the velocity vector.
    tf_msg.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(tf_msg);

    // TODO(you) Phase 1, step 3: publish a visualization_msgs::msg::Marker
    //   (type SPHERE, frame "world") at the target position so you can see
    //   it in RViz. Docs: https://wiki.ros.org/rviz/DisplayTypes/Marker
  }

  std::vector<double> initial_position_;
  std::vector<double> initial_velocity_;
  double gravity_;
  double update_rate_hz_;

  rclcpp::Time start_time_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetSimNode>());
  rclcpp::shutdown();
  return 0;
}
