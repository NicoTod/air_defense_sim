#include <chrono>
#include <memory>
#include <random>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

// Simulates a radar: reads the true target position from tf (in its own
// radar_link frame), corrupts it with Gaussian noise, and publishes the
// noisy measurement as a PoseStamped. The world -> radar_link transform is
// published by a static_transform_publisher in the launch file.
class RadarSensorNode : public rclcpp::Node {
public:
  RadarSensorNode() : Node("radar_sensor_node") {
    noise_stddev_ = declare_parameter<double>("noise_stddev", 0.5);
    update_rate_hz_ = declare_parameter<double>("update_rate", 10.0);

    // tf listener: fills tf_buffer_ with every transform broadcast on /tf,
    // so we can query "where is the target relative to the radar?"
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    measurement_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "radar/measurement", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
        "radar/measurement_markers", 10);

    // Gaussian noise generator: mean 0, standard deviation noise_stddev_
    rng_.seed(std::random_device{}());
    noise_ = std::normal_distribution<double>(0.0, noise_stddev_);

    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / update_rate_hz_),
        std::bind(&RadarSensorNode::measure, this));
  }

private:
  void measure() {
    // Target position in the radar's own frame (this is what a real radar
    // would output: a detection relative to itself, not in world coords).
    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_->lookupTransform("radar_link", "target",
                                           tf2::TimePointZero);
    } catch (const tf2::TransformException&) {
      // target_sim_node not broadcasting yet; try again on the next tick
      return;
    }

    geometry_msgs::msg::PoseStamped meas;
    meas.header.stamp = now();
    meas.header.frame_id = "radar_link";
    meas.pose.position.x = tf_msg.transform.translation.x + noise_(rng_);
    meas.pose.position.y = tf_msg.transform.translation.y + noise_(rng_);
    meas.pose.position.z = tf_msg.transform.translation.z + noise_(rng_);
    meas.pose.orientation.w = 1.0;
    measurement_pub_->publish(meas);

    // Red dots in RViz: keep the last max_points_ measurements
    recent_points_.push_back(meas.pose.position);
    if (recent_points_.size() > max_points_) {
      recent_points_.erase(recent_points_.begin());
    }

    visualization_msgs::msg::Marker marker;
    marker.header.stamp = meas.header.stamp;
    marker.header.frame_id = "radar_link";
    marker.ns = "radar";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = marker.scale.y = 0.15;
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    marker.points = recent_points_;
    marker_pub_->publish(marker);
  }

  double noise_stddev_;
  double update_rate_hz_;

  std::mt19937 rng_;
  std::normal_distribution<double> noise_;
  std::vector<geometry_msgs::msg::Point> recent_points_;
  const size_t max_points_ = 200;

  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr measurement_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RadarSensorNode>());
  rclcpp::shutdown();
  return 0;
}
