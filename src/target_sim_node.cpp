#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "air_defense_sim/explosion.hpp"

using namespace std::chrono_literals;

// Simulates the ground-truth ballistic trajectory of the incoming target and
// broadcasts its pose as a tf frame (world -> target). It also draws itself
// in RViz: the body, the trail it has flown so far, and — if it reaches the
// ground without being intercepted — the explosion where it lands.
class TargetSimNode : public rclcpp::Node {
public:
  TargetSimNode() : Node("target_sim_node") {
    // Parameters (overridable from launch file / YAML)
    initial_position_ = declare_parameter<std::vector<double>>(
        "initial_position", {0.0, 0.0, 0.0});
    nominal_velocity_ = declare_parameter<std::vector<double>>(
        "initial_velocity", {10.0, 0.0, 30.0});
    // Every new target gets a velocity drawn uniformly in
    //   nominal_velocity_ +/- velocity_spread_   (per axis).
    // Without this every shot would fly exactly the same arc and the
    // defence would always give exactly the same result.
    velocity_spread_ = declare_parameter<std::vector<double>>(
        "velocity_spread", {0.0, 0.0, 0.0});
    gravity_ = declare_parameter<double>("gravity", 9.81);
    update_rate_hz_ = declare_parameter<double>("update_rate", 50.0);
    loop_ = declare_parameter<bool>("loop", true);
    trail_step_ = declare_parameter<double>("trail_step", 0.5);

    // The ground impact is the "we missed" event, so it gets a big, slow,
    // wide explosion. The air kill drawn by the interceptor is smaller.
    explosion_.ns = "ground_explosion";
    explosion_.duration = 1.8;
    explosion_.max_radius = 9.0;
    explosion_.debris_speed = 12.0;

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "target/markers", 10);

    // The interceptor tells us when it destroyed us: we then respawn a new
    // target immediately, without an explosion on the ground.
    hit_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        "interceptor/hit", 10,
        [this](const geometry_msgs::msg::PointStamped::SharedPtr) {
          restart(now());
        });

    rng_.seed(std::random_device{}());
    restart(now());
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / update_rate_hz_),
        std::bind(&TargetSimNode::update, this));
  }

private:
  void update() {
    const rclcpp::Time stamp = now();
    double t = (stamp - start_time_).seconds();

    // p(t) = p0 + v0*t + 0.5*a*t^2, with gravity acting only on z
    double x = initial_position_[0] + velocity_[0] * t;
    double y = initial_position_[1] + velocity_[1] * t;
    double z = initial_position_[2] + velocity_[2] * t -
               0.5 * gravity_ * t * t;

    if (z <= 0.0 && t > 0.0) {
      if (loop_) {
        // We were never intercepted: the target hits the ground and the
        // defence has failed. Blow up here, then send in the next one.
        explosion_.trigger(x, y, 0.0, stamp);
        RCLCPP_WARN(get_logger(), "MISS: target hit the ground at (%.1f, %.1f)",
                    x, y);
        restart(stamp);
        t = 0.0;
        x = initial_position_[0];
        y = initial_position_[1];
        z = initial_position_[2];
      } else {
        z = 0.0;  // landed: keep broadcasting the resting pose
      }
    }

    // Broadcast world -> target
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = "world";
    tf_msg.child_frame_id = "target";
    tf_msg.transform.translation.x = x;
    tf_msg.transform.translation.y = y;
    tf_msg.transform.translation.z = z;
    // Identity rotation is fine for a point target; revisit if you want the
    // frame oriented along the velocity vector.
    tf_msg.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(tf_msg);

    appendToTrail(x, y, z);
    publishMarkers(stamp, x, y, z);
  }

  // Send in a fresh target: draw a new launch velocity, reset the clock
  // and wipe the old trail.
  void restart(const rclcpp::Time& stamp) {
    for (int i = 0; i < 3; ++i) {
      std::uniform_real_distribution<double> spread(-velocity_spread_[i],
                                                    velocity_spread_[i]);
      velocity_[i] = nominal_velocity_[i] + spread(rng_);
    }
    start_time_ = stamp;
    trail_.clear();
  }

  // Record where we have been, but only every trail_step_ metres: at 50 Hz,
  // storing every single tick would give a needlessly heavy line strip.
  void appendToTrail(double x, double y, double z) {
    if (!trail_.empty()) {
      const auto& last = trail_.back();
      const double dx = x - last.x, dy = y - last.y, dz = z - last.z;
      if (std::sqrt(dx * dx + dy * dy + dz * dz) < trail_step_) {
        return;
      }
    }
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    trail_.push_back(p);
    if (trail_.size() > max_trail_points_) {
      trail_.erase(trail_.begin());
    }
  }

  void publishMarkers(const rclcpp::Time& stamp, double x, double y,
                      double z) {
    visualization_msgs::msg::MarkerArray array;

    // The threat itself: a red sphere.
    visualization_msgs::msg::Marker body;
    body.header.stamp = stamp;
    body.header.frame_id = "world";
    body.ns = "target";
    body.id = 0;
    body.type = visualization_msgs::msg::Marker::SPHERE;
    body.action = visualization_msgs::msg::Marker::ADD;
    body.pose.position.x = x;
    body.pose.position.y = y;
    body.pose.position.z = z;
    body.pose.orientation.w = 1.0;
    body.scale.x = body.scale.y = body.scale.z = 1.5;
    body.color.r = 1.0;
    body.color.g = 0.15;
    body.color.a = 1.0;
    array.markers.push_back(body);

    // The path it has flown: a LINE_STRIP joins the stored points in order.
    // RViz needs at least two points to draw a line.
    if (trail_.size() >= 2) {
      visualization_msgs::msg::Marker trail;
      trail.header = body.header;
      trail.ns = "target_trail";
      trail.id = 1;
      trail.type = visualization_msgs::msg::Marker::LINE_STRIP;
      trail.action = visualization_msgs::msg::Marker::ADD;
      trail.pose.orientation.w = 1.0;
      trail.scale.x = 0.35;  // for a line strip, scale.x is the line width
      trail.color.r = 1.0;
      trail.color.g = 0.35;
      trail.color.a = 0.9;
      trail.points = trail_;
      array.markers.push_back(trail);
    }

    // The ground explosion, while it is still playing.
    if (explosion_.active(stamp)) {
      const auto blast = explosion_.markers(stamp);
      array.markers.insert(array.markers.end(), blast.markers.begin(),
                           blast.markers.end());
    }

    marker_pub_->publish(array);
  }

  std::vector<double> initial_position_;
  std::vector<double> nominal_velocity_;
  std::vector<double> velocity_spread_;
  std::vector<double> velocity_{0.0, 0.0, 0.0};  // this target's own velocity
  std::mt19937 rng_;
  double gravity_;
  double update_rate_hz_;
  bool loop_;
  double trail_step_;

  rclcpp::Time start_time_;
  std::vector<geometry_msgs::msg::Point> trail_;
  const size_t max_trail_points_ = 400;
  Explosion explosion_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr hit_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetSimNode>());
  rclcpp::shutdown();
  return 0;
}
