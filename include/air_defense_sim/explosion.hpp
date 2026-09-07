#pragma once

#include <algorithm>
#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// ---------------------------------------------------------------------------
// A short animated explosion, drawn entirely with RViz markers.
//
// Two nodes need one (the interceptor when it kills a target in the air, the
// target when it reaches the ground), so the animation lives here instead of
// being copy-pasted twice.
//
// How a node uses it:
//   1. call trigger(x, y, z, now) at the moment something blows up;
//   2. on every tick of the node's timer, if active(now) is true, publish the
//      markers returned by markers(now).
//
// Everything is a function of the normalized age
//     s = (now - start) / duration        s runs from 0 (bang) to 1 (gone)
//   radius = max_radius * sqrt(s)   -> expands fast, then slows down
//   alpha  = 1 - s                  -> fades out linearly
//   color  = yellow -> orange -> red
// ---------------------------------------------------------------------------
struct Explosion {
  // --- tuning knobs (a node may overwrite them after construction) ---
  double duration = 1.2;      // how long the whole animation lasts [s]
  double max_radius = 6.0;    // radius of the fireball at the end [m]
  int debris_count = 8;       // little spheres thrown outward
  double debris_speed = 9.0;  // how fast the debris fly outward [m/s]
  double debris_rise = 6.0;   // how fast the debris are thrown up [m/s]
  std::string ns = "explosion";  // marker namespace, so RViz keeps them apart

  // Start the animation at a point in the world frame.
  void trigger(double x, double y, double z, const rclcpp::Time& now) {
    cx_ = x;
    cy_ = y;
    cz_ = z;
    start_ = now;
    running_ = true;
  }

  // True while the animation still has something to show.
  bool active(const rclcpp::Time& now) const {
    return running_ && (now - start_).seconds() < duration;
  }

  // The fireball plus the debris, at the age they have right now.
  visualization_msgs::msg::MarkerArray markers(const rclcpp::Time& now) const {
    const double age = (now - start_).seconds();
    const double s = std::min(age / duration, 1.0);  // normalized 0..1

    visualization_msgs::msg::MarkerArray array;

    // --- the fireball ---------------------------------------------------
    visualization_msgs::msg::Marker ball = base(now, 0);
    ball.type = visualization_msgs::msg::Marker::SPHERE;
    ball.pose.position.x = cx_;
    ball.pose.position.y = cy_;
    ball.pose.position.z = cz_;
    // scale is the DIAMETER of the sphere, hence the factor 2
    const double radius = max_radius * std::sqrt(s);
    ball.scale.x = ball.scale.y = ball.scale.z = 2.0 * radius + 0.5;
    paint(ball, s, 1.0 - s);
    array.markers.push_back(ball);

    // --- the debris ring ------------------------------------------------
    for (int i = 0; i < debris_count; ++i) {
      // spread the pieces evenly around a circle
      const double theta = 2.0 * M_PI * i / debris_count;
      visualization_msgs::msg::Marker piece = base(now, i + 1);
      piece.type = visualization_msgs::msg::Marker::SPHERE;
      piece.pose.position.x = cx_ + debris_speed * age * std::cos(theta);
      piece.pose.position.y = cy_ + debris_speed * age * std::sin(theta);
      // thrown upward, then pulled back down by gravity; never below ground
      piece.pose.position.z =
          std::max(cz_ + debris_rise * age - 0.5 * 9.81 * age * age, 0.0);
      piece.scale.x = piece.scale.y = piece.scale.z = 1.3;
      paint(piece, s, 1.0 - s);
      array.markers.push_back(piece);
    }

    return array;
  }

private:
  // The fields every marker of the explosion shares.
  visualization_msgs::msg::Marker base(const rclcpp::Time& now, int id) const {
    visualization_msgs::msg::Marker m;
    m.header.stamp = now;
    m.header.frame_id = "world";
    m.ns = ns;
    m.id = id;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    // a lifetime slightly longer than one tick: when the node stops
    // republishing them, RViz removes them by itself
    m.lifetime = rclcpp::Duration::from_seconds(0.3);
    return m;
  }

  // Color ramp: bright yellow at s=0, orange in the middle, deep red at s=1.
  static void paint(visualization_msgs::msg::Marker& m, double s,
                    double alpha) {
    m.color.r = 1.0;
    m.color.g = std::max(0.9 - 0.9 * s, 0.0);
    m.color.b = std::max(0.5 - 1.5 * s, 0.0);
    m.color.a = std::max(alpha, 0.0);
  }

  double cx_ = 0.0, cy_ = 0.0, cz_ = 0.0;   // where it went off
  rclcpp::Time start_{0, 0, RCL_ROS_TIME};  // when it went off
  bool running_ = false;                    // has it ever been triggered?
};
