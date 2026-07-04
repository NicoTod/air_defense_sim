#include <memory>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

// ---------------------------------------------------------------------------
// Discrete Kalman filter for a ballistic target.
//
// State x = [px py pz vx vy vz]^T   (position and velocity, world frame)
// Measurement z = [px py pz]^T      (noisy radar position, world frame)
//
// Motion model (constant acceleration = gravity, known):
//   p(k+1) = p(k) + v(k)*dt - [0 0 g]*dt^2/2
//   v(k+1) = v(k)           - [0 0 g]*dt
// ---------------------------------------------------------------------------
class KalmanFilter {
public:
  Eigen::Matrix<double, 6, 1> x;  // state estimate
  Eigen::Matrix<double, 6, 6> P;  // state covariance (our uncertainty)

  // Start the filter at a measured position, with unknown velocity.
  void init(const Eigen::Vector3d& position) {
    x.setZero();
    x.head<3>() = position;
    P = Eigen::Matrix<double, 6, 6>::Identity();
    P.bottomRightCorner<3, 3>() *= 100.0;  // velocity is a pure guess
  }

  // Prediction step: push the state forward by dt using the motion model.
  // q is the process noise intensity (how much we trust the model).
  void predict(double dt, double gravity, double q) {
    const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    // F: state transition matrix,  x(k+1) = F * x(k) + gravity effect
    Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
    F.topRightCorner<3, 3>() = dt * I;

    x = F * x;
    x(2) -= 0.5 * gravity * dt * dt;  // gravity on z position
    x(5) -= gravity * dt;             // gravity on z velocity

    // Q: process noise (white-noise acceleration model). Accounts for
    // everything the model ignores, integrated into position and velocity.
    Eigen::Matrix<double, 6, 6> Q;
    Q.topLeftCorner<3, 3>() = q * std::pow(dt, 4) / 4.0 * I;
    Q.topRightCorner<3, 3>() = q * std::pow(dt, 3) / 2.0 * I;
    Q.bottomLeftCorner<3, 3>() = q * std::pow(dt, 3) / 2.0 * I;
    Q.bottomRightCorner<3, 3>() = q * dt * dt * I;

    P = F * P * F.transpose() + Q;
  }

  // Correction step: blend the prediction with a measurement z.
  // r_stddev is the standard deviation of the measurement noise.
  void update(const Eigen::Vector3d& z, double r_stddev) {
    // H: measurement matrix, picks the position out of the state (z = H*x)
    Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
    H.leftCols<3>() = Eigen::Matrix3d::Identity();

    const Eigen::Matrix3d R =
        r_stddev * r_stddev * Eigen::Matrix3d::Identity();

    const Eigen::Vector3d innovation = z - H * x;
    const Eigen::Matrix3d S = H * P * H.transpose() + R;
    const Eigen::Matrix<double, 6, 3> K = P * H.transpose() * S.inverse();

    x = x + K * innovation;
    P = (Eigen::Matrix<double, 6, 6>::Identity() - K * H) * P;
  }
};

// ---------------------------------------------------------------------------
// ROS node: feeds radar measurements to the filter and publishes the result.
// ---------------------------------------------------------------------------
class EstimatorNode : public rclcpp::Node {
public:
  EstimatorNode() : Node("estimator_node") {
    measurement_stddev_ = declare_parameter<double>("measurement_stddev", 0.5);
    process_noise_ = declare_parameter<double>("process_noise", 1.0);
    gravity_ = declare_parameter<double>("gravity", 9.81);
    reinit_distance_ = declare_parameter<double>("reinit_distance", 8.0);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "estimator/pose", 10);
    impact_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
        "estimator/predicted_impact", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
        "estimator/markers", 10);

    measurement_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "radar/measurement", 10,
        std::bind(&EstimatorNode::onMeasurement, this, std::placeholders::_1));
  }

private:
  void onMeasurement(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // The radar measures in its own frame: transform it into the world
    // frame before filtering (this is why the estimator needs tf2).
    geometry_msgs::msg::PoseStamped meas_world;
    try {
      auto tf = tf_buffer_->lookupTransform("world", msg->header.frame_id,
                                            tf2::TimePointZero);
      tf2::doTransform(*msg, meas_world, tf);
    } catch (const tf2::TransformException&) {
      return;  // tf not available yet; skip this measurement
    }

    const Eigen::Vector3d z(meas_world.pose.position.x,
                            meas_world.pose.position.y,
                            meas_world.pose.position.z);
    const rclcpp::Time stamp(msg->header.stamp);

    if (!initialized_) {
      kf_.init(z);
      last_stamp_ = stamp;
      initialized_ = true;
      return;
    }

    // 1) predict forward to the time of this measurement
    const double dt = (stamp - last_stamp_).seconds();
    last_stamp_ = stamp;
    if (dt > 0.0) {
      kf_.predict(dt, gravity_, process_noise_);
    }

    // 2) track loss: if the measurement is far from the prediction the
    //    target has restarted (sim loops) -> reinitialize the filter
    if ((z - kf_.x.head<3>()).norm() > reinit_distance_) {
      kf_.init(z);
      return;
    }

    // 3) correct with the measurement
    kf_.update(z, measurement_stddev_);

    publishEstimate(stamp);
    publishPrediction(stamp);
  }

  void publishEstimate(const rclcpp::Time& stamp) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = "world";
    pose.pose.position.x = kf_.x(0);
    pose.pose.position.y = kf_.x(1);
    pose.pose.position.z = kf_.x(2);
    pose.pose.orientation.w = 1.0;
    pose_pub_->publish(pose);

    // Green sphere at the estimated position
    visualization_msgs::msg::Marker marker;
    marker.header = pose.header;
    marker.ns = "estimate";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose.pose;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.6;
    marker.color.g = 1.0;
    marker.color.a = 1.0;
    marker_pub_->publish(marker);
  }

  // Roll the ballistic model forward from the current estimate until the
  // ground (z = 0): gives the predicted trajectory and the impact point.
  void publishPrediction(const rclcpp::Time& stamp) {
    Eigen::Vector3d p = kf_.x.head<3>();
    Eigen::Vector3d v = kf_.x.tail<3>();

    visualization_msgs::msg::Marker line;
    line.header.stamp = stamp;
    line.header.frame_id = "world";
    line.ns = "predicted_path";
    line.id = 1;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.1;
    line.color.g = 1.0;
    line.color.a = 1.0;

    const double dt = 0.05;
    for (double t = 0.0; t < 30.0 && p.z() >= 0.0; t += dt) {
      geometry_msgs::msg::Point pt;
      pt.x = p.x();
      pt.y = p.y();
      pt.z = p.z();
      line.points.push_back(pt);
      p += v * dt;
      v.z() -= gravity_ * dt;
    }
    marker_pub_->publish(line);

    if (line.points.empty()) {
      return;
    }
    geometry_msgs::msg::PointStamped impact;
    impact.header = line.header;
    impact.point = line.points.back();  // last point is at ground level
    impact_pub_->publish(impact);

    visualization_msgs::msg::Marker impact_marker;
    impact_marker.header = line.header;
    impact_marker.ns = "predicted_impact";
    impact_marker.id = 2;
    impact_marker.type = visualization_msgs::msg::Marker::SPHERE;
    impact_marker.action = visualization_msgs::msg::Marker::ADD;
    impact_marker.pose.position = impact.point;
    impact_marker.pose.orientation.w = 1.0;
    impact_marker.scale.x = impact_marker.scale.y = impact_marker.scale.z = 1.0;
    impact_marker.color.r = 1.0;
    impact_marker.color.g = 1.0;
    impact_marker.color.a = 1.0;
    marker_pub_->publish(impact_marker);
  }

  double measurement_stddev_;
  double process_noise_;
  double gravity_;
  double reinit_distance_;

  KalmanFilter kf_;
  bool initialized_ = false;
  rclcpp::Time last_stamp_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr measurement_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr impact_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EstimatorNode>());
  rclcpp::shutdown();
  return 0;
}
