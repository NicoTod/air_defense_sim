#include <array>
#include <chrono>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;

// Draws the scenery: the ground, the city we are defending, the radar tower
// and the launcher. Nothing here moves, but RViz only knows about markers it
// has actually received, so we simply republish the same array once per
// second: that way the scene also shows up if RViz is started late.
class WorldNode : public rclcpp::Node {
public:
  WorldNode() : Node("world_node") {
    ground_size_ = declare_parameter<double>("ground_size", 200.0);
    auto radar = declare_parameter<std::vector<double>>("radar_position",
                                                        {30.0, 0.0});
    auto launcher = declare_parameter<std::vector<double>>("launcher_position",
                                                           {50.0, 10.0});
    radar_x_ = radar[0];
    radar_y_ = radar[1];
    launcher_x_ = launcher[0];
    launcher_y_ = launcher[1];

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "world_markers", 10);
    timer_ = create_wall_timer(1s, std::bind(&WorldNode::publishScene, this));
  }

private:
  void publishScene() {
    const rclcpp::Time stamp = now();
    visualization_msgs::msg::MarkerArray array;
    int id = 0;

    // --- the ground: one very flat box, sitting just below z = 0 --------
    auto ground = makeMarker(stamp, "ground", id++,
                             visualization_msgs::msg::Marker::CUBE);
    ground.pose.position.z = -0.1;
    ground.scale.x = ground.scale.y = ground_size_;
    ground.scale.z = 0.2;
    setColor(ground, 0.18, 0.28, 0.18, 1.0);  // dark green terrain
    array.markers.push_back(ground);

    // --- the city we are defending --------------------------------------
    for (const auto& b : buildings_) {
      auto block = makeMarker(stamp, "city", id++,
                              visualization_msgs::msg::Marker::CUBE);
      block.pose.position.x = b[0];
      block.pose.position.y = b[1];
      block.pose.position.z = b[4] / 2.0;  // a box is centred on its origin
      block.scale.x = b[2];
      block.scale.y = b[3];
      block.scale.z = b[4];
      setColor(block, 0.55, 0.57, 0.62, 1.0);  // concrete grey
      array.markers.push_back(block);
    }

    // --- the radar tower, at the same spot as the radar_link frame -------
    auto radar = makeMarker(stamp, "radar", id++,
                            visualization_msgs::msg::Marker::CYLINDER);
    radar.pose.position.x = radar_x_;
    radar.pose.position.y = radar_y_;
    radar.pose.position.z = 4.0;
    radar.scale.x = radar.scale.y = 3.0;
    radar.scale.z = 8.0;
    setColor(radar, 0.20, 0.60, 0.70, 1.0);  // teal
    array.markers.push_back(radar);

    // --- the launcher, where the interceptor waits -----------------------
    auto launcher = makeMarker(stamp, "launcher", id++,
                               visualization_msgs::msg::Marker::CYLINDER);
    launcher.pose.position.x = launcher_x_;
    launcher.pose.position.y = launcher_y_;
    launcher.pose.position.z = 1.5;
    launcher.scale.x = launcher.scale.y = 6.0;
    launcher.scale.z = 3.0;
    setColor(launcher, 0.75, 0.45, 0.15, 1.0);  // orange pad, easy to spot
    array.markers.push_back(launcher);

    marker_pub_->publish(array);
  }

  // The fields every scenery marker shares.
  visualization_msgs::msg::Marker makeMarker(const rclcpp::Time& stamp,
                                             const std::string& ns, int id,
                                             int32_t type) {
    visualization_msgs::msg::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = "world";
    m.ns = ns;
    m.id = id;
    m.type = type;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    return m;  // no lifetime: the scenery stays until we say otherwise
  }

  static void setColor(visualization_msgs::msg::Marker& m, double r, double g,
                       double b, double a) {
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = a;
  }

  // The skyline: {centre x, centre y, width, depth, height}. The footprint
  // deliberately spans x = 55..83, which is the band the targets are aimed
  // at (see target_sim_node's initial_velocity / velocity_spread): a target
  // that gets through lands among the buildings wherever in that band it
  // was headed, instead of in an empty field behind them.
  // Edit this table to change what the scene looks like.
  const std::vector<std::array<double, 5>> buildings_ = {
      {58.0, -6.0, 6.0, 6.0, 14.0},
      {64.0, 2.0, 8.0, 6.0, 22.0},
      {70.0, -8.0, 6.0, 8.0, 10.0},
      {72.0, 6.0, 7.0, 7.0, 18.0},
      {62.0, -14.0, 5.0, 5.0, 8.0},
      {77.0, -4.0, 7.0, 7.0, 16.0},
      {80.0, 5.0, 6.0, 6.0, 12.0},
      {76.0, -12.0, 5.0, 5.0, 9.0},
  };

  double ground_size_;
  double radar_x_, radar_y_;
  double launcher_x_, launcher_y_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WorldNode>());
  rclcpp::shutdown();
  return 0;
}
