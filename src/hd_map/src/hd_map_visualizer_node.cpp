#include "hd_map/hd_map_visualizer_node.hpp"

#include <cmath>
#include <string>

namespace hd_map_visualizer
{

// Color palette: dark-background optimised
//
//  Roads (normal)   : steel blue      #4A90D9  (0.29, 0.56, 0.85)
//  Roads (junction) : soft cyan       #5EC4C4  (0.37, 0.77, 0.77)
//  Left boundary    : warm white      #E8E8E0  (0.91, 0.91, 0.88)
//  Right boundary   : mid grey        #888880  (0.53, 0.53, 0.50)
//  Junction sphere  : amber           #F5A623  (0.96, 0.65, 0.14)
//  Junction label   : bright yellow   #FFE066  (1.00, 0.88, 0.40)
//  Lane ID text     : light lavender  #C8BFE7  (0.78, 0.75, 0.91)
//  Direction arrow  : lime green      #7ED321  (0.49, 0.83, 0.13)

// Static helpers

std_msgs::msg::ColorRGBA HDMapVisualizerNode::rgba(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

geometry_msgs::msg::Point HDMapVisualizerNode::pt(double x, double y, double z)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

HDMapVisualizerNode::HDMapVisualizerNode(const rclcpp::NodeOptions & options)
: Node("hd_map_visualizer_node", options)
{
  declareParameters();
  loadMap();
  createPublishers();

  // First publish immediately
  publishAll();

  // Re-publish periodically so RViz picks up after a late connect
  const double period = this->declare_parameter<double>("republish_period_s", 5.0);
  republish_timer_ =
    this->create_wall_timer(std::chrono::duration<double>(period), [this]() { publishAll(); });
}

void HDMapVisualizerNode::declareParameters()
{
  this->declare_parameter<std::string>("map_path", "maps/Town01_hd_map.json");
  this->declare_parameter<std::string>("frame_id", "map");
  this->declare_parameter<bool>("show_boundaries", true);
  this->declare_parameter<bool>("show_lane_ids", true);
  this->declare_parameter<bool>("show_directions", true);
  this->declare_parameter<double>("centerline_z_offset", 0.15);

  frame_id_ = this->get_parameter("frame_id").as_string();
  z_offset_ = this->get_parameter("centerline_z_offset").as_double();
  show_boundaries_ = this->get_parameter("show_boundaries").as_bool();
  show_lane_ids_ = this->get_parameter("show_lane_ids").as_bool();
  show_directions_ = this->get_parameter("show_directions").as_bool();
}

void HDMapVisualizerNode::loadMap()
{
  const std::string map_path = this->get_parameter("map_path").as_string();

  RCLCPP_INFO(get_logger(), "Loading HD map: %s", map_path.c_str());

  if (!map_.load(map_path)) {
    RCLCPP_FATAL(get_logger(), "Failed to load HD map from: %s", map_path.c_str());
    throw std::runtime_error("HD map load failed");
  }

  const auto & meta = map_.getMetadata();
  RCLCPP_INFO(
    get_logger(), "Map loaded: name: %s | lanelets: %d | junctions: %d", meta.map_name.c_str(),
    meta.total_lanelets, meta.total_junctions);
}

void HDMapVisualizerNode::createPublishers()
{
  // TRANSIENT_LOCAL = latching: late RViz subscribers get the last message
  auto qos = rclcpp::QoS(1).transient_local().reliable();

  pub_centerlines_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("/hd_map/centerlines", qos);
  pub_boundaries_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("/hd_map/boundaries", qos);
  pub_junctions_ = create_publisher<visualization_msgs::msg::MarkerArray>("/hd_map/junctions", qos);
  pub_lane_ids_ = create_publisher<visualization_msgs::msg::MarkerArray>("/hd_map/lane_ids", qos);
  pub_directions_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("/hd_map/directions", qos);
}

void HDMapVisualizerNode::publishAll()
{
  pub_centerlines_->publish(buildCenterlines());
  pub_junctions_->publish(buildJunctions());

  if (show_boundaries_) {
    pub_boundaries_->publish(buildBoundaries());
  }
  if (show_lane_ids_) {
    pub_lane_ids_->publish(buildLaneIds());
  }
  if (show_directions_) {
    pub_directions_->publish(buildDirections());
  }
}

visualization_msgs::msg::Marker HDMapVisualizerNode::makeMarker(
  const std::string & ns, int id, int32_t type) const
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id_;
  m.header.stamp = now();
  m.ns = ns;
  m.id = id;
  m.type = type;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.orientation.w = 1.0;
  return m;
}

visualization_msgs::msg::MarkerArray HDMapVisualizerNode::buildCenterlines() const
{
  visualization_msgs::msg::MarkerArray ma;
  const auto & lanelets = map_.getAllLanelets();
  ma.markers.reserve(lanelets.size());

  int id = 0;
  for (const auto & lane : lanelets) {
    auto m = makeMarker("centerlines", id++, visualization_msgs::msg::Marker::LINE_STRIP);

    if (lane.is_junction) {
      // Junction lanes: soft cyan, slightly thicker
      m.scale.x = 0.14;
      m.color = rgba(0.37f, 0.77f, 0.77f, 0.95f);
    } else {
      // Normal lanes: steel blue
      m.scale.x = 0.09;
      m.color = rgba(0.29f, 0.56f, 0.85f, 1.0f);
    }

    m.points.reserve(lane.centerline.size());
    for (const auto & cp : lane.centerline) {
      m.points.push_back(pt(cp.position.x, cp.position.y, cp.position.z + z_offset_));
    }

    ma.markers.push_back(std::move(m));
  }
  return ma;
}

visualization_msgs::msg::MarkerArray HDMapVisualizerNode::buildBoundaries() const
{
  visualization_msgs::msg::MarkerArray ma;
  const auto & lanelets = map_.getAllLanelets();
  ma.markers.reserve(lanelets.size() * 2);

  int id = 0;
  for (const auto & lane : lanelets) {
    // Left boundary: warm white
    {
      auto m = makeMarker("boundary_left", id++, visualization_msgs::msg::Marker::LINE_STRIP);
      m.scale.x = 0.05;
      m.color = rgba(0.91f, 0.91f, 0.88f, 0.65f);
      m.points.reserve(lane.left_boundary.size());
      for (const auto & p : lane.left_boundary) {
        m.points.push_back(pt(p.x, p.y, p.z + z_offset_));
      }
      ma.markers.push_back(std::move(m));
    }

    // Right boundary: mid grey
    {
      auto m = makeMarker("boundary_right", id++, visualization_msgs::msg::Marker::LINE_STRIP);
      m.scale.x = 0.05;
      m.color = rgba(0.53f, 0.53f, 0.50f, 0.55f);
      m.points.reserve(lane.right_boundary.size());
      for (const auto & p : lane.right_boundary) {
        m.points.push_back(pt(p.x, p.y, p.z + z_offset_));
      }
      ma.markers.push_back(std::move(m));
    }
  }
  return ma;
}

visualization_msgs::msg::MarkerArray HDMapVisualizerNode::buildJunctions() const
{
  visualization_msgs::msg::MarkerArray ma;
  const auto & junctions = map_.getAllJunctions();
  ma.markers.reserve(junctions.size() * 2);

  int id = 0;
  for (const auto & junc : junctions) {
    // Amber disc
    {
      auto m = makeMarker("junction_disc", id++, visualization_msgs::msg::Marker::CYLINDER);
      m.scale.x = 8.0;   // diameter x
      m.scale.y = 8.0;   // diameter y
      m.scale.z = 0.08;  // flat disc
      m.color = rgba(0.96f, 0.65f, 0.14f, 0.30f);
      m.pose.position = pt(junc.center_x, junc.center_y, z_offset_ - 0.05);
      ma.markers.push_back(std::move(m));
    }

    // Bright yellow text label
    {
      auto m =
        makeMarker("junction_label", id++, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
      m.scale.z = 2.5;
      m.color = rgba(1.00f, 0.88f, 0.40f, 1.0f);
      m.text = "J" + std::to_string(junc.id) + "\n" + junc.type;
      m.pose.position = pt(junc.center_x, junc.center_y, z_offset_ + 2.8);
      ma.markers.push_back(std::move(m));
    }
  }
  return ma;
}

visualization_msgs::msg::MarkerArray HDMapVisualizerNode::buildLaneIds() const
{
  visualization_msgs::msg::MarkerArray ma;
  const auto & lanelets = map_.getAllLanelets();
  ma.markers.reserve(lanelets.size());

  int id = 0;
  for (const auto & lane : lanelets) {
    if (lane.centerline.empty()) {
      continue;
    }

    // Place label at lane midpoint
    const auto & mid = lane.centerline[lane.centerline.size() / 2];

    auto m = makeMarker("lane_ids", id++, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    m.scale.z = 0.7;
    m.color = rgba(0.78f, 0.75f, 0.91f, 0.85f);  // light lavender
    m.text = std::to_string(lane.road_id) + "/" + std::to_string(lane.lane_id);
    m.pose.position = pt(mid.position.x, mid.position.y, mid.position.z + z_offset_ + 0.6);
    ma.markers.push_back(std::move(m));
  }
  return ma;
}

visualization_msgs::msg::MarkerArray HDMapVisualizerNode::buildDirections() const
{
  visualization_msgs::msg::MarkerArray ma;
  const auto & lanelets = map_.getAllLanelets();
  ma.markers.reserve(lanelets.size());

  constexpr double ARROW_LEN = 2.5;
  int id = 0;

  for (const auto & lane : lanelets) {
    if (lane.centerline.size() < 2) {
      continue;
    }

    const auto & mid = lane.centerline[lane.centerline.size() / 2];
    const double yaw = mid.heading_rad;
    const double z = mid.position.z + z_offset_ + 0.2;

    auto m = makeMarker("directions", id++, visualization_msgs::msg::Marker::ARROW);
    m.scale.x = 0.12;  // shaft diameter
    m.scale.y = 0.30;  // head diameter
    m.scale.z = 0.0;
    m.color = rgba(0.49f, 0.83f, 0.13f, 0.90f);  // lime green

    m.points = {
      pt(mid.position.x, mid.position.y, z),
      pt(
        mid.position.x + std::cos(yaw) * ARROW_LEN, mid.position.y + std::sin(yaw) * ARROW_LEN, z)};
    ma.markers.push_back(std::move(m));
  }
  return ma;
}

}  // namespace hd_map_visualizer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hd_map_visualizer::HDMapVisualizerNode>());
  rclcpp::shutdown();
  return 0;
}
