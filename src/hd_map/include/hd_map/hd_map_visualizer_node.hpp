#ifndef HD_MAP__HD_MAP_VISUALIZER_NODE_HPP_
#define HD_MAP__HD_MAP_VISUALIZER_NODE_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "hd_map/hd_map.hpp"

namespace hd_map_visualizer
{

/**
 * @class HDMapVisualizerNode
 * @brief ROS 2 node that publishes HD map geometry as RViz MarkerArrays.
 *
 * Loads the HD map once on startup and publishes five latched topics:
 *
 *   /hd_map/centerlines  : lane centerlines
 *   /hd_map/boundaries   : left and right lane edges
 *   /hd_map/junctions    : junction zones and ID labels
 *   /hd_map/lane_ids     : road/lane text labels at lane midpoints
 *   /hd_map/directions   : travel-direction arrows
 *
 * All topics use TRANSIENT_LOCAL durability so RViz receives markers
 * even when it connects after the node starts.
 *
 * Parameters:
 *   map_path            (string) : path to Town01_hd_map.json
 *   frame_id            (string) : RViz fixed frame, default "map"
 *   show_boundaries     (bool)   : toggle boundary lines
 *   show_lane_ids       (bool)   : toggle road/lane ID labels
 *   show_directions     (bool)   : toggle direction arrows
 *   centerline_z_offset (double) : lift markers above ground plane
 *   republish_period_s  (double) : how often to re-publish (default 5 s)
 */
class HDMapVisualizerNode : public rclcpp::Node
{
public:
  explicit HDMapVisualizerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
  void declareParameters();
  void loadMap();
  void createPublishers();
  void publishAll();

  visualization_msgs::msg::MarkerArray buildCenterlines() const;
  visualization_msgs::msg::MarkerArray buildBoundaries() const;
  visualization_msgs::msg::MarkerArray buildJunctions() const;
  visualization_msgs::msg::MarkerArray buildLaneIds() const;
  visualization_msgs::msg::MarkerArray buildDirections() const;

  visualization_msgs::msg::Marker makeMarker(const std::string & ns, int id, int32_t type) const;

  static std_msgs::msg::ColorRGBA rgba(float r, float g, float b, float a = 1.0f);

  static geometry_msgs::msg::Point pt(double x, double y, double z = 0.0);

  hd_map::HDMap map_;

  std::string frame_id_;
  double z_offset_;
  bool show_boundaries_;
  bool show_lane_ids_;
  bool show_directions_;

  using MarkerArrayPub = rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr;

  MarkerArrayPub pub_centerlines_;
  MarkerArrayPub pub_boundaries_;
  MarkerArrayPub pub_junctions_;
  MarkerArrayPub pub_lane_ids_;
  MarkerArrayPub pub_directions_;

  rclcpp::TimerBase::SharedPtr republish_timer_;
};

}  // namespace hd_map_visualizer

#endif  // HD_MAP__HD_MAP_VISUALIZER_NODE_HPP_
