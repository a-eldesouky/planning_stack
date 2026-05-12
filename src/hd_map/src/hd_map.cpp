#include "hd_map/hd_map.hpp"

#include <cmath>
#include <fstream>
#include <limits>

namespace hd_map
{

bool HDMap::load(const std::string & json_path)
{
  // Prevent re-loading if already loaded - this class is designed to be loaded once per instance
  if (loaded_) {
    return true;
  }

  std::ifstream file(json_path);
  if (!file.is_open()) {
    return false;
  }

  // Clear any partial state from a previous failed attempt before parsing again.
  lanelets_.clear();
  junctions_.clear();
  lanelet_index_.clear();
  junction_index_.clear();

  nlohmann::json j;
  try {
    file >> j;
  } catch (const nlohmann::json::parse_error & e) {
    return false;
  }

  try {
    parseMetadata(j.at("metadata"));
    parseLanelets(j.at("lanelets"));
    parseJunctions(j.at("junctions"));
    buildLookupTables();
  } catch (const std::exception & e) {
    return false;
  }

  loaded_ = true;
  return true;
}

void HDMap::parseMetadata(const nlohmann::json & j)
{
  metadata_.map_name = j.at("map_name").get<std::string>();
  metadata_.version = j.at("version").get<std::string>();
  metadata_.format = j.at("format").get<std::string>();
  metadata_.waypoint_spacing_m = j.at("waypoint_spacing_m").get<double>();
  metadata_.total_lanelets = j.at("total_lanelets").get<int>();
  metadata_.total_junctions = j.at("total_junctions").get<int>();
}

void HDMap::parseLanelets(const nlohmann::json & j)
{
  lanelets_.reserve(j.size());

  for (const auto & item : j) {
    Lanelet lane;
    lane.id = item.at("id").get<std::string>();
    lane.road_id = item.at("road_id").get<int>();
    lane.lane_id = item.at("lane_id").get<int>();
    lane.is_junction = item.at("is_junction").get<bool>();
    lane.junction_id = item.at("junction_id").get<int>();
    lane.length_m = item.at("length_m").get<double>();

    // Centerline points
    lane.centerline.reserve(item.at("centerline").size());
    for (const auto & pt : item.at("centerline")) {
      lane.centerline.push_back(
        {pt.at("x").get<double>(), pt.at("y").get<double>(), pt.at("z").get<double>(),
         pt.at("heading_rad").get<double>()});
    }

    // Left boundary
    lane.left_boundary.reserve(item.at("left_boundary").size());
    for (const auto & pt : item.at("left_boundary")) {
      lane.left_boundary.push_back(
        {pt.at("x").get<double>(), pt.at("y").get<double>(), pt.at("z").get<double>()});
    }

    // Right boundary
    lane.right_boundary.reserve(item.at("right_boundary").size());
    for (const auto & pt : item.at("right_boundary")) {
      lane.right_boundary.push_back(
        {pt.at("x").get<double>(), pt.at("y").get<double>(), pt.at("z").get<double>()});
    }

    // Graph connectivity
    lane.predecessors = item.at("predecessors").get<std::vector<std::string>>();
    lane.successors = item.at("successors").get<std::vector<std::string>>();
    lane.left_neighbor = item.at("left_neighbor").get<std::string>();
    lane.right_neighbor = item.at("right_neighbor").get<std::string>();

    lanelets_.push_back(std::move(lane));
  }
}

void HDMap::parseJunctions(const nlohmann::json & j)
{
  junctions_.reserve(j.size());

  for (const auto & item : j) {
    Junction junc;
    junc.id = item.at("id").get<int>();
    junc.center_x = item.at("center_x").get<double>();
    junc.center_y = item.at("center_y").get<double>();
    junc.type = item.at("type").get<std::string>();
    junc.connecting_roads = item.at("connecting_roads").get<std::vector<int>>();
    junc.internal_lane_ids = item.at("internal_lane_ids").get<std::vector<std::string>>();

    junctions_.push_back(std::move(junc));
  }
}

void HDMap::buildLookupTables()
{
  lanelet_index_.reserve(lanelets_.size());
  junction_index_.reserve(junctions_.size());

  for (size_t i = 0; i < lanelets_.size(); ++i) {
    lanelet_index_[lanelets_[i].id] = i;
  }

  for (size_t i = 0; i < junctions_.size(); ++i) {
    junction_index_[junctions_[i].id] = i;
  }
}

const std::vector<Lanelet> & HDMap::getAllLanelets() const { return lanelets_; }

const Lanelet * HDMap::getLaneletById(const std::string & id) const
{
  const auto it = lanelet_index_.find(id);
  if (it == lanelet_index_.end()) {
    return nullptr;
  }
  return &lanelets_[it->second];
}

const Lanelet * HDMap::getNearestLanelet(double x, double y) const
{
  const Lanelet * nearest = nullptr;
  double min_dist = std::numeric_limits<double>::max();

  for (const auto & lane : lanelets_) {
    for (const auto & pt : lane.centerline) {
      const double dx = pt.position.x - x;
      const double dy = pt.position.y - y;
      const double dist = dx * dx + dy * dy;

      if (dist < min_dist) {
        min_dist = dist;
        nearest = &lane;
      }
    }
  }
  return nearest;
}

std::vector<const Lanelet *> HDMap::getSuccessors(const std::string & id) const
{
  const Lanelet * lane = getLaneletById(id);
  if (!lane) {
    return {};
  }

  std::vector<const Lanelet *> result;
  result.reserve(lane->successors.size());
  for (const auto & succ_id : lane->successors) {
    const Lanelet * succ = getLaneletById(succ_id);
    if (succ) {
      result.push_back(succ);
    }
  }
  return result;
}

std::vector<const Lanelet *> HDMap::getPredecessors(const std::string & id) const
{
  const Lanelet * lane = getLaneletById(id);
  if (!lane) {
    return {};
  }

  std::vector<const Lanelet *> result;
  result.reserve(lane->predecessors.size());
  for (const auto & pred_id : lane->predecessors) {
    const Lanelet * pred = getLaneletById(pred_id);
    if (pred) {
      result.push_back(pred);
    }
  }
  return result;
}

const std::vector<Junction> & HDMap::getAllJunctions() const { return junctions_; }

const Junction * HDMap::getJunctionById(int junction_id) const
{
  const auto it = junction_index_.find(junction_id);
  if (it == junction_index_.end()) {
    return nullptr;
  }
  return &junctions_[it->second];
}

bool HDMap::isInJunction(const std::string & lanelet_id) const
{
  const Lanelet * lane = getLaneletById(lanelet_id);
  if (!lane) {
    return false;
  }
  return lane->is_junction;
}

}  // namespace hd_map
