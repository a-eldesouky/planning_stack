#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>

#include "hd_map/hd_map.hpp"

using namespace hd_map;

// Test Fixture: loads the map once for all tests
class HDMapTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    std::string share_dir = ament_index_cpp::get_package_share_directory("hd_map");
    std::string map_path = share_dir + "/maps/Town01_hd_map.json";

    map_ = std::make_unique<HDMap>();
    ASSERT_TRUE(map_->load(map_path)) << "Failed to load map from: " << map_path;
  }

  static void TearDownTestSuite() { map_.reset(); }

  static std::unique_ptr<HDMap> map_;
};

std::unique_ptr<HDMap> HDMapTest::map_ = nullptr;

// Test 1: Loading / Metadata
TEST_F(HDMapTest, MapIsLoaded) { EXPECT_TRUE(map_->isLoaded()); }

TEST_F(HDMapTest, MetadataTotalLanelets) { EXPECT_EQ(map_->getMetadata().total_lanelets, 124); }

TEST_F(HDMapTest, MetadataTotalJunctions) { EXPECT_EQ(map_->getMetadata().total_junctions, 12); }

TEST_F(HDMapTest, MetadataMapName)
{
  EXPECT_NE(map_->getMetadata().map_name.find("Town01"), std::string::npos);
}

TEST_F(HDMapTest, MetadataWaypointSpacing)
{
  EXPECT_GT(map_->getMetadata().waypoint_spacing_m, 0.0);
}

// Test 2: Lanelet Queries in ROS Frame
TEST_F(HDMapTest, GetAllLaneletsCount) { EXPECT_EQ(map_->getAllLanelets().size(), 124u); }

TEST_F(HDMapTest, GetLaneletByIdFound)
{
  const Lanelet * l1 = map_->getLaneletById("lanelet_1_-1");
  ASSERT_NE(l1, nullptr);
  EXPECT_EQ(l1->road_id, 1);
  EXPECT_EQ(l1->lane_id, -1);
  EXPECT_FALSE(l1->is_junction);
  EXPECT_GT(l1->length_m, 0.0);
}

TEST_F(HDMapTest, GetLaneletByIdCenterlineNotEmpty)
{
  const Lanelet * l1 = map_->getLaneletById("lanelet_1_-1");
  ASSERT_NE(l1, nullptr);
  EXPECT_FALSE(l1->centerline.empty());
}

TEST_F(HDMapTest, Lanelet1IsNearRosXAxis)
{
  const Lanelet * l1 = map_->getLaneletById("lanelet_1_-1");
  ASSERT_NE(l1, nullptr);
  ASSERT_FALSE(l1->centerline.empty());

  const auto & p0 = l1->centerline.front();

  EXPECT_NEAR(p0.position.y, 2.0, 1.0)
    << "lanelet_1_-1 should be near ROS y ~= +2.0, got " << p0.position.y;

  EXPECT_GE(p0.heading_rad, -M_PI);
  EXPECT_LE(p0.heading_rad, M_PI);
}

TEST_F(HDMapTest, GetLaneletByIdBoundaryMatchesCenterline)
{
  const Lanelet * l1 = map_->getLaneletById("lanelet_1_-1");
  ASSERT_NE(l1, nullptr);
  EXPECT_EQ(l1->left_boundary.size(), l1->centerline.size());
  EXPECT_EQ(l1->right_boundary.size(), l1->centerline.size());
}

TEST_F(HDMapTest, GetLaneletByIdNotFound)
{
  const Lanelet * bad = map_->getLaneletById("lanelet_9999_-1");
  EXPECT_EQ(bad, nullptr);
}

TEST_F(HDMapTest, GetNearestLaneletReturnsResult)
{
  const Lanelet * nearest = map_->getNearestLanelet(200.0, -2.0);
  ASSERT_NE(nearest, nullptr);
}

TEST_F(HDMapTest, LaneletNearRosEgoPose)
{
  // This is the ego pose observed from /carla/ego_vehicle/odometry:
  // x ~= 339, y ~= -314, yaw ~= +1.57.
  const Lanelet * nearest = map_->getNearestLanelet(339.0, -314.0);
  ASSERT_NE(nearest, nullptr);

  // lanelet_19_1 is expected around this area in the ROS-frame map.
  EXPECT_EQ(nearest->id, "lanelet_19_1");
}

// Test 3: Graph Connectivity
TEST_F(HDMapTest, SuccessorsOfLane1)
{
  auto succs = map_->getSuccessors("lanelet_1_-1");
  EXPECT_EQ(succs.size(), 2u);
}

TEST_F(HDMapTest, SuccessorsOfLane25)
{
  auto succs = map_->getSuccessors("lanelet_25_-1");
  EXPECT_EQ(succs.size(), 2u);
}

TEST_F(HDMapTest, PredecessorsOfLane1NotEmpty)
{
  auto preds = map_->getPredecessors("lanelet_1_-1");
  EXPECT_FALSE(preds.empty());
}

TEST_F(HDMapTest, SuccessorsOfNonExistentLane)
{
  auto succs = map_->getSuccessors("lanelet_9999_-1");
  EXPECT_TRUE(succs.empty());
}

// Test 4: Junction Queries
TEST_F(HDMapTest, GetAllJunctionsCount) { EXPECT_EQ(map_->getAllJunctions().size(), 12u); }

TEST_F(HDMapTest, GetJunction332Found)
{
  const Junction * j332 = map_->getJunctionById(332);
  ASSERT_NE(j332, nullptr);
}

TEST_F(HDMapTest, Junction332IsT)
{
  const Junction * j332 = map_->getJunctionById(332);
  ASSERT_NE(j332, nullptr);
  EXPECT_EQ(j332->type, "T-junction");
}

TEST_F(HDMapTest, Junction332Has3ConnectingRoads)
{
  const Junction * j332 = map_->getJunctionById(332);
  ASSERT_NE(j332, nullptr);
  EXPECT_EQ(j332->connecting_roads.size(), 3u);
  EXPECT_LT(j332->center_y, 0.0);
}

TEST_F(HDMapTest, AllJunctionsAreTJunctions)
{
  for (const auto & junc : map_->getAllJunctions()) {
    EXPECT_EQ(junc.type, "T-junction") << "Junction " << junc.id << " is not T-junction";
  }
}

TEST_F(HDMapTest, GetJunctionNotFound)
{
  const Junction * bad = map_->getJunctionById(9999);
  EXPECT_EQ(bad, nullptr);
}

TEST_F(HDMapTest, IsInJunctionFalseForRoad1) { EXPECT_FALSE(map_->isInJunction("lanelet_1_-1")); }

// Main
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
