#include <filesystem>
#include <fstream>

#include <gtest/gtest.h>

#include "agribot_hardware_bringup/map_georeference.hpp"

namespace
{

namespace navsat = agribot_hardware_bringup::navsat;

TEST(MapGeoreference, RoundTripsAndFingerprintsMap)
{
  const std::filesystem::path directory =
    std::filesystem::temp_directory_path() / "agribot_map_georeference_test";
  std::filesystem::remove_all(directory);
  std::filesystem::create_directories(directory);
  const std::filesystem::path map_path = directory / "corridor.pcd";
  const std::filesystem::path yaml_path = directory / "corridor_georeference.yaml";
  {
    std::ofstream map(map_path, std::ios::binary);
    map << "test-pcd-payload";
  }

  navsat::MapGeoreference source;
  source.map_id = "corridor";
  source.map_pcd = map_path.string();
  source.map_fingerprint = navsat::fingerprintFile(map_path);
  source.reference_latitude_deg = 30.123456789;
  source.reference_longitude_deg = 114.123456789;
  source.reference_altitude_m = 42.5;
  source.map_from_enu_xyz = {10.0, -3.0, 0.5};
  source.map_from_enu_rpy = {0.0, 0.0, 0.25};
  source.horizontal_rmse_m = 0.08;
  source.yaw_rmse_deg = 0.7;
  source.sample_count = 120U;
  source.source_bag = "/bags/corridor";
  source.calibration_version = "test-v1";
  source.calibration_hash = navsat::fnv1a64Text("calibration");
  source.created_at_utc = "2026-08-04T00:00:00Z";

  navsat::writeMapGeoreference(yaml_path, source);
  const auto loaded = navsat::loadMapGeoreference(yaml_path);

  EXPECT_EQ(loaded.map_id, source.map_id);
  EXPECT_EQ(loaded.map_fingerprint, source.map_fingerprint);
  EXPECT_DOUBLE_EQ(loaded.reference_latitude_deg, source.reference_latitude_deg);
  EXPECT_DOUBLE_EQ(loaded.map_from_enu_xyz[1], source.map_from_enu_xyz[1]);
  EXPECT_EQ(loaded.sample_count, source.sample_count);
  const Eigen::Isometry3d transform = navsat::mapFromEnuTransform(loaded);
  EXPECT_NEAR(transform.translation().x(), 10.0, 1.0e-12);
  EXPECT_NEAR(std::atan2(transform.linear()(1, 0), transform.linear()(0, 0)), 0.25, 1.0e-12);
  auto non_planar = loaded;
  non_planar.map_from_enu_rpy[0] = 0.1;
  EXPECT_THROW(navsat::validateMapGeoreference(non_planar), std::runtime_error);
  auto missing_provenance = loaded;
  missing_provenance.calibration_hash.clear();
  EXPECT_THROW(navsat::validateMapGeoreference(missing_provenance), std::runtime_error);
  EXPECT_FALSE(std::filesystem::exists(yaml_path.string() + ".tmp"));
  std::filesystem::remove_all(directory);
}

TEST(MapGeoreference, DetectsMapContentChanges)
{
  const std::filesystem::path path =
    std::filesystem::temp_directory_path() / "agribot_fingerprint_test.pcd";
  {
    std::ofstream stream(path, std::ios::binary);
    stream << "first";
  }
  const std::string first = navsat::fingerprintFile(path);
  {
    std::ofstream stream(path, std::ios::binary | std::ios::trunc);
    stream << "second";
  }
  EXPECT_NE(navsat::fingerprintFile(path), first);
  std::filesystem::remove(path);
}

}  // namespace
