#ifndef AGRIBOT_HARDWARE_BRINGUP__MAP_GEOREFERENCE_HPP_
#define AGRIBOT_HARDWARE_BRINGUP__MAP_GEOREFERENCE_HPP_

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

namespace agribot_hardware_bringup::navsat
{

struct MapGeoreference
{
  int schema_version{1};
  std::string map_id;
  std::string map_pcd;
  std::string map_fingerprint;
  double reference_latitude_deg{0.0};
  double reference_longitude_deg{0.0};
  double reference_altitude_m{0.0};
  std::array<double, 3> map_from_enu_xyz{0.0, 0.0, 0.0};
  std::array<double, 3> map_from_enu_rpy{0.0, 0.0, 0.0};
  double horizontal_rmse_m{0.0};
  double yaw_rmse_deg{0.0};
  std::size_t sample_count{0U};
  std::string source_bag;
  std::string calibration_version;
  std::string calibration_hash;
  std::string created_at_utc;
};

inline std::string fnv1a64Hex(const std::uint64_t value)
{
  std::ostringstream stream;
  stream << std::hex << std::setfill('0') << std::setw(16) << value;
  return stream.str();
}

inline std::string fnv1a64Text(const std::string & value)
{
  std::uint64_t hash = 14695981039346656037ULL;
  for (const unsigned char byte : value) {
    hash ^= byte;
    hash *= 1099511628211ULL;
  }
  return fnv1a64Hex(hash);
}

inline std::string fingerprintFile(const std::filesystem::path & path)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("could not open map file for fingerprinting: " + path.string());
  }

  std::uint64_t hash = 14695981039346656037ULL;
  std::array<char, 1024U * 1024U> buffer{};
  while (stream) {
    stream.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const auto count = stream.gcount();
    for (std::streamsize index = 0; index < count; ++index) {
      hash ^= static_cast<unsigned char>(buffer[static_cast<std::size_t>(index)]);
      hash *= 1099511628211ULL;
    }
  }
  if (!stream.eof()) {
    throw std::runtime_error("failed while fingerprinting map file: " + path.string());
  }
  return fnv1a64Hex(hash);
}

inline Eigen::Isometry3d mapFromEnuTransform(const MapGeoreference & georeference)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(
    georeference.map_from_enu_xyz[0],
    georeference.map_from_enu_xyz[1],
    georeference.map_from_enu_xyz[2]);
  transform.linear() =
    (
    Eigen::AngleAxisd(georeference.map_from_enu_rpy[2], Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(georeference.map_from_enu_rpy[1], Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(
      georeference.map_from_enu_rpy[0],
      Eigen::Vector3d::UnitX())).toRotationMatrix();
  return transform;
}

inline void validateMapGeoreference(const MapGeoreference & georeference)
{
  const bool finite_reference =
    std::isfinite(georeference.reference_latitude_deg) &&
    std::isfinite(georeference.reference_longitude_deg) &&
    std::isfinite(georeference.reference_altitude_m);
  bool finite_transform = true;
  for (const double value : georeference.map_from_enu_xyz) {
    finite_transform = finite_transform && std::isfinite(value);
  }
  for (const double value : georeference.map_from_enu_rpy) {
    finite_transform = finite_transform && std::isfinite(value);
  }
  if (georeference.schema_version != 1) {
    throw std::runtime_error("unsupported map georeference schema version");
  }
  if (georeference.map_id.empty() || georeference.map_pcd.empty() ||
    georeference.map_fingerprint.empty())
  {
    throw std::runtime_error("map georeference is missing map identity fields");
  }
  if (georeference.calibration_version.empty() || georeference.calibration_hash.empty()) {
    throw std::runtime_error("map georeference is missing calibration provenance");
  }
  if (!finite_reference || std::abs(georeference.reference_latitude_deg) > 90.0 ||
    std::abs(georeference.reference_longitude_deg) > 180.0 || !finite_transform)
  {
    throw std::runtime_error("map georeference contains invalid coordinates");
  }
  if (std::abs(georeference.map_from_enu_rpy[0]) > 1.0e-6 ||
    std::abs(georeference.map_from_enu_rpy[1]) > 1.0e-6)
  {
    throw std::runtime_error("map georeference must be a planar yaw transform");
  }
  if (!std::isfinite(georeference.horizontal_rmse_m) ||
    !std::isfinite(georeference.yaw_rmse_deg) || georeference.horizontal_rmse_m < 0.0 ||
    georeference.yaw_rmse_deg < 0.0 || georeference.sample_count < 2U)
  {
    throw std::runtime_error("map georeference contains invalid calibration metrics");
  }
}

inline MapGeoreference loadMapGeoreference(const std::filesystem::path & path)
{
  const YAML::Node root = YAML::LoadFile(path.string());
  const YAML::Node map = root["map"];
  const YAML::Node reference = root["reference"];
  const YAML::Node transform = root["map_from_enu"];
  const YAML::Node calibration = root["calibration"];

  MapGeoreference result;
  result.schema_version = root["schema_version"].as<int>();
  result.map_id = map["id"].as<std::string>();
  result.map_pcd = map["pcd"].as<std::string>();
  result.map_fingerprint = map["fingerprint_fnv1a64"].as<std::string>();
  result.reference_latitude_deg = reference["latitude_deg"].as<double>();
  result.reference_longitude_deg = reference["longitude_deg"].as<double>();
  result.reference_altitude_m = reference["altitude_m"].as<double>();
  const auto read_vector3 = [](const YAML::Node & node, const std::string & name) {
      if (!node || !node.IsSequence() || node.size() != 3U) {
        throw std::runtime_error(name + " must contain three values");
      }
      return std::array<double, 3>{
      node[0].as<double>(), node[1].as<double>(), node[2].as<double>()};
    };
  result.map_from_enu_xyz = read_vector3(transform["xyz"], "map_from_enu.xyz");
  result.map_from_enu_rpy = read_vector3(transform["rpy"], "map_from_enu.rpy");
  result.horizontal_rmse_m = calibration["horizontal_rmse_m"].as<double>();
  result.yaw_rmse_deg = calibration["yaw_rmse_deg"].as<double>();
  result.sample_count = calibration["sample_count"].as<std::size_t>();
  result.source_bag = calibration["source_bag"].as<std::string>("");
  result.calibration_version = calibration["version"].as<std::string>();
  result.calibration_hash = calibration["hash"].as<std::string>();
  result.created_at_utc = root["created_at_utc"].as<std::string>("");
  validateMapGeoreference(result);
  return result;
}

inline void writeMapGeoreference(
  const std::filesystem::path & path,
  const MapGeoreference & georeference)
{
  validateMapGeoreference(georeference);
  YAML::Emitter output;
  output << YAML::BeginMap;
  output << YAML::Key << "schema_version" << YAML::Value << georeference.schema_version;
  output << YAML::Key << "map" << YAML::Value << YAML::BeginMap;
  output << YAML::Key << "id" << YAML::Value << georeference.map_id;
  output << YAML::Key << "pcd" << YAML::Value << georeference.map_pcd;
  output << YAML::Key << "fingerprint_fnv1a64" << YAML::Value << georeference.map_fingerprint;
  output << YAML::EndMap;
  output << YAML::Key << "reference" << YAML::Value << YAML::BeginMap;
  output << YAML::Key << "latitude_deg" << YAML::Value << georeference.reference_latitude_deg;
  output << YAML::Key << "longitude_deg" << YAML::Value << georeference.reference_longitude_deg;
  output << YAML::Key << "altitude_m" << YAML::Value << georeference.reference_altitude_m;
  output << YAML::EndMap;
  output << YAML::Key << "map_from_enu" << YAML::Value << YAML::BeginMap;
  output << YAML::Key << "xyz" << YAML::Value << YAML::Flow << YAML::BeginSeq;
  for (const double value : georeference.map_from_enu_xyz) {
    output << value;
  }
  output << YAML::EndSeq;
  output << YAML::Key << "rpy" << YAML::Value << YAML::Flow << YAML::BeginSeq;
  for (const double value : georeference.map_from_enu_rpy) {
    output << value;
  }
  output << YAML::EndSeq;
  output << YAML::EndMap;
  output << YAML::Key << "calibration" << YAML::Value << YAML::BeginMap;
  output << YAML::Key << "horizontal_rmse_m" << YAML::Value << georeference.horizontal_rmse_m;
  output << YAML::Key << "yaw_rmse_deg" << YAML::Value << georeference.yaw_rmse_deg;
  output << YAML::Key << "sample_count" << YAML::Value << georeference.sample_count;
  output << YAML::Key << "source_bag" << YAML::Value << georeference.source_bag;
  output << YAML::Key << "version" << YAML::Value << georeference.calibration_version;
  output << YAML::Key << "hash" << YAML::Value << georeference.calibration_hash;
  output << YAML::EndMap;
  output << YAML::Key << "created_at_utc" << YAML::Value << georeference.created_at_utc;
  output << YAML::EndMap;
  if (!output.good()) {
    throw std::runtime_error("could not serialize map georeference");
  }

  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }
  const std::filesystem::path temporary = path.string() + ".tmp";
  {
    std::ofstream stream(temporary);
    if (!stream) {
      throw std::runtime_error("could not create map georeference: " + temporary.string());
    }
    stream << output.c_str() << '\n';
  }
  std::filesystem::rename(temporary, path);
}

}  // namespace agribot_hardware_bringup::navsat

#endif  // AGRIBOT_HARDWARE_BRINGUP__MAP_GEOREFERENCE_HPP_
