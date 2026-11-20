#pragma once

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>

// The upstream fixed matrix is rounded to six decimals. Its orthogonality
// error is 6.15e-7, so this tolerance accepts the unchanged upstream default.
inline Eigen::Matrix3d validate_calibration_rotation(
    const std::vector<double>& values, const char* parameter_name) {
  if (values.size() != 9) {
    throw std::invalid_argument(std::string(parameter_name) + " requires 9 row-major values");
  }
  Eigen::Matrix3d rotation;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      const double value = values[3 * row + col];
      if (!std::isfinite(value)) {
        throw std::invalid_argument(std::string(parameter_name) + " must be finite");
      }
      rotation(row, col) = value;
    }
  }
  constexpr double tolerance = 1e-6;
  if ((rotation.transpose() * rotation - Eigen::Matrix3d::Identity())
          .cwiseAbs().maxCoeff() > tolerance ||
      std::abs(rotation.determinant() - 1.0) > tolerance) {
    throw std::invalid_argument(
        std::string(parameter_name) + " must be orthogonal with determinant +1");
  }
  return rotation;
}

inline Eigen::Matrix3d validate_zg_fixed_rotation(const std::vector<double>& values) {
  return validate_calibration_rotation(values, "rigelslam_rot.zg_fixed_R");
}

inline Eigen::Vector3d validate_calibration_translation(
    const std::vector<double>& values, const char* parameter_name) {
  if (values.size() != 3) {
    throw std::invalid_argument(std::string(parameter_name) + " requires 3 values");
  }
  for (double value : values) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument(std::string(parameter_name) + " must be finite");
    }
  }
  return Eigen::Vector3d(values[0], values[1], values[2]);
}

inline Eigen::Vector3d validate_zg_fixed_translation(const std::vector<double>& values) {
  return validate_calibration_translation(values, "rigelslam_rot.zg_fixed_T_m");
}

inline double validate_imu_wrt_lidar_lag_s(double value) {
  if (!std::isfinite(value)) {
    throw std::invalid_argument("rigelslam_rot.imu_wrt_lidar_lag_s must be finite");
  }
  return value;
}
