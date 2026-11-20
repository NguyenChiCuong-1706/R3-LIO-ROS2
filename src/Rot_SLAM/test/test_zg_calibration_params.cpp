#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "zg_calibration_params.hpp"

void require(bool condition) {
  if (!condition) {
    throw std::runtime_error("calibration regression check failed");
  }
}

template <typename Function>
void rejects(Function&& function) {
  try {
    function();
  } catch (const std::invalid_argument&) {
    return;
  }
  throw std::runtime_error("invalid calibration was accepted");
}

int main() {
  const std::vector<double> upstream_rotation = {
      0.0, 0.939693, 0.34202, -1.0, 0.0, 0.0,
      0.0, -0.34202, 0.939693};
  const auto R = validate_zg_fixed_rotation(upstream_rotation);
  require(R(0, 1) == 0.939693 && R(2, 1) == -0.34202);
  const auto T = validate_zg_fixed_translation({0.122378, 0.033, 0.12356});
  require(T.x() == 0.122378 && T.y() == 0.033 && T.z() == 0.12356);
  require(validate_imu_wrt_lidar_lag_s(0.0) == 0.0);
  require(validate_imu_wrt_lidar_lag_s(-0.002) == -0.002);
  const std::vector<double> li_rotation = {
      0.036100, -0.011517, 0.999282,
      -0.997294, -0.064489, 0.035285,
      0.064036, -0.997852, -0.013814};
  const auto filter_R = validate_calibration_rotation(
      li_rotation, "rigelslam_rot.extrinsic_Ril");
  const auto filter_T = validate_calibration_translation(
      {0.013144, 0.000767, 0.029840}, "rigelslam_rot.extrinsic_til");
  require(filter_R(0, 2) == 0.999282 && filter_T.z() == 0.029840);

  rejects([] { validate_zg_fixed_rotation(std::vector<double>(8, 0.0)); });
  rejects([] { validate_zg_fixed_translation({1.0, 2.0}); });
  rejects([] { validate_zg_fixed_rotation({1, 0, 0, 0, 1, 0, 0, 0, -1}); });
  rejects([] { validate_zg_fixed_rotation({1, 0, 0, 0, 1, 0, 0, 0, 2}); });
  rejects([] { validate_zg_fixed_rotation({1, 0, 0, 0, 1, 0, 0, 0,
                                           std::numeric_limits<double>::infinity()}); });
  rejects([] { validate_zg_fixed_translation({0, std::nan(""), 0}); });
  rejects([] { validate_imu_wrt_lidar_lag_s(std::nan("")); });
  rejects([] { validate_calibration_rotation(std::vector<double>(8, 0.0),
                                               "rigelslam_rot.extrinsic_Ril"); });
  rejects([] { validate_calibration_rotation({1, 0, 0, 0, 1, 0, 0, 0, -1},
                                               "rigelslam_rot.extrinsic_Ril"); });
  rejects([] { validate_calibration_translation({0, std::nan(""), 0},
                                                  "rigelslam_rot.extrinsic_til"); });
}
