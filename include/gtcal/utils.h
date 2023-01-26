#pragma once

#include <gtsam/geometry/Point2.h>
#include <limits>

namespace gtcal {
namespace utils {
static constexpr double NaN = std::numeric_limits<double>::quiet_NaN();

static bool FilterPixelCoords(const gtsam::Point2& uv, const int image_width, const int image_height) {
  if ((uv.x() >= 0) && (uv.y() >= 0) && (uv.x() < image_width) && (uv.y() < image_height)) {
    return true;
  }
  return false;
}

template <typename T>
T DegToRad(const T degrees) {
  return degrees * T(M_PI / 180.0);
}

template <typename T>
T RadToDeg(const T radians) {
  return radians * T(180.0 / M_PI);
}

}  // namespace utils
}  // namespace gtcal