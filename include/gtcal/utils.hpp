#pragma once

#include <gtsam/geometry/Point2.h>
#include <limits>

namespace gtcal {
namespace utils {

static constexpr double NaN = std::numeric_limits<double>::quiet_NaN();

static bool FilterPixelCoords(const gtsam::Point2& uv, const size_t image_width, const size_t image_height) {
  if ((uv.x() >= 0.) && (uv.y() >= 0.) && (uv.x() < image_width) && (uv.y() < image_height)) {
    return true;
  }
  return false;
}

static double DegToRad(const double degrees) { return degrees * M_PI / 180.0; }

static double RadToDeg(const double radians) { return radians * 180.0 / M_PI; }

}  // namespace utils
}  // namespace gtcal