#pragma once

#include <gtsam/geometry/Point2.h>

namespace gtcal {
struct Measurement {
  const gtsam::Point2 uv;  // Measurement in pixel coordinates.
  const size_t camera_id;  // Camera id.
  const size_t point_id;   // Target point id.

  Measurement(const gtsam::Point2& uv, const size_t camera_id, const size_t point_id)
    : uv(uv), camera_id(camera_id), point_id(point_id) {}
};

}  // namespace gtcal