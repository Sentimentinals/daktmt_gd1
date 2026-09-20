#pragma once

#include <stdint.h>

namespace terrain_model {

// Replaced by `python -m tools.tof_tinyml train` after collecting real ToF data.
constexpr bool kReady = false;
constexpr uint8_t kFeatureCount = 67;
constexpr uint8_t kPrototypeCount = 1;
constexpr int16_t kPrototypes[kPrototypeCount][kFeatureCount] = {{0}};
constexpr uint8_t kPrototypeLabels[kPrototypeCount] = {0};
constexpr uint32_t kPrototypeThresholds[kPrototypeCount] = {1};

}  // namespace terrain_model
