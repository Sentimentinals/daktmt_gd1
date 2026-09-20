#pragma once

#include <stdint.h>

#include "terrain_model_data.h"

namespace terrain_tinyml {

enum class Label : uint8_t {
  Clear = 0,
  Obstacle = 1,
  StairUp = 2,
  StairDown = 3,
  Unknown = 255,
};

struct Prediction {
  Label label;
  float confidence;
};

inline uint16_t sanitizeDistance(int32_t value) {
  return value >= 20 && value <= 4000 ? value : 4000;
}

inline void insertionSort(uint16_t *values, uint8_t size) {
  for (uint8_t i = 1; i < size; ++i) {
    const uint16_t value = values[i];
    int8_t j = static_cast<int8_t>(i) - 1;
    while (j >= 0 && values[j] > value) {
      values[j + 1] = values[j];
      --j;
    }
    values[j + 1] = value;
  }
}

inline uint16_t median(uint16_t *values, uint8_t size) {
  insertionSort(values, size);
  return size % 2 == 0
             ? static_cast<uint16_t>((values[size / 2 - 1] + values[size / 2]) / 2)
             : values[size / 2];
}

inline int16_t clampFeature(int32_t value) {
  if (value < -100) return -100;
  if (value > 100) return 100;
  return static_cast<int16_t>(value);
}

template <typename Distance>
inline void extractFeatures(const Distance *distances, int16_t *features) {
  uint16_t sorted[64];
  uint16_t minimum = 4000;
  for (uint8_t i = 0; i < 64; ++i) {
    sorted[i] = sanitizeDistance(distances[i]);
    if (sorted[i] < minimum) minimum = sorted[i];
  }
  const uint16_t frame_median = median(sorted, 64);

  for (uint8_t i = 0; i < 64; ++i) {
    const int32_t delta = static_cast<int32_t>(sanitizeDistance(distances[i])) - frame_median;
    features[i] = clampFeature(delta / 20);
  }
  features[64] = clampFeature(static_cast<int32_t>(frame_median / 20) - 100);
  features[65] = clampFeature(static_cast<int32_t>(minimum / 20) - 100);

  uint16_t min_row = 4000;
  uint16_t max_row = 0;
  for (uint8_t row = 0; row < 8; ++row) {
    uint16_t row_values[8];
    for (uint8_t col = 0; col < 8; ++col) {
      row_values[col] = sanitizeDistance(distances[row * 8 + col]);
    }
    const uint16_t row_median = median(row_values, 8);
    if (row_median < min_row) min_row = row_median;
    if (row_median > max_row) max_row = row_median;
  }
  features[66] = clampFeature(static_cast<int32_t>((max_row - min_row) / 20));
}

template <typename Distance>
inline Prediction predict(const Distance *distances) {
  if (!terrain_model::kReady) return {Label::Unknown, 0.0f};

  int16_t features[terrain_model::kFeatureCount];
  extractFeatures(distances, features);
  uint32_t best_distance = UINT32_MAX;
  uint8_t best_index = 0;

  for (uint8_t prototype = 0; prototype < terrain_model::kPrototypeCount; ++prototype) {
    uint32_t distance = 0;
    for (uint8_t feature = 0; feature < terrain_model::kFeatureCount; ++feature) {
      const int32_t delta = static_cast<int32_t>(features[feature]) -
                            terrain_model::kPrototypes[prototype][feature];
      distance += static_cast<uint32_t>(delta * delta);
    }
    if (distance < best_distance) {
      best_distance = distance;
      best_index = prototype;
    }
  }

  const uint32_t threshold = terrain_model::kPrototypeThresholds[best_index];
  if (best_distance > threshold) return {Label::Unknown, 0.0f};

  uint32_t second_distance = UINT32_MAX;
  for (uint8_t prototype = 0; prototype < terrain_model::kPrototypeCount; ++prototype) {
    if (terrain_model::kPrototypeLabels[prototype] ==
        terrain_model::kPrototypeLabels[best_index]) {
      continue;
    }
    uint32_t distance = 0;
    for (uint8_t feature = 0; feature < terrain_model::kFeatureCount; ++feature) {
      const int32_t delta = static_cast<int32_t>(features[feature]) -
                            terrain_model::kPrototypes[prototype][feature];
      distance += static_cast<uint32_t>(delta * delta);
    }
    if (distance < second_distance) second_distance = distance;
  }

  const float fit = 1.0f - static_cast<float>(best_distance) / threshold;
  const float margin = second_distance == UINT32_MAX || second_distance == 0
                           ? 0.0f
                           : 1.0f - static_cast<float>(best_distance) / second_distance;
  float confidence = 0.5f * fit + 0.5f * margin;
  if (confidence < 0.0f) confidence = 0.0f;
  if (confidence > 0.99f) confidence = 0.99f;
  return {static_cast<Label>(terrain_model::kPrototypeLabels[best_index]), confidence};
}

inline const char *labelName(Label label) {
  switch (label) {
    case Label::Clear:
      return "clear";
    case Label::Obstacle:
      return "obstacle";
    case Label::StairUp:
      return "stair_up";
    case Label::StairDown:
      return "stair_down";
    default:
      return "unknown";
  }
}

}  // namespace terrain_tinyml
