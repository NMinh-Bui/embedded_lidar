#pragma once

#include <array>
#include <cstdint>
#include <optional>

namespace vista::models {

struct LidarPose {
    std::array<double,3> position_m{};
    std::array<double,4> orientation_xyzw{0,0,0,1};
    std::optional<std::array<double,3>> world_up{}; // From Point-LIO gravity, when known.
};

enum class LidarPoseState { unavailable, tracking, fixed_mount, last_known };

/// A small visualization stream in the same world frame as the map.
struct LidarPoseStatus {
    LidarPoseState state{LidarPoseState::unavailable};
    std::optional<LidarPose> pose;
    std::uint64_t mapping_generation{};
};

} // namespace vista::models
