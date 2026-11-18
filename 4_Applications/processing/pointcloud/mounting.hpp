#pragma once

#include "models/mapping/lidar_pose.hpp"
#include <cmath>

namespace vista::application {

/// Initial LiDAR world pose for LIO, or the fixed sensor-to-world transform
/// without LIO. Independent of floor detection and LiDAR-to-IMU calibration.
struct MountingConfig {
    float mount_x_m{};
    float mount_y_m{};
    float mount_z_m{};
    float mount_roll_deg{};
    float mount_pitch_deg{};
    float mount_yaw_deg{};
};

inline models::LidarPose pose_from_mount(const MountingConfig& mount) {
    constexpr double half_rad = 3.14159265358979323846 / 360.0;
    const auto sr = std::sin(mount.mount_roll_deg * half_rad);
    const auto cr = std::cos(mount.mount_roll_deg * half_rad);
    const auto sp = std::sin(mount.mount_pitch_deg * half_rad);
    const auto cp = std::cos(mount.mount_pitch_deg * half_rad);
    const auto sy = std::sin(mount.mount_yaw_deg * half_rad);
    const auto cy = std::cos(mount.mount_yaw_deg * half_rad);
    return {{mount.mount_x_m,mount.mount_y_m,mount.mount_z_m},
            {sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy,
             cr*cp*sy - sr*sp*cy, cr*cp*cy + sr*sp*sy}};
}

} // namespace vista::application
