#pragma once

#include "4_Applications/processing/pointcloud/mounting.hpp"
#include <cmath>
#include <cstdint>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace vista::application {

struct MappingPose {
    MountingConfig mount;
    bool lio_enabled{};
    std::uint64_t generation{};
};

// A single pending command serializes operators. Only the localization worker
// can acknowledge it, after the old engine and synchronization state are gone.
class MappingPoseControl {
public:
    MappingPoseControl(MappingPose initial, std::string token)
        : active_(initial), token_(std::move(token)) {}

    static bool valid_token(std::string_view token) {
        if (token.size() < 24 || token.size() > 128) return false;
        return token.find_first_not_of(
            "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_-") == std::string_view::npos;
    }

    struct Status { MappingPose active; bool pending; };

    Status status() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return {active_, pending_.has_value()};
    }

    std::uint64_t generation() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return active_.generation;
    }

    void request(std::string_view token, std::uint64_t expected_generation,
                 std::optional<MountingConfig> mount, std::optional<bool> lio_enabled) {
        if (token.size() != token_.size()) throw std::invalid_argument("Invalid control token");
        unsigned difference = 0;
        for (std::size_t i = 0; i < token.size(); ++i) difference |= token[i] ^ token_[i];
        if (difference != 0) throw std::invalid_argument("Invalid control token");
        if (mount) {
            const float position[]{mount->mount_x_m, mount->mount_y_m, mount->mount_z_m};
            const float rotation[]{mount->mount_roll_deg, mount->mount_pitch_deg, mount->mount_yaw_deg};
            for (const auto value : position)
                if (!std::isfinite(value) || std::abs(value) > 1000)
                    throw std::invalid_argument("Mount position must be between -1000 and 1000 m");
            for (const auto value : rotation)
                if (!std::isfinite(value) || std::abs(value) > 360)
                    throw std::invalid_argument("Mount rotation must be between -360 and 360 degrees");
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (suspended_) throw std::invalid_argument("Advanced settings are being applied");
        if (pending_) throw std::invalid_argument("A pose reset is already pending");
        if (expected_generation != active_.generation)
            throw std::invalid_argument("Active pose changed; review the current settings");
        pending_ = MappingPose{mount.value_or(active_.mount), lio_enabled.value_or(active_.lio_enabled),
                               active_.generation + 1};
    }

    bool suspend_if_idle() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (pending_) return false;
        suspended_ = true;
        return true;
    }

    void resume() {
        std::lock_guard<std::mutex> lock(mutex_);
        suspended_ = false;
    }

    std::optional<MappingPose> pending() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return pending_;
    }

    void acknowledge(std::uint64_t generation) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (pending_ && pending_->generation == generation) {
            active_ = *pending_;
            pending_.reset();
        }
    }

private:
    mutable std::mutex mutex_;
    MappingPose active_;
    const std::string token_;
    std::optional<MappingPose> pending_;
    bool suspended_{};
};

} // namespace vista::application
