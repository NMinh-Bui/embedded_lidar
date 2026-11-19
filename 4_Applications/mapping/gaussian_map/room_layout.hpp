#pragma once
#include "models/mapping/room_layout.hpp"
#include <functional>
#include <optional>

namespace vista::application {

// Planar evidence already computed while fitting Gaussian surfels.
struct RoomLayoutSample {
    std::array<float, 3> position{}, normal{};
};

struct RoomLayoutReference {
    models::RoomLayoutPosition lidar_position{};
    models::RoomLayoutPosition world_up{0, 0, 1};
    // World point on the floor datum, when known from Mount. Measured floor wins.
    std::optional<models::RoomLayoutPosition> floor_datum;
};

models::RoomLayout estimate_room_layout(const std::vector<RoomLayoutSample>& samples,
    const RoomLayoutReference& reference, double voxel_size_m,
    const std::function<bool()>& cancelled = {});

} // namespace vista::application
