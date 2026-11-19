#pragma once
#include <array>
#include <vector>

namespace vista::models {

using RoomLayoutPosition = std::array<double, 3>;
enum class RoomLayoutFloor { unknown, measured, estimated };

struct RoomLayoutCorner {
    RoomLayoutPosition position{};
    bool inferred{};
};
struct RoomLayoutSegment {
    RoomLayoutPosition start{}, end{};
    bool inferred{};
};
struct RoomLayout {
    std::vector<RoomLayoutCorner> corners;
    std::vector<RoomLayoutSegment> segments;
    RoomLayoutFloor floor{RoomLayoutFloor::unknown};
    bool closed{};
};

} // namespace vista::models
