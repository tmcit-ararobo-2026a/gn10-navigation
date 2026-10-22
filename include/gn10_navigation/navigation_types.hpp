#pragma once

#include <cstdint>
#include <string>

namespace gn10_navigation {

enum class TargetId : uint8_t {
    StartZone    = 0,
    FixedBucket1 = 1,
    FixedBucket2 = 2,
    FixedBucket3 = 3,
    Desk1        = 4,
    Desk2        = 5,
    Flag         = 6,
    MovingBucket = 7
};

enum class NavigationStatus : uint8_t {
    Stanby   = 0,
    Ready    = 1,
    Moving   = 2,
    Tracking = 3,
    Goal     = 4,
    Fail     = 5
};

enum class NavigationCommand : uint8_t { Sleep = 0, Stanby = 1, Move = 2 };

inline const char* toString(TargetId id)
{
    switch (id) {
        case TargetId::StartZone:
            return "StartZone";
        case TargetId::FixedBucket1:
            return "FixedBucket1";
        case TargetId::FixedBucket2:
            return "FixedBucket2";
        case TargetId::FixedBucket3:
            return "FixedBucket3";
        case TargetId::Desk1:
            return "Desk1";
        case TargetId::Desk2:
            return "Desk2";
        case TargetId::Flag:
            return "Flag";
        case TargetId::MovingBucket:
            return "MovingBucket";
        default:
            return "Unknown";
    }
}

inline const char* toString(NavigationStatus status)
{
    switch (status) {
        case NavigationStatus::Stanby:
            return "Stanby";
        case NavigationStatus::Ready:
            return "Ready";
        case NavigationStatus::Moving:
            return "Moving";
        case NavigationStatus::Tracking:
            return "Tracking";
        case NavigationStatus::Goal:
            return "Goal";
        case NavigationStatus::Fail:
            return "Fail";
        default:
            return "Unknown";
    }
}

inline const char* toString(NavigationCommand cmd)
{
    switch (cmd) {
        case NavigationCommand::Sleep:
            return "Sleep";
        case NavigationCommand::Stanby:
            return "Stanby";
        case NavigationCommand::Move:
            return "Move";
        default:
            return "Unknown";
    }
}

}  // namespace gn10_navigation
