#pragma once

#include "desk_controller.hpp"

namespace smart_desk {

struct DeskPresets {
    std::uint16_t sit = 800;
    std::uint16_t stand = 1150;
};

DeskPresets load_presets(const ControllerConfig& config);
bool save_presets(const DeskPresets& presets, const ControllerConfig& config);
bool load_recovery_lock();
bool save_recovery_lock(bool locked);

}  // namespace smart_desk
