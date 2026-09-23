#pragma once

#include "desk_controller.hpp"

namespace smart_desk {

struct DeskPresets {
    std::uint16_t sit = 800;
    std::uint16_t stand = 1150;
};

DeskPresets load_presets(const ControllerConfig& config);
bool save_presets(const DeskPresets& presets, const ControllerConfig& config);

}  // namespace smart_desk
