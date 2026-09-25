#pragma once

#include <cstdint>

namespace ble_bonds {

void refresh();
bool matches(const std::uint8_t* address);
void connected(const std::uint8_t* address);
void disconnected();
// Preserve the newly authenticated identity and retire older bonds one by
// one. If its stored identity cannot be established, leave all bonds intact.
void authenticated(const std::uint8_t* address);
bool queue_forget_connected();
void process_requests();
void removal_complete(bool success);

}  // namespace ble_bonds
