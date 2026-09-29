#include "desk_protocol.hpp"

#include <algorithm>

namespace smart_desk {
namespace {

constexpr std::array<std::uint8_t, 2> header_for(Direction direction) {
    return direction == Direction::kPanelToController
               ? std::array<std::uint8_t, 2>{0xAA, 0x55}
               : std::array<std::uint8_t, 2>{0x55, 0xAA};
}

bool is_known_command_code(std::uint8_t code) {
    switch (code) {
        case 0x00:
        case 0x01:
        case 0x02:
        case 0x03:
        case 0x04:
        case 0x05:
        case 0x06:
        case 0x07:
        case 0x08:
        case 0x0E:
            return true;
        default:
            return false;
    }
}

}  // namespace

std::optional<Command> PanelMessage::known_command() const {
    if (!is_known_command_code(command_code)) {
        return std::nullopt;
    }
    return static_cast<Command>(command_code);
}

std::uint8_t calculate_checksum(const Frame& frame) {
    const std::uint16_t sum = static_cast<std::uint16_t>(frame[2]) + frame[3] +
                              frame[4] + frame[5];
    return static_cast<std::uint8_t>((sum - 1U) & 0xFFU);
}

bool is_valid_frame(const Frame& frame, Direction direction) {
    const auto header = header_for(direction);
    return frame[0] == header[0] && frame[1] == header[1] &&
           frame[2] == kFrameSize && frame[7] == calculate_checksum(frame);
}

Frame make_panel_frame(Command command, std::uint8_t status) {
    Frame frame{0xAA, 0x55, 0x08, 0x01, 0x00,
                static_cast<std::uint8_t>(command), status, 0x00};
    frame[7] = calculate_checksum(frame);
    return frame;
}

std::optional<PanelMessage> decode_panel_frame(const Frame& frame) {
    if (!is_valid_frame(frame, Direction::kPanelToController) ||
        frame[3] != 0x01) {
        return std::nullopt;
    }
    return PanelMessage{frame[5], frame[6]};
}

std::optional<ControllerMessage> decode_controller_frame(const Frame& frame) {
    if (!is_valid_frame(frame, Direction::kControllerToPanel) ||
        frame[3] != 0x02) {
        return std::nullopt;
    }

    constexpr std::uint8_t kHeightMask = 0x3F;
    constexpr std::uint8_t kFlagMask = 0xC0;
    const auto height = static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(frame[4] & kHeightMask) << 8U) | frame[5]);
    return ControllerMessage{height,
                             static_cast<std::uint8_t>(frame[4] & kFlagMask),
                             frame[6]};
}

ControllerFrameKind classify_controller_frame(const Frame& frame) {
    if (!is_valid_frame(frame, Direction::kControllerToPanel)) {
        return ControllerFrameKind::kUnknown;
    }
    if (frame[3] == 0x02) {
        return ControllerFrameKind::kHeight;
    }
    constexpr Frame kStartupFrame{0x55, 0xAA, 0x08, 0x0F,
                                  0x00, 0x0F, 0x01, 0x25};
    constexpr Frame kResetFrame{0x55, 0xAA, 0x08, 0x04,
                                0x00, 0x01, 0x01, 0x0C};
    if (frame == kStartupFrame) {
        return ControllerFrameKind::kStartup;
    }
    if (frame == kResetFrame) {
        return ControllerFrameKind::kReset;
    }
    return ControllerFrameKind::kUnknown;
}

const char* command_name(std::uint8_t command_code) {
    switch (command_code) {
        case 0x00: return "IDLE";
        case 0x01: return "RECALL_STAND";
        case 0x02: return "RECALL_SIT";
        case 0x03: return "SAVE_STAND";
        case 0x04: return "SAVE_SIT";
        case 0x05: return "DOWN_STEP";
        case 0x06: return "DOWN_HOLD";
        case 0x07: return "UP_STEP";
        case 0x08: return "UP_HOLD";
        case 0x0E: return "WAKE";
        default: return "UNKNOWN";
    }
}

StreamParser::StreamParser(Direction direction) : direction_(direction) {}

void StreamParser::reset() {
    used_ = 0;
    rejected_.reset();
}

std::optional<Frame> StreamParser::take_rejected() {
    const auto result = rejected_;
    rejected_.reset();
    return result;
}

std::optional<Frame> StreamParser::feed(std::uint8_t byte) {
    const auto header = header_for(direction_);

    if (used_ == 0) {
        if (byte == header[0]) {
            buffer_[0] = byte;
            used_ = 1;
        }
        return std::nullopt;
    }

    if (used_ == 1) {
        if (byte == header[1]) {
            buffer_[1] = byte;
            used_ = 2;
        } else if (byte == header[0]) {
            buffer_[0] = byte;
        } else {
            used_ = 0;
        }
        return std::nullopt;
    }

    buffer_[used_++] = byte;
    if (used_ < kFrameSize) {
        return std::nullopt;
    }

    if (is_valid_frame(buffer_, direction_)) {
        const Frame result = buffer_;
        used_ = 0;
        return result;
    }

    rejected_ = buffer_;
    resync_after_invalid_frame();
    return std::nullopt;
}

void StreamParser::resync_after_invalid_frame() {
    const auto header = header_for(direction_);

    for (std::size_t start = 1; start < kFrameSize; ++start) {
        if (buffer_[start] != header[0]) {
            continue;
        }

        if (start + 1 < kFrameSize && buffer_[start + 1] != header[1]) {
            continue;
        }

        const std::size_t remaining = kFrameSize - start;
        std::move(buffer_.begin() + static_cast<std::ptrdiff_t>(start),
                  buffer_.end(), buffer_.begin());
        used_ = remaining;
        return;
    }

    used_ = 0;
}

}  // namespace smart_desk
