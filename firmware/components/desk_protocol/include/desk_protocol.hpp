#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace smart_desk {

constexpr std::size_t kFrameSize = 8;
using Frame = std::array<std::uint8_t, kFrameSize>;

// Observed twice when the desk supply was switched off. This diagnostic
// name does not relax the unknown-frame recovery policy.
inline constexpr Frame kObservedPowerOffFrame{0x55, 0xAA, 0x08, 0x03,
                                              0x08, 0x00, 0x01, 0x12};

enum class Direction {
    kPanelToController,
    kControllerToPanel,
};

enum class ControllerFrameKind {
    kHeight,
    kStartup,
    kReset,
    kUnknown,
};

enum class Command : std::uint8_t {
    kIdle = 0x00,
    kRecallStand = 0x01,
    kRecallSit = 0x02,
    kSaveStand = 0x03,
    kSaveSit = 0x04,
    kDownStep = 0x05,
    kDownHold = 0x06,
    kUpStep = 0x07,
    kUpHold = 0x08,
    kWake = 0x0E,
};

struct PanelMessage {
    std::uint8_t command_code;
    std::uint8_t status;

    std::optional<Command> known_command() const;
};

struct ControllerMessage {
    // Высота в десятых долях сантиметра: 801 означает 80,1 см.
    std::uint16_t height_tenths_cm;
    // Два старших бита byte[4] не входят в высоту. Наблюдавшийся 0x40
    // подтверждает сохранение пресета.
    std::uint8_t flags;
    std::uint8_t status;
};

std::uint8_t calculate_checksum(const Frame& frame);
bool is_valid_frame(const Frame& frame, Direction direction);
Frame make_panel_frame(Command command, std::uint8_t status = 0x01);
std::optional<PanelMessage> decode_panel_frame(const Frame& frame);
std::optional<ControllerMessage> decode_controller_frame(const Frame& frame);
ControllerFrameKind classify_controller_frame(const Frame& frame);
const char* command_name(std::uint8_t command_code);

// Потоковый парсер принимает по одному байту, ищет заголовок направления,
// проверяет длину и checksum и восстанавливает синхронизацию после мусора.
class StreamParser {
public:
    explicit StreamParser(Direction direction);

    std::optional<Frame> feed(std::uint8_t byte);
    // Diagnostic candidate rejected by framing/checksum validation. This
    // does not change parser resynchronization or controller semantics.
    std::optional<Frame> take_rejected();
    void reset();

private:
    void resync_after_invalid_frame();

    Direction direction_;
    Frame buffer_{};
    std::size_t used_{0};
    std::optional<Frame> rejected_;
};

}  // namespace smart_desk
