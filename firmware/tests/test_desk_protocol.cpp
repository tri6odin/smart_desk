#include "desk_protocol.hpp"

#include <array>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace {

int failures = 0;

#define CHECK(condition)                                                       \
    do {                                                                       \
        if (!(condition)) {                                                    \
            std::cerr << __FILE__ << ':' << __LINE__                           \
                      << ": CHECK failed: " #condition << '\n';               \
            ++failures;                                                        \
        }                                                                      \
    } while (false)

void test_known_command_frames() {
    using smart_desk::Command;
    using smart_desk::Frame;

    struct Case {
        Command command;
        Frame expected;
    };

    const std::array<Case, 10> cases{{
        {Command::kIdle,        {0xAA, 0x55, 0x08, 0x01, 0x00, 0x00, 0x01, 0x08}},
        {Command::kRecallStand, {0xAA, 0x55, 0x08, 0x01, 0x00, 0x01, 0x01, 0x09}},
        {Command::kRecallSit,   {0xAA, 0x55, 0x08, 0x01, 0x00, 0x02, 0x01, 0x0A}},
        {Command::kSaveStand,   {0xAA, 0x55, 0x08, 0x01, 0x00, 0x03, 0x01, 0x0B}},
        {Command::kSaveSit,     {0xAA, 0x55, 0x08, 0x01, 0x00, 0x04, 0x01, 0x0C}},
        {Command::kDownStep,    {0xAA, 0x55, 0x08, 0x01, 0x00, 0x05, 0x01, 0x0D}},
        {Command::kDownHold,    {0xAA, 0x55, 0x08, 0x01, 0x00, 0x06, 0x01, 0x0E}},
        {Command::kUpStep,      {0xAA, 0x55, 0x08, 0x01, 0x00, 0x07, 0x01, 0x0F}},
        {Command::kUpHold,      {0xAA, 0x55, 0x08, 0x01, 0x00, 0x08, 0x01, 0x10}},
        {Command::kWake,        {0xAA, 0x55, 0x08, 0x01, 0x00, 0x0E, 0x01, 0x16}},
    }};

    for (const auto& item : cases) {
        const auto frame = smart_desk::make_panel_frame(item.command);
        CHECK(frame == item.expected);
        CHECK(smart_desk::is_valid_frame(
            frame, smart_desk::Direction::kPanelToController));
        const auto decoded = smart_desk::decode_panel_frame(frame);
        CHECK(decoded.has_value());
        CHECK(decoded->known_command().has_value());
        CHECK(decoded->known_command().value() == item.command);
    }
}

void test_height_and_flags() {
    const smart_desk::Frame normal{0x55, 0xAA, 0x08, 0x02,
                                   0x03, 0x21, 0x01, 0x2D};
    const smart_desk::Frame save_ack{0x55, 0xAA, 0x08, 0x02,
                                     0x43, 0x21, 0x01, 0x6D};

    const auto normal_message = smart_desk::decode_controller_frame(normal);
    CHECK(normal_message.has_value());
    CHECK(normal_message->height_tenths_cm == 801);
    CHECK(normal_message->flags == 0x00);

    const auto save_message = smart_desk::decode_controller_frame(save_ack);
    CHECK(save_message.has_value());
    CHECK(save_message->height_tenths_cm == 801);
    CHECK(save_message->flags == 0x40);
}

void test_startup_frame_is_valid_but_not_height() {
    const smart_desk::Frame startup{0x55, 0xAA, 0x08, 0x0F,
                                    0x00, 0x0F, 0x01, 0x25};
    CHECK(smart_desk::is_valid_frame(
        startup, smart_desk::Direction::kControllerToPanel));
    CHECK(!smart_desk::decode_controller_frame(startup).has_value());
}

void test_bad_checksum_is_rejected() {
    auto frame = smart_desk::make_panel_frame(smart_desk::Command::kIdle);
    frame[7] ^= 0x80;
    CHECK(!smart_desk::is_valid_frame(
        frame, smart_desk::Direction::kPanelToController));
    CHECK(!smart_desk::decode_panel_frame(frame).has_value());
}

void test_stream_parser_recovers_after_noise() {
    smart_desk::StreamParser parser(
        smart_desk::Direction::kPanelToController);
    auto bad = smart_desk::make_panel_frame(smart_desk::Command::kIdle);
    bad[7] = 0xF8;
    const auto good = smart_desk::make_panel_frame(smart_desk::Command::kUpHold);

    std::vector<std::uint8_t> input{0x00, 0xAA, 0x12, 0xFE};
    input.insert(input.end(), bad.begin(), bad.end());
    input.insert(input.end(), good.begin(), good.end());

    std::optional<smart_desk::Frame> result;
    for (const auto byte : input) {
        const auto parsed = parser.feed(byte);
        if (parsed.has_value()) {
            result = parsed;
        }
    }

    CHECK(result.has_value());
    CHECK(result.value() == good);
}

void test_unknown_command_is_preserved() {
    auto frame = smart_desk::make_panel_frame(
        static_cast<smart_desk::Command>(0x7A));
    const auto decoded = smart_desk::decode_panel_frame(frame);
    CHECK(decoded.has_value());
    CHECK(decoded->command_code == 0x7A);
    CHECK(!decoded->known_command().has_value());
    CHECK(std::string(smart_desk::command_name(0x7A)) == "UNKNOWN");
}

}  // namespace

int main() {
    test_known_command_frames();
    test_height_and_flags();
    test_startup_frame_is_valid_but_not_height();
    test_bad_checksum_is_rejected();
    test_stream_parser_recovers_after_noise();
    test_unknown_command_is_preserved();

    if (failures != 0) {
        std::cerr << failures << " test(s) failed\n";
        return EXIT_FAILURE;
    }

    std::cout << "all desk protocol tests passed\n";
    return EXIT_SUCCESS;
}
