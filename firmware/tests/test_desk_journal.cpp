#include "desk_journal.hpp"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <iostream>
#include <vector>

using namespace smart_desk;
using namespace smart_desk::journal;
namespace {
int failures = 0;
#define CHECK(x) do { if (!(x)) { std::cerr << __LINE__ << ": " #x "\n"; ++failures; } } while (false)

class MemoryFlash : public Flash {
public:
    std::vector<std::uint8_t> bytes = std::vector<std::uint8_t>(2 * kSectorSize, 0xff);
    std::size_t write_limit = 1000000, erase_limit = kSectorSize;
    unsigned erases = 0;
    bool report_write_error = false;
    std::size_t size() const override { return bytes.size(); }
    bool read(std::size_t at, void* out, std::size_t n) override {
        if (at + n > size()) return false;
        std::memcpy(out, bytes.data() + at, n);
        return true;
    }
    bool write(std::size_t at, const void* in, std::size_t n) override {
        if (at + n > size()) return false;
        const auto* data = static_cast<const std::uint8_t*>(in);
        for (std::size_t i = 0; i < std::min(n, write_limit); ++i) {
            CHECK((bytes[at + i] & data[i]) == data[i]);  // no 0 -> 1 without erase
            bytes[at + i] &= data[i];
        }
        return n <= write_limit && !report_write_error;
    }
    bool erase_sector(std::size_t at) override {
        ++erases;
        CHECK(at % kSectorSize == 0);
        if (at + kSectorSize > size()) return false;
        std::fill_n(bytes.begin() + static_cast<std::ptrdiff_t>(at), erase_limit, 0xff);
        return erase_limit == kSectorSize;
    }
};

Record entry(std::uint32_t sequence, std::uint32_t boot = 1) {
    Record record;
    record.uptime_ms = (1ULL << 32) + sequence;
    record.sequence = sequence;
    record.boot_id = boot;
    record.kind = Kind::kTx;
    record.data = make_panel_frame(Command::kUpHold);
    seal(record);
    return record;
}

std::vector<Record> records(MemoryFlash& flash) {
    std::vector<std::pair<std::uint64_t, std::size_t>> sectors;
    for (std::size_t at = 0; at < flash.size(); at += kSectorSize) {
        SectorHeader header;
        flash.read(at, &header, sizeof(header));
        if (valid(header)) sectors.emplace_back(header.generation, at);
    }
    std::sort(sectors.begin(), sectors.end());
    std::vector<Record> result;
    for (const auto& sector : sectors) {
        for (std::size_t slot = 0; slot < kRecordsPerSector; ++slot) {
            Record record;
            flash.read(sector.second + sizeof(SectorHeader) + slot * sizeof(Record),
                       &record, sizeof(record));
            if (valid(record)) result.push_back(record);
        }
    }
    return result;
}

void filters() {
    Filter filter;
    Frame a{0x55, 0xaa, 8, 2, 3, 0x21, 1, 0x2d};
    auto b = a;
    b[5]++;
    b[7] = calculate_checksum(b);
    CHECK(filter.receive(a));
    CHECK(filter.transmit(Command::kUpHold));
    CHECK(!filter.receive(a));  // independent per-direction baselines
    CHECK(!filter.transmit(Command::kUpHold));
    CHECK(filter.receive(b));
    CHECK(filter.receive(a));   // A -> B -> A is a new change
    CHECK(filter.receive(a, true));
    CHECK(!filter.receive(a, true));
    CHECK(filter.transmit(Command::kIdle));
    CHECK(filter.transmit(Command::kUpStep));
    CHECK(!filter.transmit(Command::kIdle, true));
    CHECK(filter.transmit(Command::kUpStep));  // second identical STEP is retained
    CHECK(filter.transmit(Command::kIdle, false));  // safety stop is not hidden
    CHECK(!filter.transmit(Command::kIdle));
    CHECK(filter.transmit_error(Command::kIdle));
    CHECK(!filter.transmit_error(Command::kIdle));
    CHECK(filter.transmit(Command::kIdle));  // transport recovered, same opcode
    CHECK(filter.transmit_error(Command::kUpStep));
    CHECK(filter.transmit(Command::kIdle, true));  // failed STEP is not a normal pulse
}

void capture_order_gate_and_overflow() {
    Capture capture;
    Frame unknown{0x55, 0xaa, 8, 7, 0, 2, 1, 0x10};
    capture.begin(3, 100, 10, {}, 1);
    capture.receive(unknown, 20);
    CHECK(!capture.take(21));  // no persistence before writer handles the fault
    capture.transmitted(Command::kIdle, 25, Context::kNone, false);
    capture.cycle_completed(0);  // older writer snapshot must not open the gate
    CHECK(!capture.take(26));
    capture.receive(unknown, 27);  // same frame: no log row, but newer safety epoch
    capture.cycle_completed(1);
    CHECK(capture.safety_pending());
    capture.cycle_completed(2);
    const auto boot = capture.take(30);
    const auto rx = capture.take(30);
    const auto tx = capture.take(30);
    CHECK(boot && boot->kind == Kind::kBoot && boot->sequence == 101);
    CHECK(rx && rx->kind == Kind::kRx && rx->uptime_ms == 20);
    CHECK(tx && tx->kind == Kind::kTx && tx->uptime_ms == 25);
    CHECK(!capture.take(30));  // no duplicate frame or repeated timestamp

    capture.begin(4, 0, 0, {}, 1);
    for (std::size_t n = 0; n <= Capture::kCapacity; ++n) {
        capture.transmitted(n % 2 ? Command::kUpHold : Command::kIdle,
                            n + 1, Context::kNone, false);
    }
    auto first = capture.take(2000);
    CHECK(first && first->sequence == 3);  // two oldest entries displaced
    for (std::size_t n = 1; n < Capture::kCapacity; ++n) CHECK(capture.take(2000));
    const auto gap = capture.take(2000);
    CHECK(gap && gap->kind == Kind::kGap && gap->data[0] == 2);
    CHECK(!capture.take(2000));
}

void record_power_cuts() {
    CHECK(crc32("123456789", 9) == 0xcbf43926);  // standard cross-language CRC vector
    for (std::size_t cut = 0; cut < sizeof(Record); ++cut) {
        MemoryFlash flash;
        Store store(flash);
        CHECK(store.mount());
        CHECK(store.append(entry(1)) == AppendResult::kWritten);
        flash.write_limit = cut;
        CHECK(store.append(entry(2)) == AppendResult::kIoError);
        flash.write_limit = 1000000;
        Store reboot(flash);
        CHECK(reboot.mount());
        CHECK(reboot.last_sequence() == 1);
        CHECK(reboot.last_boot_id() == 1);
        CHECK(reboot.append(entry(2, 2)) == AppendResult::kWritten);
        const auto saved = records(flash);
        CHECK(saved.size() == 2);
        CHECK(saved.back().boot_id == 2);
        CHECK(saved.back().uptime_ms > (1ULL << 32));
    }
    MemoryFlash flash;
    Store store(flash);
    CHECK(store.mount());
    CHECK(store.append(entry(1)) == AppendResult::kWritten);
    flash.report_write_error = true;
    CHECK(store.append(entry(2)) == AppendResult::kWritten);  // readback resolves ambiguity
    CHECK(records(flash).size() == 2);
}

void header_power_cuts() {
    for (std::size_t cut = 0; cut < sizeof(SectorHeader); ++cut) {
        MemoryFlash flash;
        Store store(flash);
        CHECK(store.mount());
        for (unsigned n = 1; n <= kRecordsPerSector; ++n) {
            CHECK(store.append(entry(n)) == AppendResult::kWritten);
        }
        flash.write_limit = cut;
        CHECK(store.append(entry(128)) == AppendResult::kIoError);
        flash.write_limit = 1000000;
        Store reboot(flash);
        CHECK(reboot.mount());
        CHECK(reboot.last_sequence() == 127);
        CHECK(reboot.prepare_sector());
        CHECK(reboot.append(entry(128, 2)) == AppendResult::kWritten);
        CHECK(records(flash).size() == 128);
    }
}

void rotation_and_erase_cuts() {
    MemoryFlash original;
    Store store(original);
    CHECK(store.mount());
    for (unsigned n = 1; n <= 254; ++n) CHECK(store.append(entry(n)) == AppendResult::kWritten);
    CHECK(original.erases == 0);  // appends never erase, even across a blank boundary
    CHECK(store.append(entry(255)) == AppendResult::kNeedsSector);
    CHECK(original.erases == 0);
    for (const std::size_t cut : {0U, 1U, 16U, 32U, 2048U, 4095U}) {
        MemoryFlash flash = original;
        Store mounted(flash);
        CHECK(mounted.mount());
        flash.erase_limit = cut;
        CHECK(!mounted.prepare_sector());
        Store reboot(flash);
        CHECK(reboot.mount());
        CHECK(reboot.last_sequence() == 254);
        flash.erase_limit = kSectorSize;
        CHECK(reboot.prepare_sector());
        CHECK(reboot.append(entry(255, 2)) == AppendResult::kWritten);
        const auto saved = records(flash);
        CHECK(saved.size() == 128);
        CHECK(saved.front().sequence == 128 && saved.back().sequence == 255);
    }
    for (unsigned n = 255; n <= 1000; ++n) {
        auto result = store.append(entry(n));
        if (result == AppendResult::kNeedsSector) {
            CHECK(store.prepare_sector());
            result = store.append(entry(n));
        }
        CHECK(result == AppendResult::kWritten);
    }
    Store reboot(original);
    CHECK(reboot.mount());
    CHECK(reboot.last_sequence() == 1000);
    const auto saved = records(original);
    CHECK(saved.size() <= 254 && saved.size() >= 128);
    for (std::size_t i = 1; i < saved.size(); ++i) {
        CHECK(saved[i].sequence == saved[i - 1].sequence + 1);
    }
}

void invalid_capture_preserves_parser() {
    StreamParser parser(Direction::kControllerToPanel);
    Frame bad{0x55, 0xaa, 8, 2, 3, 0x21, 1, 0};
    for (auto byte : bad) CHECK(!parser.feed(byte));
    CHECK(parser.take_rejected() == bad);
    CHECK(!parser.take_rejected());
    auto good = bad;
    good[7] = calculate_checksum(good);
    std::optional<Frame> decoded;
    for (auto byte : good) decoded = parser.feed(byte);
    CHECK(decoded == good);
    CHECK(!parser.take_rejected());
}

void preset_changes_survive_mount() {
    MemoryFlash flash;
    Store store(flash);
    CHECK(store.mount());
    Capture capture;
    capture.begin(8, 0, 100, {}, 1);
    capture.presets(804, 1150, 804, 1150, 110);  // initial NVS snapshot
    capture.presets(804, 1150, 820, 1150, 200, Context::kSit);
    capture.presets(820, 1150, 820, 1150, 201, Context::kSit);  // no change
    capture.presets(820, 1150, 820, 1160, 300, Context::kStand);
    capture.presets(820, 1160, 804, 1160, 400, Context::kSit);  // A -> B -> A
    capture.presets(804, 1160, 804, 1160, 401, Context::kCalibration);  // invalid source
    while (auto record = capture.take(500)) {
        seal(*record);
        CHECK(store.append(*record) == AppendResult::kWritten);
    }
    Store remounted(flash);
    CHECK(remounted.mount());
    CHECK(remounted.last_sequence() == 5);
    CHECK(remounted.last_boot_id() == 8);
    const auto saved = records(flash);
    CHECK(saved.size() == 5);
    CHECK(saved[1].kind == Kind::kPresets && saved[1].context == 0);
    const Frame sit_change{0x24, 0x03, 0x7e, 0x04, 0x34, 0x03, 0x7e, 0x04};
    CHECK(saved[2].data == sit_change);
    CHECK(saved[2].context == static_cast<unsigned>(Context::kSit));
    CHECK(saved[2].uptime_ms == 200);
    CHECK(saved[3].context == static_cast<unsigned>(Context::kStand));
    CHECK(saved[4].uptime_ms == 400);
}

void fixture(const char* path) {
    MemoryFlash flash;
    Store store(flash);
    CHECK(store.mount());
    auto record = entry(1);
    record.kind = Kind::kBoot;
    record.context = 1;
    record.data = {1, 2, 3, 4, 5, 6, 7, 8};
    seal(record);
    CHECK(store.append(record) == AppendResult::kWritten);
    record = entry(2);
    record.context = static_cast<unsigned>(Context::kDial);
    seal(record);
    CHECK(store.append(record) == AppendResult::kWritten);
    record = entry(3);
    record.kind = Kind::kRx;
    record.data = {0x55, 0xaa, 8, 7, 0, 2, 1, 0x10};
    seal(record);
    CHECK(store.append(record) == AppendResult::kWritten);
    record = entry(4);
    record.data = make_panel_frame(Command::kIdle);
    seal(record);
    CHECK(store.append(record) == AppendResult::kWritten);
    record = entry(5);
    record.kind = Kind::kPresets;
    record.data = {0x24, 0x03, 0x7e, 0x04, 0x24, 0x03, 0x7e, 0x04};
    record.context = 0;
    seal(record);
    CHECK(store.append(record) == AppendResult::kWritten);
    record.sequence = 6;
    record.context = static_cast<unsigned>(Context::kSit);
    record.data[4] = 0x34;
    seal(record);
    CHECK(store.append(record) == AppendResult::kWritten);
    record.sequence = 7;
    record.context = static_cast<unsigned>(Context::kStand);
    record.data = {0x34, 0x03, 0x7e, 0x04, 0x34, 0x03, 0x88, 0x04};
    seal(record);
    CHECK(store.append(record) == AppendResult::kWritten);
    // Leave a torn tail: decoder must retain all seven previous records.
    flash.write_limit = 13;
    CHECK(store.append(entry(8)) == AppendResult::kIoError);
    std::ofstream out(path, std::ios::binary);
    out.write(reinterpret_cast<const char*>(flash.bytes.data()), flash.bytes.size());
    CHECK(out.good());
}
}  // namespace

int main(int argc, char** argv) {
    if (argc == 3 && std::strcmp(argv[1], "--fixture") == 0) fixture(argv[2]);
    else {
        filters();
        capture_order_gate_and_overflow();
        record_power_cuts();
        header_power_cuts();
        rotation_and_erase_cuts();
        invalid_capture_preserves_parser();
        preset_changes_survive_mount();
    }
    return failures ? 1 : 0;
}
