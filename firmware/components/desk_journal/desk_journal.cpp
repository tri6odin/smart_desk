#include "desk_journal.hpp"

#include <cstring>

namespace smart_desk::journal {
namespace {
bool erased(const void* data, std::size_t size) {
    const auto* bytes = static_cast<const std::uint8_t*>(data);
    for (std::size_t i = 0; i < size; ++i) {
        if (bytes[i] != 0xff) return false;
    }
    return true;
}
}  // namespace

std::uint32_t crc32(const void* data, std::size_t size) {
    const auto* bytes = static_cast<const std::uint8_t*>(data);
    std::uint32_t crc = 0xffffffff;
    for (std::size_t i = 0; i < size; ++i) {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ ((crc & 1) ? 0xedb88320 : 0);
        }
    }
    return crc ^ 0xffffffff;
}

void seal(Record& record) { record.crc = crc32(&record, offsetof(Record, crc)); }
bool valid(const Record& record) {
    return record.kind >= Kind::kBoot && record.kind <= Kind::kPresets &&
           record.reserved == 0 &&
           record.crc == crc32(&record, offsetof(Record, crc));
}
bool valid(const SectorHeader& header) {
    return header.magic == kMagic && header.version == kVersion &&
           header.record_size == sizeof(Record) && header.generation != 0 &&
           header.reserved == 0 &&
           header.crc == crc32(&header, offsetof(SectorHeader, crc));
}

bool Filter::receive(const Frame& frame, bool invalid) {
    if (have_rx_ && frame == last_rx_ && invalid == last_rx_invalid_) return false;
    last_rx_ = frame;
    last_rx_invalid_ = invalid;
    have_rx_ = true;
    return true;
}

bool Filter::transmit(Command command, bool routine_step_idle, StopReason reason) {
    if (command != Command::kIdle) reason = StopReason::kNone;
    const bool changed = !have_tx_ || last_tx_error_ || command != last_tx_ ||
        (reason != StopReason::kNone && reason != last_stop_reason_);
    const bool step_idle = have_tx_ && !last_tx_error_ && command == Command::kIdle &&
        (last_tx_ == Command::kUpStep || last_tx_ == Command::kDownStep) &&
        routine_step_idle && reason == StopReason::kNone;
    last_tx_ = command;
    have_tx_ = true;
    last_tx_error_ = false;
    if (command != Command::kIdle || reason != StopReason::kNone) {
        last_stop_reason_ = reason;
    }
    return changed && !step_idle;
}

bool Filter::transmit_error(Command command) {
    const bool changed = !have_tx_ || !last_tx_error_ || command != last_tx_;
    last_tx_ = command;
    have_tx_ = true;
    last_tx_error_ = true;
    return changed;
}

void Capture::begin(std::uint32_t boot, std::uint32_t previous_sequence,
                    std::uint64_t now, const Frame& build, std::uint8_t reset_reason) {
    boot_ = boot;
    sequence_ = previous_sequence;
    head_ = count_ = 0;
    dropped_ = unknown_ = handled_ = 0;
    filter_ = Filter{};
    enqueue(Kind::kBoot, build, now, reset_reason);
}

Record Capture::make_record(Kind kind, const Frame& data, std::uint64_t now,
                            std::uint8_t context) {
    Record record;
    record.uptime_ms = now;
    record.boot_id = boot_;
    record.sequence = ++sequence_;
    record.kind = kind;
    record.data = data;
    record.context = context;
    return record;
}

void Capture::enqueue(Kind kind, const Frame& data, std::uint64_t now,
                      std::uint8_t context) {
    // Preserve newest evidence (especially fault + stop) on overflow.
    if (count_ == kCapacity) {
        head_ = (head_ + 1) % kCapacity;
        --count_;
        ++dropped_;
    }
    queue_[(head_ + count_) % kCapacity] = make_record(kind, data, now, context);
    ++count_;
}

void Capture::receive(const Frame& frame, std::uint64_t now, bool invalid) {
    // Count all unknown frames, including duplicates, matching TelemetryTracker.
    if (!invalid && classify_controller_frame(frame) == ControllerFrameKind::kUnknown) {
        ++unknown_;
    }
    if (filter_.receive(frame, invalid)) enqueue(invalid ? Kind::kRxInvalid : Kind::kRx, frame, now);
}

void Capture::transmitted(Command command, std::uint64_t now, Context context,
                          bool routine_step_idle, StopReason reason) {
    if (filter_.transmit(command, routine_step_idle, reason)) {
        enqueue(Kind::kTx, make_panel_frame(command), now, tx_context(command, context, reason));
    }
}

void Capture::transmit_error(Command command, std::uint64_t now,
                             Context context, StopReason reason) {
    if (filter_.transmit_error(command)) {
        enqueue(Kind::kTxError, make_panel_frame(command), now, tx_context(command, context, reason));
    }
}

void Capture::presets(std::uint16_t previous_sit, std::uint16_t previous_stand,
                      std::uint16_t sit, std::uint16_t stand, std::uint64_t now,
                      Context context) {
    if (context != Context::kNone && context != Context::kSit &&
        context != Context::kStand) return;
    if (context != Context::kNone && previous_sit == sit &&
        previous_stand == stand) return;
    Frame data{};
    const std::uint16_t values[]{previous_sit, previous_stand, sit, stand};
    for (std::size_t i = 0; i < 4; ++i) {
        data[2 * i] = static_cast<std::uint8_t>(values[i]);
        data[2 * i + 1] = static_cast<std::uint8_t>(values[i] >> 8);
    }
    enqueue(Kind::kPresets, data, now, static_cast<std::uint8_t>(context));
}

std::optional<Record> Capture::take(std::uint64_t now) {
    if (safety_pending()) return std::nullopt;
    if (count_ != 0) {
        auto record = queue_[head_];
        head_ = (head_ + 1) % kCapacity;
        --count_;
        return record;
    }
    if (dropped_ != 0) {
        Frame bytes{};
        std::memcpy(bytes.data(), &dropped_, sizeof(dropped_));
        auto record = make_record(Kind::kGap, bytes, now);
        dropped_ = 0;
        return record;
    }
    return std::nullopt;
}

bool Store::mount() {
    if (flash_.size() < 2 * kSectorSize || flash_.size() % kSectorSize != 0) {
        return false;
    }
    generation_ = 0;
    sequence_ = boot_id_ = 0;
    sector_ = 0;
    slot_ = kRecordsPerSector;
    for (std::size_t offset = 0; offset < flash_.size(); offset += kSectorSize) {
        SectorHeader header;
        if (!flash_.read(offset, &header, sizeof(header))) return false;
        if (valid(header) && header.generation > generation_) {
            generation_ = header.generation;
            sector_ = offset / kSectorSize;
            sequence_ = header.next_sequence - 1;
            boot_id_ = header.boot_id;
        }
    }
    if (generation_ == 0) return true;
    slot_ = 0;
    // A torn record consumes its slot. Never overwrite partly programmed
    // bytes, and never stop scanning at the first invalid record.
    for (std::size_t i = 0; i < kRecordsPerSector; ++i) {
        Record record;
        const auto offset = sector_ * kSectorSize + sizeof(SectorHeader) +
                            i * sizeof(Record);
        if (!flash_.read(offset, &record, sizeof(record))) return false;
        if (!erased(&record, sizeof(record))) slot_ = i + 1;
        if (valid(record)) {
            sequence_ = record.sequence;
            boot_id_ = record.boot_id;
        }
    }
    return true;
}

bool Store::prepare_sector() {
    if (!needs_sector()) return true;
    return open_next(true);
}

bool Store::open_next(bool erase) {
    const auto next = generation_ == 0 ? 0 :
        (sector_ + 1) % (flash_.size() / kSectorSize);
    const auto offset = next * kSectorSize;
    SectorHeader header;
    header.generation = generation_ + 1;
    header.next_sequence = sequence_ + 1;
    header.boot_id = boot_id_;
    header.crc = crc32(&header, offsetof(SectorHeader, crc));
    if (erase && !flash_.erase_sector(offset)) return false;
    const bool written = flash_.write(offset, &header, sizeof(header));
    SectorHeader check;
    if (!written || !flash_.read(offset, &check, sizeof(check)) ||
        std::memcmp(&header, &check, sizeof(header)) != 0) return false;
    sector_ = next;
    slot_ = 0;
    generation_ = header.generation;
    return true;
}

AppendResult Store::append(const Record& record) {
    if (!valid(record)) return AppendResult::kIoError;
    if (needs_sector()) {
        const auto next = generation_ == 0 ? 0 :
            (sector_ + 1) % (flash_.size() / kSectorSize);
        // A blank header alone is insufficient: a power cut during erase
        // can leave the sector's tail programmed.
        std::array<std::uint8_t, 256> bytes{};
        for (std::size_t pos = 0; pos < kSectorSize; pos += bytes.size()) {
            if (!flash_.read(next * kSectorSize + pos, bytes.data(), bytes.size())) {
                return AppendResult::kIoError;
            }
            if (!erased(bytes.data(), bytes.size())) return AppendResult::kNeedsSector;
        }
        if (!open_next(false)) return AppendResult::kIoError;
    }
    const auto offset = sector_ * kSectorSize + sizeof(SectorHeader) +
                        slot_++ * sizeof(Record);
    // A driver can report an error after the write reached flash. Readback
    // resolves that ambiguity without creating a duplicate retry record.
    flash_.write(offset, &record, sizeof(record));
    Record check;
    if (!flash_.read(offset, &check, sizeof(check)) ||
        std::memcmp(&record, &check, sizeof(record)) != 0) {
        return AppendResult::kIoError;
    }
    sequence_ = record.sequence;
    boot_id_ = record.boot_id;
    return AppendResult::kWritten;
}

}  // namespace smart_desk::journal
