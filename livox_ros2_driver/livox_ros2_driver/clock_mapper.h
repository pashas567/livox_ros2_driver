#pragma once
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>

// Shared clock mapping for BOTH packet streams of one device.
class LivoxClockMapper {
 public:
  void Configure(bool host_offset, int64_t ptp_utc_offset_ns) {
    std::lock_guard<std::mutex> guard(mutex_);
    host_offset_ = host_offset;
    ptp_offset_ = ptp_utc_offset_ns;
  }
  int64_t Offset(int64_t raw, int mode, bool imu, int64_t wall, int64_t steady) {
    std::lock_guard<std::mutex> guard(mutex_);
    const size_t index = imu ? 1 : 0;
    if (seen_ && (mode != mode_ || (last_raw_[index] && raw < last_raw_[index] - 50000000LL)
                  || std::llabs((wall - steady) - wall_steady_) > 50000000LL)) {
      ++epoch_;
      fault_ = true; // Explicit driver restart required; never refresh stale data.
    }
    if (!seen_) {
      offset_ = host_offset_ && mode == 0 ? wall - raw : 0;
      seen_ = true;
    }
    mode_ = mode;
    wall_steady_ = wall - steady;
    const int64_t offset = mode == 1 ? -ptp_offset_ : (mode == 0 ? offset_ : 0);
    last_raw_[index] = raw;
    last_wall_[index] = wall;
    last_mapped_[index] = raw + offset;
    received_steady_[index] = steady;
    return offset;
  }
  std::string Json(int64_t now_steady) {
    std::lock_guard<std::mutex> guard(mutex_);
    if (!seen_) return "";
    bool valid = mode_ == 1 && !fault_;
    for (size_t i = 0; i < 2; ++i) {
      const auto age = last_wall_[i] - last_mapped_[i];
      valid = valid && received_steady_[i] && now_steady - received_steady_[i] < 500000000LL
              && age >= -50000000LL && age < 250000000LL;
    }
    std::ostringstream out;
    out << "{\"timestamp_type\":" << mode_ << ",\"epoch\":" << epoch_
        << ",\"fault\":" << (fault_ ? "true" : "false")
        << ",\"ptp_eligible\":" << (valid ? "true" : "false")
        << ",\"mapped_offset_ns\":" << (last_mapped_[0] - last_raw_[0])
        << ",\"host_offset\":" << (host_offset_ ? "true" : "false")
        << ",\"raw_ns\":[" << last_raw_[0] << ',' << last_raw_[1]
        << "],\"mapped_ns\":[" << last_mapped_[0] << ',' << last_mapped_[1]
        << "],\"receipt_ns\":[" << last_wall_[0] << ',' << last_wall_[1] << "]}";
    return out.str();
  }
 private:
  std::mutex mutex_;
  bool seen_ = false, host_offset_ = false, fault_ = false;
  int mode_ = -1;
  uint64_t epoch_ = 0;
  int64_t offset_ = 0, ptp_offset_ = 0, wall_steady_ = 0;
  std::array<int64_t, 2> last_raw_{}, last_mapped_{}, last_wall_{}, received_steady_{};
};
