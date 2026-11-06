// USB commands and VideoRay packets, independent of Arduino for verification.
#ifndef ROBOSUB_TEENSY_USB_M5_PROTOCOL_H_
#define ROBOSUB_TEENSY_USB_M5_PROTOCOL_H_

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace robosub {

// USB packets contain eight slots; six are assigned to thrusters.
constexpr size_t kSlots = 8;
constexpr uint16_t kNeutralUs = 1500;
constexpr uint16_t kMaxDeltaUs = 500;
constexpr uint32_t kWatchdogMs = 250;
constexpr size_t kLineCapacity = 160;
constexpr size_t kPacketSize = 48;

inline uint32_t Crc32(const uint8_t* data, size_t length) {
  uint32_t crc = 0xffffffffU;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1) ^ ((crc & 1U) ? 0xedb88320U : 0U);
    }
  }
  return crc ^ 0xffffffffU;
}

inline bool ParseUint(const char* text, uint32_t* result, uint32_t base = 10) {
  if (*text == '\0') {
    return false;
  }
  uint32_t value = 0;
  for (; *text != '\0'; ++text) {
    uint32_t digit;
    if (*text >= '0' && *text <= '9') {
      digit = static_cast<uint32_t>(*text - '0');
    } else if (base == 16 && *text >= 'A' && *text <= 'F') {
      digit = static_cast<uint32_t>(*text - 'A' + 10);
    } else if (base == 16 && *text >= 'a' && *text <= 'f') {
      digit = static_cast<uint32_t>(*text - 'a' + 10);
    } else {
      return false;
    }
    if (digit >= base || value > (UINT32_MAX - digit) / base) {
      return false;
    }
    value = value * base + digit;
  }
  *result = value;
  return true;
}

inline void WriteUint32(uint8_t* output, uint32_t value) {
  for (size_t i = 0; i < 4; ++i) {
    output[i] = static_cast<uint8_t>(value >> (8 * i));
  }
}

inline void MakePacket(const uint16_t (&pwm)[kSlots], uint8_t* output,
                       uint8_t reply_node = 1) {
  static_assert(sizeof(float) == 4, "VideoRay requires 32-bit floats");
  // Group and power slots follow the installed sketch; node IDs are separate.
  const uint8_t header[] = {0xf5, 0x5f, 0x81, 0x02, 0xf0, 34};
  memcpy(output, header, sizeof(header));
  WriteUint32(output + 6, Crc32(output, 6));
  output[10] = 0xaa;
  output[11] = reply_node;
  for (size_t i = 0; i < kSlots; ++i) {
    const float power = (static_cast<float>(pwm[i]) - kNeutralUs) / 500.0f;
    uint32_t bits;
    memcpy(&bits, &power, sizeof(bits));
    WriteUint32(output + 12 + 4 * i, bits);
  }
  WriteUint32(output + 44, Crc32(output + 10, 34));
}

enum class Accepted { kRejected, kHello, kCommand, kQuery };

class UsbController {
 public:
  UsbController() { Stop(); }

  void Stop() {
    enabled_ = false;
    ready_ = false;
    for (size_t i = 0; i < kSlots; ++i) {
      pwm_[i] = kNeutralUs;
    }
  }

  void Disconnect() {
    Stop();
    session_ = 0;
    sequence_ = 0;
  }

  bool PollWatchdog(uint32_t now_ms) {
    if (ready_ && static_cast<uint32_t>(now_ms - last_ms_) >= kWatchdogMs) {
      Stop();
      return true;
    }
    return false;
  }

  // The caller owns a bounded, NUL-terminated line without the newline.
  Accepted AcceptLine(char* line, uint32_t now_ms) {
    const Accepted result = ParseLine(line, now_ms);
    if (result == Accepted::kRejected) {
      Stop();
    }
    return result;
  }

  const uint16_t (&pwm() const)[kSlots] { return pwm_; }
  uint32_t session() const { return session_; }
  uint32_t sequence() const { return sequence_; }
  bool enabled() const { return enabled_; }
  uint8_t query_node() const { return query_node_; }

 private:
  Accepted ParseLine(char* line, uint32_t now_ms) {
    char* star = strchr(line, '*');
    uint32_t checksum;
    if (star == nullptr || strlen(star + 1) != 8 ||
        !ParseUint(star + 1, &checksum, 16) ||
        Crc32(reinterpret_cast<const uint8_t*>(line),
              static_cast<size_t>(star - line)) != checksum) {
      return Accepted::kRejected;
    }
    *star = '\0';
    char* fields[12];
    size_t count = 1;
    fields[0] = line;
    for (char* p = line; *p != '\0'; ++p) {
      if (*p == ',') {
        if (count == 12) {
          return Accepted::kRejected;
        }
        *p = '\0';
        fields[count++] = p + 1;
      }
    }
    uint32_t session;
    if (count < 2 || !ParseUint(fields[1], &session) || session == 0) {
      return Accepted::kRejected;
    }
    if (strcmp(fields[0], "H1") == 0 && count == 2) {
      Stop();
      session_ = session;
      sequence_ = 0;
      return Accepted::kHello;
    }
    if (strcmp(fields[0], "Q1") == 0 && count == 3 && session == session_) {
      uint32_t node;
      if (!ParseUint(fields[2], &node) || node > 127) {
        return Accepted::kRejected;
      }
      query_node_ = static_cast<uint8_t>(node);
      // Reads must not enable outputs or extend the command watchdog.
      return Accepted::kQuery;
    }
    if (strcmp(fields[0], "C1") != 0 || count != 12 || session != session_) {
      return Accepted::kRejected;
    }
    uint32_t sequence;
    uint32_t enabled;
    if (!ParseUint(fields[2], &sequence) || sequence <= sequence_ ||
        !ParseUint(fields[3], &enabled) || enabled > 1) {
      return Accepted::kRejected;
    }
    uint16_t proposed[kSlots];
    for (size_t i = 0; i < kSlots; ++i) {
      uint32_t value;
      if (!ParseUint(fields[4 + i], &value) ||
          value < kNeutralUs - kMaxDeltaUs ||
          value > kNeutralUs + kMaxDeltaUs ||
          (enabled == 0 && value != kNeutralUs)) {
        return Accepted::kRejected;
      }
      proposed[i] = static_cast<uint16_t>(value);
    }
    if (enabled != 0 && !ready_) {
      return Accepted::kRejected;
    }
    memcpy(pwm_, proposed, sizeof(pwm_));
    sequence_ = sequence;
    enabled_ = enabled != 0;
    ready_ = true;
    last_ms_ = now_ms;
    return Accepted::kCommand;
  }

  uint16_t pwm_[kSlots];
  uint32_t session_ = 0;
  uint32_t sequence_ = 0;
  uint32_t last_ms_ = 0;
  bool enabled_ = false;
  bool ready_ = false;
  uint8_t query_node_ = 0;
};

}  // namespace robosub
#endif  // ROBOSUB_TEENSY_USB_M5_PROTOCOL_H_
