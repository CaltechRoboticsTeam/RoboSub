// Teensy 4.1: USB servo commands -> existing VideoRay RS-485 bus.
#include <Arduino.h>
#include <stdarg.h>

#include "hardware_config.h"
#include "protocol.h"

namespace {

static_assert(robosub::kMaxDeltaUs == 500, "USB capability must match limit");

constexpr int kDriverEnablePin = 3;
constexpr int kReceiverEnablePin = 2;
constexpr int kPressurePin = A0;
constexpr uint32_t kBusBaud = 115200;
#if ROBOSUB_RS485_SERIAL == 2
HardwareSerial& thruster_serial = Serial2;
#else
HardwareSerial& thruster_serial = Serial1;
#endif
robosub::UsbController controller;
char input_line[robosub::kLineCapacity];
size_t input_length = 0;
bool discard_line = false;
bool usb_was_connected = false;
uint32_t packets_sent = 0;
uint32_t response_bytes = 0;
uint32_t serial1_bytes = 0;
uint32_t serial2_bytes = 0;
uint32_t last_neutral_ms = 0;
uint32_t last_depth_ms = 0;

void Drive() {
  if (ROBOSUB_RS485_SERIAL == 0) {
    return;
  }
  uint8_t packet[robosub::kPacketSize];
  robosub::MakePacket(controller.pwm(), packet);
  digitalWrite(kDriverEnablePin, HIGH);
  digitalWrite(kReceiverEnablePin, HIGH);
  thruster_serial.write(packet, sizeof(packet));
  // 48 bytes at 115200 baud takes about 4.2 ms, below the watchdog period.
  thruster_serial.flush();
  digitalWrite(kDriverEnablePin, LOW);
  digitalWrite(kReceiverEnablePin, LOW);
  ++packets_sent;
  last_neutral_ms = millis();
}

void Send(const char* format, ...) {
  char line[robosub::kLineCapacity];
  va_list args;
  va_start(args, format);
  const int length = vsnprintf(line, sizeof(line), format, args);
  va_end(args);
  if (length < 0 || static_cast<size_t>(length) + 10 >= sizeof(line)) {
    return;
  }
  const uint32_t crc = robosub::Crc32(reinterpret_cast<const uint8_t*>(line),
                                      static_cast<size_t>(length));
  const int suffix = snprintf(line + length, sizeof(line) - length, "*%08lX\n",
                              static_cast<unsigned long>(crc));
  if (suffix != 10) {
    return;
  }
  const size_t total = static_cast<size_t>(length + suffix);
  // A slow or disconnected USB reader must never delay the watchdog.
  if (Serial && Serial.availableForWrite() >= static_cast<int>(total)) {
    Serial.write(reinterpret_cast<const uint8_t*>(line), total);
  }
}

void Fault(const char* reason) {
  controller.Stop();
  Drive();
  Send("F1,%lu,%s", static_cast<unsigned long>(controller.session()), reason);
}

void HandleLine() {
  input_line[input_length] = '\0';
  const auto accepted = controller.AcceptLine(input_line, millis());
  if (ROBOSUB_RS485_SERIAL == 0 && controller.enabled()) {
    Fault("BUS_DISABLED");
    return;
  }
  Drive();
  if (accepted == robosub::Accepted::kHello) {
    Send("I1,%lu,1,8,250,500,%u",
         static_cast<unsigned long>(controller.session()),
         static_cast<unsigned int>(ROBOSUB_RS485_SERIAL));
  } else if (accepted == robosub::Accepted::kCommand) {
    Send("A1,%lu,%lu,%u,%lu,%lu",
         static_cast<unsigned long>(controller.session()),
         static_cast<unsigned long>(controller.sequence()),
         controller.enabled() ? 1U : 0U,
         static_cast<unsigned long>(packets_sent),
         static_cast<unsigned long>(response_bytes));
  } else {
    Send("F1,%lu,INVALID", static_cast<unsigned long>(controller.session()));
  }
}

}  // namespace

void setup() {
  pinMode(kDriverEnablePin, OUTPUT);
  pinMode(kReceiverEnablePin, OUTPUT);
  digitalWrite(kDriverEnablePin, LOW);
  digitalWrite(kReceiverEnablePin, LOW);
  pinMode(kPressurePin, INPUT);
  // Receive-only diagnostics on both UARTs; DE remains low in USB-only mode.
  Serial1.begin(kBusBaud);
  Serial2.begin(kBusBaud);
  Drive();
}

void loop() {
  const uint32_t now_ms = millis();
  const bool connected = static_cast<bool>(Serial);
  if (!connected && usb_was_connected) {
    controller.Disconnect();
    input_length = 0;
    discard_line = false;
    Drive();
  }
  usb_was_connected = connected;
  if (controller.PollWatchdog(now_ms)) {
    Fault("TIMEOUT");
  }
  // Bound input work so a continuously writing host cannot starve safety work.
  for (size_t budget = 0; connected && budget < 256 && Serial.available();
       ++budget) {
    const int value = Serial.read();
    if (value < 0) {
      break;
    }
    const char byte = static_cast<char>(value);
    if (byte == '\n') {
      if (!discard_line) {
        HandleLine();
      }
      input_length = 0;
      discard_line = false;
    } else if (!discard_line) {
      if (byte < 32 || byte > 126 || input_length >= sizeof(input_line) - 1) {
        discard_line = true;
        Fault("FRAMING");
      } else {
        input_line[input_length++] = byte;
      }
    }
  }
  for (size_t budget = 0; budget < 128; ++budget) {
    if (Serial1.available() && Serial1.read() >= 0) {
      ++serial1_bytes;
      if (ROBOSUB_RS485_SERIAL == 1) {
        ++response_bytes;
      }
    }
    if (Serial2.available() && Serial2.read() >= 0) {
      ++serial2_bytes;
      if (ROBOSUB_RS485_SERIAL == 2) {
        ++response_bytes;
      }
    }
  }
  if (!controller.enabled() && millis() - last_neutral_ms >= 100) {
    Drive();
  }
  if (connected && now_ms - last_depth_ms >= 100) {
    last_depth_ms = now_ms;
    const int adc = analogRead(kPressurePin);
    // Preserve the installed sketch's ADC configuration and calibration.
    const double slope = 30.48 / (415.0 - 403.0);
    const double offset = 30.48 - slope * 415.0;
    const double depth_m = (slope * adc + offset) / 100.0;
    Send("D1,%d,%.6f,%lu,%lu", adc, depth_m,
         static_cast<unsigned long>(serial1_bytes),
         static_cast<unsigned long>(serial2_bytes));
  }
}
