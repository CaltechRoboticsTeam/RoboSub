// Teensy 4.1: USB servo commands -> existing VideoRay RS-485 bus.
#include <Arduino.h>
#include <stdarg.h>

#include "hardware_config.h"
#include "protocol.h"

#if ROBOSUB_RS485_SERIAL != 0
#include <FlexSerial.h>
#endif

namespace {

static_assert(robosub::kMaxDeltaUs == 500, "USB capability must match limit");

constexpr int kDriverEnablePin = 4;
constexpr int kReceiverEnablePin = 5;
constexpr int kReceiverOutputPin = 2;
constexpr int kDriverInputPin = 3;
constexpr int kPressurePin = A10;  // Teensy 4.1 pin 24.
constexpr uint32_t kBusBaud = 115200;
#if ROBOSUB_RS485_SERIAL == 2
HardwareSerialIMXRT& thruster_serial = Serial2;
#else
HardwareSerialIMXRT& thruster_serial = Serial1;
#endif

#if ROBOSUB_RS485_SERIAL != 0
// Pin 3 has no UART TX function. Reserve FlexIO timer/shifter 0 for TX;
// Serial1/Serial2 receives pin 2 through the Teensy XBAR.
class BusTransmitter : public FlexSerial {
 public:
  BusTransmitter()
      : FlexSerial(-1, kDriverInputPin, 0xff, 0xff, 0xff, 0xff, 0, 0) {}

  void Begin() {
    begin(kBusBaud);
    ready_ = static_cast<bool>(*this);
  }

  bool Transmit(const uint8_t* packet, size_t length) {
    // Queue the entire bounded packet without entering Print's blocking wait.
    if (!ready_ || length == 0 || length >= TX_BUFFER_SIZE ||
        static_cast<size_t>(availableForWrite()) < length) {
      return false;
    }
    digitalWrite(kReceiverEnablePin, HIGH);
    digitalWrite(kDriverEnablePin, HIGH);
    write(packet, length);
    const uint32_t started_us = micros();
    IMXRT_FLEXIO_t& port = flexIOHandlerTX()->port();
    // FlexSerial clears timer interrupt 0 after the final stop bit. Avoid
    // flush()/yield() so USB processing cannot delay the return to receive.
    while (((port.SHIFTSIEN | port.TIMIEN) & 1U) != 0) {
      if (micros() - started_us >= 7000) {
        ready_ = false;
        break;
      }
    }
    digitalWrite(kDriverEnablePin, LOW);
    digitalWrite(kReceiverEnablePin, LOW);
    return ready_;
  }

 private:
  bool ready_ = false;
};

BusTransmitter bus_transmitter;
#endif

bool TransmitBusPacket(const uint8_t* packet, size_t length) {
#if ROBOSUB_RS485_SERIAL != 0
  return bus_transmitter.Transmit(packet, length);
#else
  (void)packet;
  (void)length;
  return false;
#endif
}

#if ROBOSUB_RS485_PASSTHROUGH

constexpr char kEnableCommand[] = "M5-PASSTHROUGH";
constexpr char kBinaryCommand[] = "M5-BINARY";
constexpr uint8_t kExitPassthrough = 0x1d;  // Ctrl-].
// The menu can arrive in a burst while USB waits for the host to read.
uint8_t passthrough_rx_memory[2048];
bool passthrough_connected = false;
bool passthrough_active = false;
bool passthrough_binary = false;
char enable_line[sizeof(kEnableCommand) - 1];
size_t enable_index = 0;
bool discard_enable_line = false;
bool skip_enable_lf = false;

void PassthroughLoop() {
  if (!Serial) {
    passthrough_connected = false;
    passthrough_active = false;
    passthrough_binary = false;
    enable_index = 0;
    discard_enable_line = false;
    skip_enable_lf = false;
    Serial.clear();
    thruster_serial.clear();
    return;
  }
  if (!passthrough_connected) {
    passthrough_connected = true;
    Serial.println("RS-485 terminal: bounded TX turnaround.");
    Serial.println("Isolate one thruster; stop ROS first.");
    Serial.println(
        "Type M5-PASSTHROUGH then Enter; Ctrl-] disables forwarding.");
    Serial.println(
        "Or M5-BINARY then Enter; close USB to disable binary mode.");
  }
  if (!passthrough_active) {
    thruster_serial.clear();
    for (size_t budget = 0; budget < 64 && Serial.available(); ++budget) {
      const int byte = Serial.read();
      if (byte < 0) {
        break;
      }
      if (byte == '\r' || byte == '\n') {
        passthrough_binary =
            !discard_enable_line &&
            enable_index == sizeof(kBinaryCommand) - 1 &&
            memcmp(enable_line, kBinaryCommand, enable_index) == 0;
        passthrough_active =
            passthrough_binary ||
            (!discard_enable_line &&
             enable_index == sizeof(kEnableCommand) - 1 &&
             memcmp(enable_line, kEnableCommand, enable_index) == 0);
        enable_index = 0;
        discard_enable_line = false;
        if (passthrough_active) {
          skip_enable_lf = byte == '\r';
          if (passthrough_binary) {
            Serial.println(
                "Binary forwarding at 115200 8N1. Close USB to end.");
          } else {
            Serial.println("Forwarding at 115200 8N1. No motor watchdog here.");
          }
          return;
        }
      } else if (!discard_enable_line && enable_index < sizeof(enable_line)) {
        enable_line[enable_index++] = static_cast<char>(byte);
      } else {
        discard_enable_line = true;
      }
    }
    return;
  }

  for (size_t budget = 0; budget < 256 && thruster_serial.available() &&
                          Serial.availableForWrite() > 0;
       ++budget) {
    const int byte = thruster_serial.read();
    if (byte < 0) {
      break;
    }
    Serial.write(static_cast<uint8_t>(byte));
  }
  uint8_t outgoing[32];
  size_t length = 0;
  while (length < sizeof(outgoing) && Serial.available()) {
    const int byte = Serial.read();
    if (byte < 0) {
      break;
    }
    if (skip_enable_lf) {
      skip_enable_lf = false;
      if (byte == '\n') {
        continue;
      }
    }
    if (!passthrough_binary && byte == kExitPassthrough) {
      passthrough_active = false;
      Serial.println("\r\nForwarding disabled. This does not stop the motor.");
      return;
    }
    outgoing[length++] = static_cast<uint8_t>(byte);
  }
  if (length > 0) {
    if (!TransmitBusPacket(outgoing, length)) {
      passthrough_active = false;
      Serial.println("TX timeout: forwarding disabled.");
    }
  }
}

#else

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
uint8_t next_reply_node = 0;
uint8_t capture[48];
size_t capture_length = 0;
uint32_t capture_dropped = 0;
uint32_t last_capture_ms = 0;
bool bus_tx_failed = false;

bool Transmit(const uint8_t* packet, size_t length) {
  if (!TransmitBusPacket(packet, length)) {
    controller.Stop();
    bus_tx_failed = true;
    return false;
  }
  return true;
}

void Drive() {
  if (ROBOSUB_RS485_SERIAL == 0) {
    return;
  }
  uint8_t packet[robosub::kPacketSize];
  // Discover network nodes without assuming they equal the motor slots.
  robosub::MakePacket(controller.pwm(), packet, next_reply_node);
  next_reply_node = static_cast<uint8_t>((next_reply_node + 1U) % 128U);
  if (Transmit(packet, sizeof(packet))) {
    ++packets_sent;
  }
  last_neutral_ms = millis();
}

bool Send(const char* format, ...) {
  char line[robosub::kLineCapacity];
  va_list args;
  va_start(args, format);
  const int length = vsnprintf(line, sizeof(line), format, args);
  va_end(args);
  if (length < 0 || static_cast<size_t>(length) + 10 >= sizeof(line)) {
    return false;
  }
  const uint32_t crc = robosub::Crc32(reinterpret_cast<const uint8_t*>(line),
                                      static_cast<size_t>(length));
  const int suffix = snprintf(line + length, sizeof(line) - length, "*%08lX\n",
                              static_cast<unsigned long>(crc));
  if (suffix != 10) {
    return false;
  }
  const size_t total = static_cast<size_t>(length + suffix);
  // A slow or disconnected USB reader must never delay the watchdog.
  if (Serial && Serial.availableForWrite() >= static_cast<int>(total)) {
    Serial.write(reinterpret_cast<const uint8_t*>(line), total);
    return true;
  }
  return false;
}

void SendCapture() {
  if (capture_length == 0) {
    return;
  }
  char hex[2 * sizeof(capture) + 1];
  constexpr char kDigits[] = "0123456789abcdef";
  for (size_t i = 0; i < capture_length; ++i) {
    hex[2 * i] = kDigits[capture[i] >> 4];
    hex[2 * i + 1] = kDigits[capture[i] & 0x0f];
  }
  hex[2 * capture_length] = '\0';
  if (!Send("R1,%lu,%lu,%s", static_cast<unsigned long>(controller.session()),
            static_cast<unsigned long>(capture_dropped), hex)) {
    capture_dropped += static_cast<uint32_t>(capture_length);
  }
  capture_length = 0;
}

void CaptureByte(int value) {
  capture[capture_length++] = static_cast<uint8_t>(value);
  last_capture_ms = millis();
  if (capture_length == sizeof(capture)) {
    SendCapture();
  }
}

void QueryMotorId() {
  if (ROBOSUB_RS485_SERIAL == 0) {
    return;
  }
  // Read only the uint16 motor ordinal at CSR 0x4c. No payload is written.
  uint8_t packet[10] = {0xf5, 0x5f, controller.query_node(), 0x82, 0x4c, 0};
  robosub::WriteUint32(packet + 6, robosub::Crc32(packet, 6));
  Transmit(packet, sizeof(packet));
}

void Fault(const char* reason) {
  controller.Stop();
  Drive();
  Send("F1,%lu,%s", static_cast<unsigned long>(controller.session()), reason);
}

void HandleLine() {
  input_line[input_length] = '\0';
  const auto accepted = controller.AcceptLine(input_line, millis());
  if (accepted == robosub::Accepted::kQuery) {
    QueryMotorId();
    if (bus_tx_failed) {
      Send("F1,%lu,TX_TIMEOUT",
           static_cast<unsigned long>(controller.session()));
      return;
    }
    Send("K1,%lu,%u", static_cast<unsigned long>(controller.session()),
         static_cast<unsigned int>(controller.query_node()));
    return;
  }
  if (ROBOSUB_RS485_SERIAL == 0 && controller.enabled()) {
    Fault("BUS_DISABLED");
    return;
  }
  Drive();
  if (bus_tx_failed) {
    Send("F1,%lu,TX_TIMEOUT", static_cast<unsigned long>(controller.session()));
    return;
  }
  if (accepted == robosub::Accepted::kHello) {
    Send("I1,%lu,3,8,250,500,%u",
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

#endif  // ROBOSUB_RS485_PASSTHROUGH

}  // namespace

void setup() {
  pinMode(kDriverEnablePin, OUTPUT);
  digitalWrite(kDriverEnablePin, LOW);
  pinMode(kReceiverEnablePin, OUTPUT);
  digitalWrite(kReceiverEnablePin, LOW);
  pinMode(kReceiverOutputPin, INPUT);
  pinMode(kDriverInputPin, INPUT);

#if ROBOSUB_RS485_PASSTHROUGH
  thruster_serial.begin(kBusBaud, SERIAL_8N1);
  thruster_serial.setRX(kReceiverOutputPin);
  bus_transmitter.Begin();
  thruster_serial.addMemoryForRead(passthrough_rx_memory,
                                   sizeof(passthrough_rx_memory));
#else
  pinMode(kPressurePin, INPUT);
  // Receive-only diagnostics on both UARTs; DE remains low in USB-only mode.
  Serial1.begin(kBusBaud);
  Serial2.begin(kBusBaud);
  thruster_serial.setRX(kReceiverOutputPin);
#if ROBOSUB_RS485_SERIAL != 0
  bus_transmitter.Begin();
#endif
  Drive();
#endif
}

void loop() {
#if ROBOSUB_RS485_PASSTHROUGH
  PassthroughLoop();
#else
  const uint32_t now_ms = millis();
  const bool connected = static_cast<bool>(Serial);
  if (!connected && usb_was_connected) {
    controller.Disconnect();
    input_length = 0;
    discard_line = false;
    capture_length = 0;
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
    const int value1 = Serial1.available() ? Serial1.read() : -1;
    if (value1 >= 0) {
      ++serial1_bytes;
      if (ROBOSUB_RS485_SERIAL == 1) {
        ++response_bytes;
        CaptureByte(value1);
      }
    }
    const int value2 = Serial2.available() ? Serial2.read() : -1;
    if (value2 >= 0) {
      ++serial2_bytes;
      if (ROBOSUB_RS485_SERIAL == 2) {
        ++response_bytes;
        CaptureByte(value2);
      }
    }
  }
  if (millis() - last_capture_ms >= 2) {
    SendCapture();
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
#endif
}
