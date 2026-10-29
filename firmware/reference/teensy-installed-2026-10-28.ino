// TeensySensorThrusterBreakOutBoard
//
// Reads SERVO_OUTPUT_RAW from the Pixhawk (TELEM1) and drives the thrusters over
// RS485 using the VideoRay VRCSR propulsion protocol.
//
// Changes from the previous version:
//   * The servo stream is re-requested periodically until it actually arrives,
//     and again if it stalls, instead of once at boot. This is the main fix for
//     thrusters that spin sometimes and not others.
//   * A stream watchdog commands zero thrust if SERVO_OUTPUT_RAW stops, so the
//     thrusters cannot latch on stale data.
//   * Depth is sent to the flight controller as a NAMED_VALUE_FLOAT ("DEPTH"),
//     which is pure telemetry that PX4 never fuses, so it cannot compete with
//     the DVL. The Jetson DVL node picks it up (via MAVROS) and folds it into
//     the single ODOMETRY message as the Z position. The two readings are then
//     merged by the EKF: DVL for motion, depth for absolute height. This needs
//     MAV_x_FORWARD = 1 on the TELEM port so the message reaches MAVROS.
//     The old competing ODOMETRY send is gone.
//   * Debug prints are rate limited so they can't slow the control loop.
//   * Thrust is clamped and out-of-range PWM is treated as neutral.

#include <Arduino.h>
#include <MAVLink.h>
#include <ArduinoRS485.h>
#include <CRC32.h>

using namespace std;

// ---- RS485 / thruster protocol -------------------------------------------
#define DE 3
#define RE 2

uint16_t SYNC_REQUEST  = 0x5FF5;
uint8_t  RESPONSE_THRUSTER_STANDARD = 0x2;
uint8_t  PROPULSION_COMMAND = 0xAA;
uint8_t  ADDR_CUSTOM_COMMAND = 0xF0;
uint8_t  NODE_ID = 0x81;
uint8_t  MOTOR_ID = 0x01;
const int NUM_THRUSTERS = 8;
const int HEADER_LENGTH = 6;
const int PAYLOAD_LENGTH = 2 + 4 * NUM_THRUSTERS;
const int CHECKSUM_LENGTH = 4;
const int PACKET_LENGTH = HEADER_LENGTH + CHECKSUM_LENGTH + PAYLOAD_LENGTH + CHECKSUM_LENGTH;

// ---- MAVLink identity -----------------------------------------------------
const uint8_t system_id = 1;
const uint8_t component_id = MAV_COMP_ID_ONBOARD_COMPUTER;

// ---- Depth sensor ---------------------------------------------------------
// Sent as NAMED_VALUE_FLOAT "DEPTH" (telemetry only, never fused by PX4). The
// Jetson DVL node merges it into the odometry Z. Set to 0 to disable entirely.
#define SEND_DEPTH 1
const char DEPTH_NAME[10] = "DEPTH";     // must match the DVL node's depth_key
const int sensorPin = A0;
const unsigned long DEPTH_INTERVAL = 100; // ms between depth sends (10 Hz)
unsigned long lastDepthTime = 0;

// ---- Timing ---------------------------------------------------------------
const unsigned long REQUEST_INTERVAL = 1000; // ms between stream re-requests while unhealthy
const unsigned long STREAM_TIMEOUT   = 500;  // ms without a servo msg => stream lost
const unsigned long DEBUG_INTERVAL   = 200;  // ms between debug prints (5 Hz)

unsigned long lastRequestTime = 0;
unsigned long lastServoTime   = 0;   // last time a SERVO_OUTPUT_RAW arrived
unsigned long lastDebugTime   = 0;
bool streamHealthy = false;

HardwareSerial &RS485Serial = Serial1;
HardwareSerial &MAVSerial   = Serial2;

struct __attribute__((packed)) Header {
  uint16_t sync;
  uint8_t node_id;
  uint8_t flag;
  uint8_t csr_addr;
  uint8_t payload_length;
};

struct __attribute__((packed)) Payload {
  uint8_t propulsion;
  uint8_t motor_id;
  float thrusts[NUM_THRUSTERS] = {0};
};

// Ask PX4 to stream SERVO_OUTPUT_RAW (msg 36) at 50 Hz on this link.
void request_servo_output_stream() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_command_long_pack(
    system_id, component_id, &msg,
    1,                             // target system: Pixhawk
    MAV_COMP_ID_AUTOPILOT1,        // target component
    MAV_CMD_SET_MESSAGE_INTERVAL,  // 511
    0,                             // confirmation
    MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, // param1: message id (36)
    20000,                         // param2: interval us (20 ms = 50 Hz)
    0, 0, 0, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  MAVSerial.write(buf, len);
  MAVSerial.flush();
}

// Read the pressure sensor and return depth in metres (positive = deeper).
float read_depth_m() {
  int adcValue = analogRead(sensorPin);
  // Linear fit of ADC -> depth (cm). Recalibrate as needed.
  const double m = 30.48 / (415.0 - 403.0);
  const double b = 30.48 - m * 415.0;
  return (float)((m * adcValue + b) / 100.0);
}

// Send depth as NAMED_VALUE_FLOAT. Telemetry only: PX4 does not fuse it, so it
// never competes with the DVL. The Jetson DVL node reads it and merges it.
void send_depth(float depth_m) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_named_value_float_pack(
    system_id, component_id, &msg, millis(), DEPTH_NAME, depth_m);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  MAVSerial.write(buf, len);
  MAVSerial.flush();
}

void make_thrust_packet(uint8_t* packet, float thrust_values[NUM_THRUSTERS]) {
  Header hd;
  hd.sync = SYNC_REQUEST;
  hd.node_id = NODE_ID;
  hd.flag = RESPONSE_THRUSTER_STANDARD;
  hd.csr_addr = ADDR_CUSTOM_COMMAND;
  hd.payload_length = 2 + NUM_THRUSTERS * 4;

  Payload py;
  py.propulsion = PROPULSION_COMMAND;
  py.motor_id = MOTOR_ID;
  memcpy(py.thrusts, thrust_values, NUM_THRUSTERS * sizeof(float));

  uint8_t header_buf[HEADER_LENGTH];
  uint8_t payload_buf[PAYLOAD_LENGTH];
  memcpy(header_buf, &hd, HEADER_LENGTH);
  memcpy(payload_buf, &py, PAYLOAD_LENGTH);

  CRC32 crc;
  for (int i = 0; i < HEADER_LENGTH; i++) crc.update(header_buf[i]);
  uint32_t hcrc = crc.finalize();
  crc.reset();
  for (int i = 0; i < PAYLOAD_LENGTH; i++) crc.update(payload_buf[i]);
  uint32_t pcrc = crc.finalize();
  crc.reset();

  memcpy(packet, header_buf, HEADER_LENGTH);
  memcpy(packet + HEADER_LENGTH, &hcrc, CHECKSUM_LENGTH);
  memcpy(packet + HEADER_LENGTH + CHECKSUM_LENGTH, payload_buf, PAYLOAD_LENGTH);
  memcpy(packet + HEADER_LENGTH + CHECKSUM_LENGTH + PAYLOAD_LENGTH, &pcrc, CHECKSUM_LENGTH);
}

void drive(float thrust_values[NUM_THRUSTERS]) {
  uint8_t packet[PACKET_LENGTH];
  make_thrust_packet(packet, thrust_values);
  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);
  RS485Serial.write(packet, PACKET_LENGTH);
  RS485Serial.flush();          // wait for the last byte before releasing the driver
  digitalWrite(DE, LOW);
  digitalWrite(RE, LOW);
}

void drive_zero() {
  float zero[NUM_THRUSTERS] = {0};
  drive(zero);
}

// Convert a servo PWM (us) to a thrust command in [-1, 1].
// Out-of-range values (e.g. an unpopulated channel reading 0) are treated as
// neutral so a bad channel can never command a huge thrust.
float pwm_to_thrust(int pwm_val) {
  if (pwm_val < 1000 || pwm_val > 2000) return 0.0f;
  const float max_pwm = 2000.0f, min_pwm = 1000.0f;
  float t = 1.0f - 2.0f * ((max_pwm - pwm_val) / (max_pwm - min_pwm)); // 1500->0, 2000->+1
  if (t > 1.0f) t = 1.0f;
  if (t < -1.0f) t = -1.0f;
  return t;
}

void setup() {
  Serial.begin(115200);
  MAVSerial.begin(115200);
  RS485Serial.begin(115200);
  pinMode(DE, OUTPUT);
  pinMode(RE, OUTPUT);
  digitalWrite(DE, LOW);
  digitalWrite(RE, LOW);
  pinMode(sensorPin, INPUT);
  delay(1000);

  request_servo_output_stream();
  lastRequestTime = millis();
}

void loop() {
  const unsigned long nowMs = millis();

  // 1) Keep the servo stream alive: re-request while it is not healthy.
  if (!streamHealthy && (nowMs - lastRequestTime >= REQUEST_INTERVAL)) {
    request_servo_output_stream();
    lastRequestTime = nowMs;
  }

  // 2) Read MAVLink; drive thrusters on each SERVO_OUTPUT_RAW.
  mavlink_message_t msg;
  mavlink_status_t status;
  while (MAVSerial.available() > 0) {
    Serial.println("Pixhawk Comms Available");
    uint8_t c = MAVSerial.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      if (msg.msgid == MAVLINK_MSG_ID_SERVO_OUTPUT_RAW) {
        mavlink_servo_output_raw_t servo;
        mavlink_msg_servo_output_raw_decode(&msg, &servo);

        uint16_t pwm[NUM_THRUSTERS] = {
          servo.servo1_raw, servo.servo2_raw, servo.servo3_raw, servo.servo4_raw,
          servo.servo5_raw, servo.servo6_raw, servo.servo7_raw, servo.servo8_raw
        };
        float thrusts[NUM_THRUSTERS];
        for (int i = 0; i < NUM_THRUSTERS; i++) thrusts[i] = pwm_to_thrust(pwm[i]);
        drive(thrusts);

        lastServoTime = nowMs;
        streamHealthy = true;
      }
    }
  }

  // 3) Watchdog: if the stream stops, stop the thrusters and re-request.
  if (streamHealthy && (nowMs - lastServoTime > STREAM_TIMEOUT)) {
    streamHealthy = false;
    drive_zero();
    request_servo_output_stream();
    lastRequestTime = nowMs;
  }

  // 4) Send depth to the FCU at 10 Hz (telemetry only; merged by the DVL node).
#if SEND_DEPTH
  if (nowMs - lastDepthTime >= DEPTH_INTERVAL) {
    lastDepthTime = nowMs;
    send_depth(read_depth_m());
  }
#endif

  // 5) Rate-limited debug.
  if (nowMs - lastDebugTime >= DEBUG_INTERVAL) {
    lastDebugTime = nowMs;
    Serial.print("stream=");
    Serial.print(streamHealthy ? "OK" : "WAIT");
    Serial.print("  depth=");
    Serial.print(read_depth_m(), 2);
    Serial.println(" m");
  }
}