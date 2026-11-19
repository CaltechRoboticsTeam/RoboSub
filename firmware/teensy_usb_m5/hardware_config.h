#ifndef ROBOSUB_TEENSY_USB_M5_HARDWARE_CONFIG_H_
#define ROBOSUB_TEENSY_USB_M5_HARDWARE_CONFIG_H_

// 0: USB/depth diagnostics only; 1: Serial1 RX; 2: Serial2 RX.
// The selected UART receives RO on pin 2 via XBAR; FlexIO transmits DI on
// pin 3. Build rs485-serial1 explicitly to enable bus transmission.
#ifndef ROBOSUB_RS485_SERIAL
#define ROBOSUB_RS485_SERIAL 0
#endif

#if ROBOSUB_RS485_SERIAL < 0 || ROBOSUB_RS485_SERIAL > 2
#error "ROBOSUB_RS485_SERIAL must be 0, 1, or 2"
#endif

#ifndef ROBOSUB_RS485_PASSTHROUGH
#define ROBOSUB_RS485_PASSTHROUGH 0
#endif

#if ROBOSUB_RS485_PASSTHROUGH != 0 && ROBOSUB_RS485_PASSTHROUGH != 1
#error "ROBOSUB_RS485_PASSTHROUGH must be 0 or 1"
#endif

#if ROBOSUB_RS485_PASSTHROUGH && ROBOSUB_RS485_SERIAL == 0
#error "RS-485 passthrough requires an explicitly selected UART"
#endif

#endif  // ROBOSUB_TEENSY_USB_M5_HARDWARE_CONFIG_H_
