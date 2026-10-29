#ifndef ROBOSUB_TEENSY_USB_M5_HARDWARE_CONFIG_H_
#define ROBOSUB_TEENSY_USB_M5_HARDWARE_CONFIG_H_

// 0: USB/depth diagnostics only; 1: Serial1 bus; 2: Serial2 bus.
// The operator confirmed UART1 for the installed RS-485 module. The default
// remains USB-only; build rs485-serial1 explicitly to enable bus transmission.
#ifndef ROBOSUB_RS485_SERIAL
#define ROBOSUB_RS485_SERIAL 0
#endif

#if ROBOSUB_RS485_SERIAL < 0 || ROBOSUB_RS485_SERIAL > 2
#error "ROBOSUB_RS485_SERIAL must be 0, 1, or 2"
#endif

#endif  // ROBOSUB_TEENSY_USB_M5_HARDWARE_CONFIG_H_
