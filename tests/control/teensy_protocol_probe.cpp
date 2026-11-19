// Exercise the actual firmware parser and packet builder without hardware.
#include <stdint.h>

#include <iomanip>
#include <iostream>
#include <string>

#include "protocol.h"

int main() {
  robosub::UsbController controller;
  uint32_t timestamp;
  std::string line;
  while (std::cin >> timestamp >> line) {
    int accepted = -1;
    if (line == "TICK") {
      accepted = controller.PollWatchdog(timestamp) ? 1 : 0;
    } else if (line == "DISCONNECT") {
      controller.Disconnect();
    } else {
      if (line.size() >= robosub::kLineCapacity) {
        return 2;
      }
      char buffer[robosub::kLineCapacity];
      line.copy(buffer, line.size());
      buffer[line.size()] = '\0';
      accepted = static_cast<int>(controller.AcceptLine(buffer, timestamp));
    }
    std::cout << accepted << ' ' << controller.session() << ' '
              << controller.sequence() << ' ' << controller.enabled();
    for (uint16_t value : controller.pwm()) {
      std::cout << ' ' << value;
    }
    uint8_t packet[robosub::kPacketSize];
    robosub::MakePacket(controller.pwm(), packet);
    std::cout << ' ';
    for (uint8_t byte : packet) {
      std::cout << std::hex << std::setw(2) << std::setfill('0')
                << static_cast<unsigned int>(byte);
    }
    std::cout << std::dec << '\n';
  }
  return 0;
}
