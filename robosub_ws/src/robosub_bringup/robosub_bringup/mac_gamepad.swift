import Foundation
import IOKit.hid

let manager = IOHIDManagerCreate(
  kCFAllocatorDefault, IOOptionBits(kIOHIDOptionsTypeNone))
let matching = [kIOHIDVendorIDKey: 0x045e, kIOHIDProductIDKey: 0x02fd]
IOHIDManagerSetDeviceMatching(manager, matching as CFDictionary)
IOHIDManagerScheduleWithRunLoop(
  manager, CFRunLoopGetCurrent(), CFRunLoopMode.defaultMode.rawValue)

func emit(_ frame: [String: Any]) {
  if let data = try? JSONSerialization.data(withJSONObject: frame) {
    FileHandle.standardOutput.write(data)
    FileHandle.standardOutput.write(Data([10]))
  }
}

func value(
  _ device: IOHIDDevice, _ element: IOHIDElement,
  trigger: Bool = false
) -> Double? {
  let raw: Double? = withUnsafeTemporaryAllocation(
    of: Unmanaged<IOHIDValue>.self, capacity: 1
  ) { buffer in
    guard
      IOHIDDeviceGetValue(device, element, buffer.baseAddress!)
        == kIOReturnSuccess
    else { return nil }
    return Double(
      IOHIDValueGetIntegerValue(
        buffer[0].takeUnretainedValue()))
  }
  guard let raw = raw else { return nil }
  let low = Double(IOHIDElementGetLogicalMin(element))
  let high = Double(IOHIDElementGetLogicalMax(element))
  guard high > low else { return nil }
  let unit = max(0, min(1, (raw - low) / (high - low)))
  return trigger ? unit : 2 * unit - 1
}

func controller() -> [String: Any]? {
  guard
    let devices = IOHIDManagerCopyDevices(manager)
      as? Set<IOHIDDevice>, devices.count == 1,
    let device = devices.first,
    let elements = IOHIDDeviceCopyMatchingElements(
      device, nil, IOOptionBits(kIOHIDOptionsTypeNone))
      as? [IOHIDElement]
  else { return nil }
  var inputs: [String: IOHIDElement] = [:]
  for element in elements {
    let page = IOHIDElementGetUsagePage(element)
    let usage = IOHIDElementGetUsage(element)
    inputs["\(page):\(usage)"] = element
  }
  // This controller's HID descriptor exposes LT/RT as Brake/Accelerator.
  let axes = ["1:48", "1:49", "1:50", "1:53", "2:197", "2:196"]
  var values: [Double] = []
  for (index, key) in axes.enumerated() {
    guard let element = inputs[key],
      let number = value(device, element, trigger: index >= 4)
    else { return nil }
    values.append(number)
  }
  var buttons: [Int] = []
  // Xbox reserves button usage 3; X and Y are usages 4 and 5.
  for usage in [1, 2, 4, 5] {
    guard let element = inputs["9:\(usage)"],
      let number = value(device, element, trigger: true)
    else { return nil }
    buttons.append(number > 0.5 ? 1 : 0)
  }
  return [
    "id": "045e-02fd-Xbox Wireless Controller",
    "axes": values, "buttons": buttons,
  ]
}

let result = IOHIDManagerOpen(manager, IOOptionBits(kIOHIDOptionsTypeNone))
guard result == kIOReturnSuccess else {
  emit([
    "controller": NSNull(),
    "error": "macOS controller access failed: \(result)",
  ])
  exit(1)
}
let timer = Timer(timeInterval: 0.05, repeats: true) { _ in
  emit(["controller": controller() as Any? ?? NSNull()])
}
RunLoop.current.add(timer, forMode: .default)
RunLoop.current.run()
