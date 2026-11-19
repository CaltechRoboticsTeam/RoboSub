import Foundation
import IOKit.hid

let manager = IOHIDManagerCreate(
  kCFAllocatorDefault, IOOptionBits(kIOHIDOptionsTypeNone))
let matching = [
  [kIOHIDVendorIDKey: 0x045e, kIOHIDProductIDKey: 0x02fd],
  [
    kIOHIDVendorIDKey: 0x045e, kIOHIDProductIDKey: 0x028e,
    kIOHIDTransportKey: "USB",
  ] as [String: Any],
]
IOHIDManagerSetDeviceMatchingMultiple(manager, matching as CFArray)
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

func controller(_ device: IOHIDDevice) -> [String: Any]? {
  guard
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
  let product = IOHIDDeviceGetProperty(
    device, kIOHIDProductIDKey as CFString) as? Int
  let wired = product == 0x028e
  // The wired descriptor uses Ry/Rx for right X/Y and Z/Rz for triggers.
  let axes = wired
    ? ["1:48", "1:49", "1:52", "1:51", "1:50", "1:53"]
    : ["1:48", "1:49", "1:50", "1:53", "2:197", "2:196"]
  var values: [Double] = []
  for (index, key) in axes.enumerated() {
    guard let element = inputs[key],
      let number = value(device, element, trigger: index >= 4)
    else { return nil }
    let invert = wired && (index == 1 || index == 3)
    values.append(invert ? -number : number)
  }
  var buttons: [Int] = []
  // The two descriptors assign different button usages to the same controls.
  let buttonKeys = wired
    ? ["9:1", "9:2", "9:3", "9:4", "9:5", "9:6", "9:10", "9:9"]
    : ["9:1", "9:2", "9:4", "9:5", "9:7", "9:8", "12:548", "9:12"]
  for key in buttonKeys {
    guard let element = inputs[key],
      let number = value(device, element, trigger: true)
    else { return nil }
    buttons.append(number > 0.5 ? 1 : 0)
  }
  return [
    "id": wired
      ? "045e-028e-Xbox USB Controller"
      : "045e-02fd-Xbox Wireless Controller",
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
  let devices = IOHIDManagerCopyDevices(manager) as? Set<IOHIDDevice> ?? []
  let frames = devices.compactMap { controller($0) }
  var frame: [String: Any] = [
    "controllers": frames,
    "controller": frames.count == 1 ? frames[0] : NSNull(),
  ]
  if frames.isEmpty {
    frame["error"] = devices.isEmpty
      ? "No supported Xbox controller detected on this Mac"
      : "Xbox detected, but required axes or buttons could not be read"
  }
  emit(frame)
}
RunLoop.current.add(timer, forMode: .default)
RunLoop.current.run()
