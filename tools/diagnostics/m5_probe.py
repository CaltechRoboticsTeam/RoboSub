"""Preview or execute read-only M5 voltage requests through passthrough."""

import argparse
import binascii
import json
import math
import os
from pathlib import Path
import struct
import time

from m5_terminal import Console, open_device


def csr_read_request(node, address, size):
    if type(node) is not int or not 0 <= node <= 127:
        raise ValueError("An individual node ID must be between 0 and 127")
    if type(address) is not int or not 0 <= address < 0xF0:
        raise ValueError("Read address must be in the ordinary CSR space")
    if type(size) is not int or not 1 <= size <= min(127, 0xF0 - address):
        raise ValueError("Read size exceeds the ordinary CSR space")
    header = struct.pack("<HBBBB", 0x5FF5, node, 0x80 | size, address, 0)
    # Empty M5 reads end at the header CRC; trailing bytes overlap fast replies.
    packet = header + struct.pack("<I", binascii.crc32(header))
    return packet


def read_request(node):
    # CSR 0x0c and 0x10 are voltage and current. A zero payload writes nothing.
    return csr_read_request(node, 0x0C, 4)


def csr_reply(data, node, address, size):
    offset = 0
    while True:
        start = data.find(b"\xf0\x0f", offset)
        if start < 0 or len(data) - start < 10:
            return None
        offset = start + 1
        header = data[start : start + 6]
        checksum = struct.unpack_from("<I", data, start + 6)[0]
        if checksum != binascii.crc32(header):
            continue
        end = start + 14 + header[5]
        if end > len(data):
            continue
        payload = data[start + 10 : end - 4]
        checksum = struct.unpack_from("<I", data, end - 4)[0]
        if checksum != binascii.crc32(payload):
            continue
        if (
            header[2] != node
            or header[3] != 0x80
            or header[4] != address
            or len(payload) != size + 1
        ):
            continue
        return {"node_id": node, "device_type": payload[0], "data": payload[1:]}


def voltage_reply(data, node, size):
    reply = csr_reply(data, node, 0x0C, size)
    if reply is None:
        return None
    voltage = struct.unpack_from("<f", reply["data"])[0]
    if math.isfinite(voltage):
        return {
            "node_id": node,
            "device_type": reply["device_type"],
            "volts": voltage,
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", required=True)
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--execute", action="store_true")
    options = parser.parse_args()
    requests = {node: read_request(node) for node in range(128)}
    plan = {
        "request": "Read voltage at CSR 0x0c; optional current at 0x10",
        "write_payload_bytes": 0,
        "node_ids": list(requests),
        "execute": options.execute,
    }
    print(json.dumps(plan, indent=2), flush=True)
    if not options.execute:
        return
    print(
        "Use only with one isolated M5, power-cycled into normal mode. "
        "Wait five seconds after power-up; do not send '+' first.",
        flush=True,
    )
    descriptor = open_device(options.device)
    try:
        with options.log.open("a", encoding="utf-8") as log:
            console = Console(descriptor, log)
            console.enable(binary=True)
            for node, packet in requests.items():
                console.send(packet)
                received = bytearray()
                deadline = time.monotonic() + 0.1
                while time.monotonic() < deadline:
                    received.extend(
                        console.receive(max(0.0, deadline - time.monotonic()))
                    )
                    if len(received) > 4096:
                        raise RuntimeError("Unexpected continuous RX data")
                    reply = voltage_reply(received, node, packet[3] & 0x7F)
                    if reply is not None:
                        print("VALID M5 REPLY: " + json.dumps(reply))
                        return
            print("No valid reply from Node IDs 0 through 127.")
    finally:
        os.close(descriptor)


if __name__ == "__main__":
    main()
