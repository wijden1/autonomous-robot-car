"""Encode a CAN frame with the DBC file, decode it again, and send it on vcan0."""
import os
import sys

import cantools

db = cantools.database.load_file(os.path.join(os.path.dirname(__file__), "robot_car.dbc"))

print("Messages in robot_car.dbc:")
for msg in db.messages:
    signals = ", ".join(f"{s.name} [{s.unit or '-'}]" for s in msg.signals)
    print(f"  0x{msg.frame_id:03X} {msg.name:<20} {msg.length} bytes: {signals}")

cmd = db.get_message_by_name("WHEEL_CMD")
data = cmd.encode({"SetpointLeft": 1000, "SetpointRight": -250, "AliveCounter": 7})
print(f"\nEncoded WHEEL_CMD: ID 0x{cmd.frame_id:03X}  data {data.hex(' ').upper()}")
print("Decoded again:    ", db.decode_message(cmd.frame_id, data))

if "--send" in sys.argv:
    import can
    with can.Bus(interface="socketcan", channel="vcan0") as bus:
        bus.send(can.Message(arbitration_id=cmd.frame_id, data=data, is_extended_id=False))
    print("Sent on vcan0")