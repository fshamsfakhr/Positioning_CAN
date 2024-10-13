# """
# ===================================================================
# |                                                                 |
# |  COPYRIGHT NOTICE                                               |
# |  Developer: Farhad Shamsfakhr, PhD                              |
# |  EMAIL : fshamsfakhr@gmail.com                                  | 
# |  © [2024] Farhad Shamsfakhr, PhD.                               |
# |                                                                 |
# ===================================================================
# """
import can
import struct

def receive_can_messages():
    # Setup the CAN bus
    bus = can.interface.Bus(channel='can0', bustype='socketcan')

    print("Listening for CAN messages...")

    # Loop to continuously receive messages
    while True:
        messages = {}
        while len(messages) < 3:
            msg = bus.recv()
            if msg is not None:
                print("Received:", msg)
                messages[msg.arbitration_id] = msg.data

        # Combine data from the messages based on arbitration ID
        if 0x123 in messages and 0x124 in messages and 0x125 in messages:
            data = messages[0x123] + messages[0x124] + messages[0x125]

            # Ensure we have exactly 24 bytes of data
            if len(data) == 24:
                # Unpack the data (6 floats)
                all_numbers = struct.unpack('6f', data)

                # Split the unpacked data into two arrays of three floats each
                numbers1 = all_numbers[:3]
                numbers2 = all_numbers[3:]

                print("Decoded Position:", numbers1)
                print("Decoded Attitude:", numbers2)
            else:
                print("Unexpected data length:", len(data))
        else:
            print("Missing CAN messages")

receive_can_messages()
