from __future__ import print_function

import datetime
import os
import time
import math
from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("--port", default=None, help="serial port", required=True)
parser.add_argument("--baud", default=115200, help="serial baudrate", type=int)
parser.add_argument("firmware", metavar="FIRMWARE", nargs=1)

args = parser.parse_args()

from pymavlink import mavutil

print ("Connecting to %s at %u" % (args.port, args.baud))
mlog = mavutil.mavlink_connection(args.port, baud=args.baud)


# ublox checksum
def ubx_checksum(data32):
    CK_A = 0
    CK_B = 0
    for i in range(0, len(data32)):
        CK_A = (CK_A + data32[i]) & 0xFFFFFFFF
        CK_B = (CK_B + CK_A) & 0xFFFFFFFF
    return CK_A, CK_B

def progress_bar(progress, width=50):
    bar = int(progress * width)
    print("\r[{0}{1}] {2:.0f}%".format('#' * bar, ' ' * (width - bar), progress * 100), end='')


# calculate the size of the firmware
firmware_size = os.stat(args.firmware[0]).st_size
firmware_data = open(args.firmware[0], 'rb').read()

# convert firmware_data to 32bit ints
firmware_data32 = []
for i in range(0, len(firmware_data), 4):
    firmware_data32.append(int.from_bytes(firmware_data[i:i+4], byteorder='little'))

# calculate the checksum
ck_a, ck_b = ubx_checksum(firmware_data32)

print("Firmware size: %u bytes" % firmware_size)
print("Firmware Checksum: 0x%x 0x%x" % (ck_a, ck_b))

# wait for the heartbeat msg to find the system ID
print("Waiting for heartbeat")
mlog.wait_heartbeat(blocking=True)

# Enter UBX Safeboot mode
retries = 5
print("Entering UBX Safeboot mode")
while True:
    mlog.mav.ubx_enter_exit_safeboot_send(mlog.target_system, mlog.target_component, 1)

    m = mlog.recv_match(type=['UBX_FW_RESP'], blocking=True, timeout=3)

    if m is not None:
        print("Safeboot response: ", "SAFEBOOT_OK" if m.result else "SAFEBOOT_ERROR")
        break
    elif retries == 0:
        print("Safeboot response: TIMEOUT")
        exit(1)
    retries -= 1

print("Get Firmware Info")
retries = 5
while True:
    # send the firmware size and checksum
    mlog.mav.ubx_fw_info_send(mlog.target_system, mlog.target_component,
                            1, # version 
                            0x00000048, # address
                            firmware_size,
                            ck_a,
                            ck_b)
    m = mlog.recv_match(type=['UBX_FW_RESP'], blocking=True, timeout=3)

    if m is not None:
        print("Firmware info response: ", "FW_MATCHED" if m.result else "FW_MISMATCH")
        break
    elif retries == 0:
        print("Firmware info response: TIMEOUT")
        exit(1)
    retries -= 1

print("Erasing flash")
start_address = 0x00000048
end_address = 0x00000048 + firmware_size
num_blocks = 512
block_index = 0
while True:
    mlog.mav.ubx_fw_erase_block_send(mlog.target_system, mlog.target_component, start_address + (block_index * 0x1000))
    m = mlog.recv_match(type=['UBX_FW_RESP'], blocking=True, timeout=3)
    if m is None:
        continue
    if m.result != 1:
        print("Erase error: %u" % m.result)
        continue
    progress_bar(block_index / num_blocks)
    block_index += 1
    if block_index >= num_blocks:
        break

print("\nFlash erase complete")

print("Writing flash")
offset = 0
seq = 0
num_words = len(firmware_data32)
# <field type="uint32_t" name="addr">Address</field>
# <field type="uint32_t" name="size">Size</field>
# <field type="uint8_t" name="seq">Sequence number out of 4 chunks</field>
# <field type="uint32_t[32]" name="data">Firmware data block</field>

while True:
    address = start_address + offset
    size = min(firmware_size - offset, 512)
    for i in range(0, 4):
        data32_start_index = offset//4 + (i*32)
        data32_end_index = data32_start_index + 32
        firmware_data32 
        mlog.mav.ubx_fw_write_block_send(mlog.target_system, mlog.target_component,
                                        start_address + offset,
                                        size,
                                        i,
                                        firmware_data32[data32_start_index:data32_end_index])
    m = mlog.recv_match(type=['UBX_FW_RESP'], blocking=True, timeout=3)
    if m is None:
        continue
    if m.result != 1:
        print("Write error: %u" % m.result)
        continue
    progress_bar(offset / firmware_size)
    offset += 512
    if offset >= firmware_size:
        break