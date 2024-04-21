import asyncio
import serial
import threading


# Constants for the packet protocol
PACKET_LENGTH_BYTES = 1
PACKET_DATA_BYTES = 16
PACKET_CRC_BYTES = 1
PACKET_CRC_INDEX = PACKET_LENGTH_BYTES + PACKET_DATA_BYTES
PACKET_LENGTH = PACKET_LENGTH_BYTES + PACKET_DATA_BYTES + PACKET_CRC_BYTES

PACKET_ACK_DATA0 = 0x15
PACKET_RETX_DATA0 = 0x19

# Details about the serial port connection
SERIAL_PATH = "/dev/ttyACM0"
SERIAL_BAUDRATE = 115200

# CRC8 implementation
def crc8(data):
    crc = 0
    for byte in data:
        crc = (crc ^ byte) & 0xff
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xff
            else:
                crc = (crc << 1) & 0xff
    return crc

# Async delay function, which gives the event loop time to process outside input
async def delay(ms):
    await asyncio.sleep(ms / 1000)

# Class for serializing and deserializing packets
class Packet:
    retx = None
    ack = None

    @classmethod
    def create_packet(cls, length, data):
        if cls.retx is None:
            cls.retx = cls(1, bytes([PACKET_RETX_DATA0])).to_buffer()
        if cls.ack is None:
            cls.ack = cls(1, bytes([PACKET_ACK_DATA0])).to_buffer()

    def __init__(self, length, data, crc=None):
        self.length = length
        self.data = data + bytes([0xff] * (PACKET_DATA_BYTES - len(data)))

        if crc is None:
            self.crc = self.compute_crc()
        else:
            self.crc = crc

    def compute_crc(self):
        all_data = [self.length] + list(self.data)
        return crc8(all_data)

    def to_buffer(self):
        return bytes([self.length]) + self.data + bytes([self.crc])

    def is_single_byte_packet(self, byte):
        if self.length != 1:
            return False
        if self.data[0] != byte:
            return False
        for i in range(1, PACKET_DATA_BYTES):
            if self.data[i] != 0xff:
                return False
        return True

    def is_ack(self):
        return self.is_single_byte_packet(PACKET_ACK_DATA0)

    def is_retx(self):
        return self.is_single_byte_packet(PACKET_RETX_DATA0)

    def __str__(self):
        data_str = ' '.join(f"{byte:02X}" for byte in self.data)
        return f"Packet[length={self.length}, data={data_str}, crc={self.crc:02X}]"

# Serial port instance
uart = serial.Serial(SERIAL_PATH, SERIAL_BAUDRATE)

# Packet buffer
packets = []

# Last packet buffer
last_packet = Packet.ack

def write_packet(packet):
    uart.write(packet)
    global last_packet
    last_packet = packet

# Serial data buffer, with a splice-like function for consuming data
rx_buffer = bytearray()
def consume_from_buffer(n):
    global rx_buffer
    consumed = rx_buffer[:n]
    rx_buffer = rx_buffer[n:]
    return consumed

# This function fires whenever data is received over the serial port. The whole
# packet state machine runs here.
async def handle_serial_data():
    while True:
        # # Wait until at least 18 bytes are available
        # while uart.in_waiting < 18:
        #     pass  # Wait until enough data is available

        # Read the entire packet of 18 bytes
        data = uart.read(uart.in_waiting or 1)
        if data:
            print(f"Received {len(data)} bytes through uart")
            # hex_data = ' '.join([f"{byte:02X}" for byte in data])
            # print("Data received:", hex_data)
            rx_buffer.extend(data)

            # Can we build a packet?
            while len(rx_buffer) >= PACKET_LENGTH:
                print("Building a packet")
                raw = consume_from_buffer(PACKET_LENGTH)
                packet = Packet(raw[0], raw[1:1 + PACKET_DATA_BYTES], raw[PACKET_CRC_INDEX])
                computed_crc = packet.compute_crc()

                # Need retransmission?
                if packet.crc != computed_crc:
                    print(f"CRC failed, computed 0x{computed_crc:02X}, got 0x{packet.crc:02X}")
                    write_packet(Packet.retx)
                    continue

                # Are we being asked to retransmit?
                if packet.is_retx():
                    print("Retransmitting last packet")
                    write_packet(last_packet)
                    continue

                # If this is an ack, move on
                if packet.is_ack():
                    print("It was an ack, nothing to do")
                    continue

                # Otherwise write the packet in to the buffer, and send an ack
                print("Storing packet and ack'ing")
                packets.append(packet)
                write_packet(Packet.ack)
        await asyncio.sleep(0.1)

# Function to allow us to await a packet
async def wait_for_packet():
    while not packets:
        await delay(1)
    return packets.pop(0)

# Initialize Packet.ack and Packet.retx
Packet.create_packet(1, bytes([PACKET_ACK_DATA0]))
Packet.create_packet(1, bytes([PACKET_RETX_DATA0]))


# Do everything in an async function so we can have loops, awaits etc
async def main():
    # Start the serial data handling coroutine
    serial_task = asyncio.create_task(handle_serial_data())
    print('Waiting for packet...')
    while True:
        packet = await wait_for_packet()
        print(packet)

        packet_to_send = Packet(4, bytes([5, 6, 7, 8]))
        packet_to_send.crc += 1
        # write_packet(packet_to_send)
        uart.write(packet_to_send.to_buffer())

if __name__ == "__main__":
    asyncio.run(main())