"""
Minimal Python 3 implementation of the s3g protocol subset needed to drive
a Sailfish-firmware Replicator 2/2X: homing, recall-home, and queued moves.

The upstream `makerbot_driver` package (https://github.com/makerbot/s3g) only
supports Python 2 in practice: the PyPI release refuses to install on
Python 3, and the GitHub source's Python 3 build path depends on
`distutils.command.build_py_2to3`, which was removed from setuptools years
ago. Rather than depend on a package that no longer builds, this module
reimplements the handful of s3g commands this project actually sends.

Protocol reference: https://github.com/makerbot/s3g/blob/master/doc/s3gProtocol.markdown
"""

import struct
import time

import serial

HEADER = 0xD5
MAX_PAYLOAD_LENGTH = 32
MAX_RETRY_COUNT = 5
RESPONSE_TIMEOUT_SEC = 1.0

RESPONSE_SUCCESS = 0x81
RETRYABLE_RESPONSE_CODES = {
    0x80,  # GENERIC_PACKET_ERROR
    0x83,  # CRC_MISMATCH
}

CMD_GET_EXTENDED_POSITION = 21
CMD_EXTENDED_STOP = 22
CMD_FIND_AXES_MINIMUMS = 131
CMD_FIND_AXES_MAXIMUMS = 132
CMD_QUEUE_EXTENDED_POINT = 139
CMD_RECALL_HOME_POSITIONS = 144

EXTENDED_STOP_HALT_STEPPERS = 0x01
EXTENDED_STOP_CLEAR_QUEUE = 0x02

# Response payload (after the 1-byte response code): 5x int32 step position
# (x, y, z, a, b) + a uint16 endstop status bitfield.
_GET_EXTENDED_POSITION_FMT = '<iiiiiH'

AXIS_BITS = {'x': 0x01, 'y': 0x02, 'z': 0x04, 'a': 0x08, 'b': 0x10}

# iButton/Maxim CRC-8 table (same table used by the s3g protocol).
_CRC_TABLE = [
    0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
    219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
    17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53,
]


def _crc8(data):
    val = 0
    for byte in data:
        val = _CRC_TABLE[val ^ byte]
    return val


def _encode_axes(axes):
    bitfield = 0
    for axis in axes:
        bitfield |= AXIS_BITS[axis.lower()]
    return bitfield


def _encode_packet(payload):
    packet = bytearray()
    packet.append(HEADER)
    packet.append(len(payload))
    packet.extend(payload)
    packet.append(_crc8(payload))
    return bytes(packet)


class TransmissionError(IOError):
    """Raised when the s3g link exhausts its retries or reports a fatal error."""


class S3GClient(object):
    """Serial-port client speaking the subset of the s3g protocol this tool needs."""

    def __init__(self, ser):
        self.ser = ser

    def _send_command(self, payload):
        """Send one packet, wait for the ack, retrying on transient errors."""
        packet = _encode_packet(payload)
        received_errors = []

        for attempt in range(MAX_RETRY_COUNT):
            self.ser.write(packet)
            self.ser.flush()

            try:
                response = self._read_response()
            except TransmissionError as e:
                received_errors.append(str(e))
                continue

            response_code = response[0]
            if response_code == RESPONSE_SUCCESS:
                return response
            if response_code in RETRYABLE_RESPONSE_CODES:
                received_errors.append("response_code=0x%02x" % response_code)
                continue
            raise TransmissionError(
                "Machine reported non-retryable response code 0x%02x" % response_code
            )

        raise TransmissionError(received_errors)

    def _read_response(self):
        """Read and validate one framed packet from the serial link."""
        start_time = time.time()

        def read_byte():
            while True:
                if time.time() - start_time > RESPONSE_TIMEOUT_SEC:
                    raise TransmissionError("Timed out waiting for response")
                data = self.ser.read(1)
                if data:
                    return data[0]

        header = read_byte()
        if header != HEADER:
            raise TransmissionError("Bad response header: 0x%02x" % header)

        length = read_byte()
        if length > MAX_PAYLOAD_LENGTH:
            raise TransmissionError("Response length field too large: %d" % length)

        payload = bytearray(read_byte() for _ in range(length))

        crc = read_byte()
        if _crc8(payload) != crc:
            raise TransmissionError("CRC mismatch in response")

        return payload

    def find_axes_minimums(self, axes, rate, timeout):
        payload = struct.pack('<BBIH', CMD_FIND_AXES_MINIMUMS, _encode_axes(axes), rate, timeout)
        self._send_command(payload)

    def find_axes_maximums(self, axes, rate, timeout):
        payload = struct.pack('<BBIH', CMD_FIND_AXES_MAXIMUMS, _encode_axes(axes), rate, timeout)
        self._send_command(payload)

    def recall_home_positions(self, axes):
        payload = struct.pack('<BB', CMD_RECALL_HOME_POSITIONS, _encode_axes(axes))
        self._send_command(payload)

    def queue_extended_point(self, position, dda_speed):
        """position: 5 ints (x, y, z, a, b) in steps. dda_speed: microseconds per step."""
        payload = struct.pack('<BiiiiiI', CMD_QUEUE_EXTENDED_POINT, *position, dda_speed)
        self._send_command(payload)

    def get_extended_position(self):
        """Query the firmware's current step position for all 5 axes plus
        the endstop status bitfield. Returns (x, y, z, a, b, endstop_bits).
        """
        payload = struct.pack('<B', CMD_GET_EXTENDED_POSITION)
        response = self._send_command(payload)
        size = struct.calcsize(_GET_EXTENDED_POSITION_FMT)
        data = bytes(response[1:1 + size])
        return struct.unpack(_GET_EXTENDED_POSITION_FMT, data)

    def extended_stop(self, halt_steppers=True, clear_queue=True):
        """Immediately halt stepper motion and/or clear the queued command
        buffer, without a full firmware reset (unlike 'abort immediately').
        """
        bitfield = 0
        if halt_steppers:
            bitfield |= EXTENDED_STOP_HALT_STEPPERS
        if clear_queue:
            bitfield |= EXTENDED_STOP_CLEAR_QUEUE
        payload = struct.pack('<BB', CMD_EXTENDED_STOP, bitfield)
        self._send_command(payload)


def open_connection(port, baud, timeout=1):
    """Open a serial connection to the printer, applying the baud-rate-set-twice
    workaround needed for the 8U2 firmware/PySerial interaction on some boards."""
    ser = serial.Serial(port, baud, timeout=timeout)
    ser.baudrate = 9600
    ser.baudrate = baud
    return ser
