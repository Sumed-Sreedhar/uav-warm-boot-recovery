#!/usr/bin/env python3
"""Build the standalone C parser and test it with Python-generated HIL frames."""

import ctypes as c
from pathlib import Path
import struct
import subprocess
import tempfile
import unittest
import zlib


CORE = Path(__file__).resolve().parents[1] / "flight-app" / "Core"
VECTORS = (0.0, 0.0, 0.0, 0.0, 0.0, -9.80665, 22.0, 0.0, 44.0)
FRAME_SIZE = 58
RING_SIZE = 512


def encode_cobs(data):
    encoded = bytearray([0])
    code_index = 0
    for byte in data:
        if byte:
            encoded.append(byte)
        else:
            encoded[code_index] = len(encoded) - code_index
            code_index = len(encoded)
            encoded.append(0)
    encoded[code_index] = len(encoded) - code_index
    return bytes(encoded) + b"\0"


def sensor_frame(seq=42, time_us=210000, *, version=1, msg_type=1, length=36):
    payload = struct.pack("<BBHIQ9f", version, msg_type, length, seq, time_us, *VECTORS)
    return encode_cobs(payload + struct.pack("<I", zlib.crc32(payload)))


class Parser(c.Structure):
    _fields_ = [
        ("encoded", c.c_uint8 * (FRAME_SIZE - 1)),
        ("length", c.c_size_t),
        ("discard_until_delimiter", c.c_bool),
    ]


class Sample(c.Structure):
    _fields_ = [
        ("time_us", c.c_uint64),
        ("seq", c.c_uint32),
        ("gyro", c.c_float * 3),
        ("accel", c.c_float * 3),
        ("mag", c.c_float * 3),
    ]


class SensorParserTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        tmp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(tmp.cleanup)
        library = Path(tmp.name) / "hil_sensor.so"
        subprocess.run(
            [
                "cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic",
                "-O2", "-shared", "-fPIC", f"-I{CORE / 'Inc'}",
                CORE / "Src" / "hil_sensor.c", "-o", library,
            ],
            check=True,
        )
        cls.push = c.CDLL(str(library)).HilSensor_PushByte
        cls.push.argtypes = [c.POINTER(Parser), c.c_uint8, c.POINTER(Sample)]
        cls.push.restype = c.c_bool

    def setUp(self):
        self.parser = Parser()
        self.sample = Sample()

    def feed(self, data):
        sequences = []
        for byte in data:
            if self.push(c.byref(self.parser), byte, c.byref(self.sample)):
                sequences.append(self.sample.seq)
        return sequences

    def test_known_good(self):
        frame = sensor_frame()
        self.assertEqual(len(frame), FRAME_SIZE)
        self.assertEqual(self.feed(frame), [42])
        self.assertEqual(self.sample.time_us, 210000)
        want = struct.unpack("<9f", struct.pack("<9f", *VECTORS))
        got = (*self.sample.gyro, *self.sample.accel, *self.sample.mag)
        self.assertEqual(got, want)

    def test_every_split(self):
        seq, time_us = 0xFEDCBA98, 0x0102030405060708
        frame = sensor_frame(seq, time_us)
        for split in range(1, len(frame)):
            with self.subTest(split=split):
                self.assertEqual(self.feed(frame[:split]), [])
                self.assertEqual(self.feed(frame[split:]), [seq])
                self.assertEqual(self.sample.time_us, time_us)

    def test_frames_across_simulated_ring_wraps(self):
        ring = bytearray(RING_SIZE)
        cursor = RING_SIZE - 12
        stream = b"".join(sensor_frame(seq, seq * 5000) for seq in range(64))
        sequences = []
        for offset in range(0, len(stream), 113):
            chunk = stream[offset:offset + 113]
            first = min(len(chunk), RING_SIZE - cursor)
            ring[cursor:cursor + first] = chunk[:first]
            ring[:len(chunk) - first] = chunk[first:]
            sequences += self.feed(ring[cursor:cursor + first])
            sequences += self.feed(ring[:len(chunk) - first])
            cursor = (cursor + len(chunk)) % RING_SIZE
        self.assertEqual(sequences, list(range(64)))

    def test_invalid_frames_and_resynchronization(self):
        frame = sensor_frame()
        damaged = bytearray(frame)
        damaged[4] ^= 0x80  # Changes data without changing COBS structure.
        bad_frames = {
            "crc": damaged,
            "version": sensor_frame(version=2),
            "type": sensor_frame(msg_type=2),
            "length": sensor_frame(length=35),
            "invalid COBS": b"\xff" + frame[1:],
            "oversize": b"\x01" + frame,
            "missing delimiter": frame[:-1] + frame,
            "reset mid-frame": frame[12:],
            "empty": b"\0\0",
        }
        for reason, bad in bad_frames.items():
            with self.subTest(reason=reason):
                self.sample.seq = 123
                self.assertEqual(self.feed(bad), [])
                self.assertEqual(self.sample.seq, 123)
                self.assertEqual(self.feed(frame), [42])


if __name__ == "__main__":
    unittest.main()
