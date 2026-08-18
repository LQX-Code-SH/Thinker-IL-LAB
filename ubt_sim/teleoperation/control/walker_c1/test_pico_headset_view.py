"""Offline protocol and H.264 checks for PICO Remote Vision."""
from __future__ import annotations

import shutil
import struct
import subprocess
import types
import unittest

import numpy as np

from pico_headset_view import (
    AnnexBAccessUnitParser,
    CameraRequest,
    PicoHeadsetView,
    _ros_image_to_rgb,
    parse_camera_request,
    parse_control_body,
)


def _compact(value: str) -> bytes:
    encoded = value.encode("utf-8")
    return bytes([len(encoded)]) + encoded


class PicoHeadsetViewProtocolTest(unittest.TestCase):
    def test_open_camera_protocol_matches_unity_layout(self):
        request_data = (
            b"\xca\xfe\x01"
            + struct.pack("<7i", 1280, 720, 60, 4_000_000, 0, 1, 12345)
            + _compact("ZED")
            + _compact("192.168.1.100")
        )
        command = b"OPEN_CAMERA"
        body = (
            struct.pack("<i", len(command))
            + command
            + struct.pack("<i", len(request_data))
            + request_data
        )

        parsed_command, parsed_data = parse_control_body(body)
        parsed_request = parse_camera_request(parsed_data)

        self.assertEqual(parsed_command, "OPEN_CAMERA")
        self.assertEqual(
            parsed_request,
            CameraRequest(
                1280, 720, 60, 4_000_000, 0, 1, 12345,
                "ZED", "192.168.1.100",
            ),
        )

    def test_annex_b_parser_preserves_access_units_across_chunks(self):
        aud_one = b"\x00\x00\x00\x01\x09\xf0"
        sps = b"\x00\x00\x00\x01\x67\x42\x00\x1f"
        idr = b"\x00\x00\x01\x65\x88\x84"
        aud_two = b"\x00\x00\x00\x01\x09\xf0"
        predicted = b"\x00\x00\x01\x41\x9a\x20"
        stream = aud_one + sps + idr + aud_two + predicted
        parser = AnnexBAccessUnitParser()
        output = []
        # Feed deliberately awkward slices, including split start codes.
        chunks = [stream[:2], stream[2:9], stream[9:17], stream[17:29], stream[29:]]
        for chunk in chunks:
            output.extend(parser.feed(chunk))
        output.extend(parser.flush())

        self.assertEqual(output, [aud_one + sps + idr, aud_two + predicted])

    def test_ros_rgb_conversion_honors_row_padding_and_bgr(self):
        # Two BGR pixels plus two padding bytes.
        msg = types.SimpleNamespace(
            height=1,
            width=2,
            step=8,
            encoding="bgr8",
            data=bytes([30, 20, 10, 60, 50, 40, 0, 0]),
        )
        rgb = _ros_image_to_rgb(msg)
        np.testing.assert_array_equal(
            rgb, np.asarray([[[10, 20, 30], [40, 50, 60]]], dtype=np.uint8)
        )

    def test_ffmpeg_command_emits_aud_delimited_h264(self):
        ffmpeg = shutil.which("ffmpeg")
        if not ffmpeg:
            self.skipTest("ffmpeg is not installed")
        encoder_list = subprocess.run(
            [ffmpeg, "-hide_banner", "-encoders"],
            check=False,
            text=True,
            capture_output=True,
        )
        if "libx264" not in encoder_list.stdout:
            self.skipTest("ffmpeg has no libx264 encoder")
        view = object.__new__(PicoHeadsetView)
        view.ffmpeg_path = ffmpeg
        request = CameraRequest(64, 48, 30, 500_000, 0, 0, 12345, "ZED", "127.0.0.1")
        process = subprocess.run(
            view._ffmpeg_command(request),
            input=bytes(64 * 48 * 3 * 3),
            check=False,
            capture_output=True,
            timeout=10.0,
        )
        self.assertEqual(process.returncode, 0, process.stderr.decode(errors="replace"))
        parser = AnnexBAccessUnitParser()
        access_units = parser.feed(process.stdout) + parser.flush()
        self.assertGreaterEqual(len(access_units), 3)
        self.assertTrue(all(unit.startswith((b"\x00\x00\x01", b"\x00\x00\x00\x01")) for unit in access_units))


if __name__ == "__main__":
    unittest.main()
