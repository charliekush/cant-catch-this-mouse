"""ESP32-WROVER camera capture on the MPU.

The kit's stock camera is a separate ESP32 microcontroller, not a USB device:
it hosts its own WiFi AP (default 192.168.4.1) and serves an MJPEG multipart
stream over HTTP from the stock Espressif CameraWebServer firmware. Network
prerequisite: the MPU must already be a WiFi client of that AP (or of
whatever network the ESP32 is reflashed to join in STA mode) before this
class can reach it -- that join happens outside this code and is not
something to work around here.

Uses only the standard library for HTTP (urllib) rather than requests, since
requests isn't installed in .venv and pulling it in would be a new
dependency for a fairly small amount of parsing. Frames are found by
scanning the byte stream for JPEG SOI/EOI markers rather than parsing the
multipart boundary and Content-Length headers: a literal 0xFFD8/0xFFD9 can't
appear spuriously inside a valid JPEG's entropy-coded scan data (any real
0xFF byte there is followed by a stuffed 0x00), so this is robust without
needing to track the ESP32's exact boundary string. Decoding uses
cv2.imdecode on the raw JPEG bytes, which only needs OpenCV's built-in JPEG
codec -- not an FFMPEG-enabled build (cv2.VideoCapture on an http:// URL
would need one, and that may not be present in the apt python3-opencv build
used on the board).
"""

import urllib.request
from urllib.error import URLError

import cv2
import numpy as np

from .. import config

_JPEG_SOI = b"\xff\xd8"
_JPEG_EOI = b"\xff\xd9"
_READ_CHUNK = 4096


class Camera:
    """Pulls frames from the ESP32-WROVER's MJPEG stream over HTTP."""

    def __init__(self, stream_url=config.CAMERA_STREAM_URL,
                 timeout=config.CAMERA_TIMEOUT_S):
        self.stream_url = stream_url
        self.timeout = timeout
        self._response = None
        self._buffer = bytearray()
        self._open()

    def _open(self):
        try:
            self._response = urllib.request.urlopen(self.stream_url, timeout=self.timeout)
        except (URLError, OSError) as exc:
            raise RuntimeError(
                f"Could not open camera stream at {self.stream_url}: {exc}"
            ) from exc

    def read(self):
        """Return the latest frame (BGR ndarray), or None on failure."""
        jpeg = self._next_jpeg()
        if jpeg is None:
            return None
        return cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)

    def _next_jpeg(self):
        """Pull bytes off the stream until one full JPEG frame is buffered."""
        if self._response is None:
            return None
        while True:
            start = self._buffer.find(_JPEG_SOI)
            if start != -1:
                end = self._buffer.find(_JPEG_EOI, start + 2)
                if end != -1:
                    end += 2
                    jpeg = bytes(self._buffer[start:end])
                    del self._buffer[:end]
                    return jpeg
                if start > 0:
                    del self._buffer[:start]  # drop multipart boundary/header noise
            try:
                chunk = self._response.read(_READ_CHUNK)
            except (URLError, OSError):
                chunk = b""
            if not chunk:
                self._reconnect()
                return None
            self._buffer.extend(chunk)

    def _reconnect(self):
        """Drop and retry the HTTP connection once, e.g. after a WiFi hiccup."""
        self.release()
        try:
            self._open()
        except RuntimeError:
            pass

    def release(self):
        if self._response is not None:
            self._response.close()
            self._response = None
        self._buffer.clear()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.release()


class StubCamera:
    """Hardware-free stand-in: yields a blank frame so the MPU loop can run off-board."""

    def __init__(self, width=config.FRAME_WIDTH, height=config.FRAME_HEIGHT):
        self._frame = np.zeros((height, width, 3), dtype=np.uint8)

    def read(self):
        return self._frame

    def release(self):
        pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.release()
