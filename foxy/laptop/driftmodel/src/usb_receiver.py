"""Serial link to the STM32 board.

Receiving: the board sends text lines  "<steer>\\t<mode>\\t<throttle>\\t<punishment>\\n".
Sending  : the laptop sends two lines "<steer>\\n<throttle>\\n" (voltages in V).
"""

import threading
import time

import serial   # pyserial

from utils import clamp, STEER_NEUTRAL, THROTTLE_NEUTRAL

MAX_PENDING_BYTES = 4096


def parse_line(line):
    """Parse one received line. Returns (steer, mode_flag, throttle, punishment_flag) or None if it is not valid."""
    parts = line.strip().split()
    if len(parts) != 4:
        return None
    try:
        steer = float(parts[0])
        mode_flag = int(parts[1]) != 0
        throttle = float(parts[2])
        punishment_flag = int(parts[3]) != 0
    except ValueError:
        return None
    return steer, mode_flag, throttle, punishment_flag


class LineBuffer:
    """Collects the bytes read from the port and returns the newest complete valid line.

    Reading everything that is waiting and using only the first line would return old data or a
    partial line when bytes pile up, so the newest complete line is used instead.
    """

    def __init__(self):
        self._pending = b''

    def feed(self, data):
        self._pending += data
        chunks = self._pending.split(b'\n')
        self._pending = chunks[-1]                  # incomplete tail, kept for the next call
        if len(self._pending) > MAX_PENDING_BYTES:  # no newline for a long time: discard the garbage
            self._pending = b''
        for raw in reversed(chunks[:-1]):           # newest line first
            parsed = parse_line(raw.decode('ascii', errors='ignore'))
            if parsed is not None:
                return parsed
        return None


class USBReceiver:
    def __init__(self, port='/dev/ACMSTM32', baudrate=115200, timeout=0.05):
        self.steer = STEER_NEUTRAL
        self.throttle = THROTTLE_NEUTRAL
        self.mode_flag = False
        self.punishment_flag = False
        self.has_data = False        # True after the first valid line
        self.last_rx_time = None     # time.monotonic() of the last valid line

        self._buffer = LineBuffer()
        self._write_lock = threading.Lock()
        self._serial = serial.Serial(port=port, baudrate=baudrate, timeout=timeout, write_timeout=1.0)

    def receive_data(self):
        """Read what is available. Returns True when a new valid line was received."""
        waiting = self._serial.in_waiting
        if waiting <= 0:
            return False
        parsed = self._buffer.feed(self._serial.read(waiting))
        if parsed is None:
            return False
        self.steer, self.mode_flag, self.throttle, self.punishment_flag = parsed
        self.has_data = True
        self.last_rx_time = time.monotonic()
        return True

    def transmit_data(self, steer, throttle):
        """Send a command. Every value is limited to 0 .. 3.3 V."""
        steer = clamp(steer)
        throttle = clamp(throttle)
        payload = '{:.6f}\n{:.6f}\n'.format(steer, throttle).encode('ascii')
        with self._write_lock:
            self._serial.write(payload)

    def send_neutral(self):
        self.transmit_data(STEER_NEUTRAL, THROTTLE_NEUTRAL)

    def close(self):
        try:
            self._serial.close()
        except Exception:
            pass
