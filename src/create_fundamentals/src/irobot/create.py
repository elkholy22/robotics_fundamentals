"""Driver utilities for the iRobot Create Open Interface."""

from datetime import datetime
from math import pi
from struct import pack, unpack
from threading import Lock, Thread
from time import sleep

from serial import Serial


class Monitor(Thread):
    def __init__(self, watchdog, packet_ref, create, read, send_all, update):
        super().__init__(daemon=True)
        self.watchdog = watchdog
        self.packet_ref = packet_ref
        self.create = create
        self.read = read
        self.send_all = send_all
        self.update = update

        def mask(num):
            return (
                bool(num >> 7 & 1),
                bool(num >> 6 & 1),
                bool(num >> 5 & 1),
                bool(num >> 4 & 1),
                bool(num >> 3 & 1),
                bool(num >> 2 & 1),
                bool(num >> 1 & 1),
                bool(num & 1),
            )

        def unsigned(num):
            return (num,)

        def as_bool(num):
            return (bool(num),)

        def signed(num):
            if num >> 15:
                return (-1 * (2**16 - num),)
            return (num,)

        def signed8(num):
            if num >> 7:
                return (-1 * (2**8 - num),)
            return (num,)

        self.packets = (
            (
                7,
                1,
                mask,
                (
                    "na",
                    "na",
                    "na",
                    "wheeldrop_caster",
                    "wheeldrop_left",
                    "wheeldrop_right",
                    "bump_left",
                    "bump_right",
                ),
            ),
            (8, 1, as_bool, ("wall",)),
            (9, 1, as_bool, ("cliff_left",)),
            (10, 1, as_bool, ("cliff_front_left",)),
            (11, 1, as_bool, ("cliff_front_right",)),
            (12, 1, as_bool, ("cliff_right",)),
            (13, 1, as_bool, ("virtual_wall",)),
            (17, 1, unsigned, ("infrared_byte",)),
            (18, 1, mask, ("na", "na", "na", "na", "na", "advance", "na", "play")),
            (19, 2, signed, ("distance",)),
            (20, 2, signed, ("angle",)),
            (21, 1, unsigned, ("charging_state",)),
            (22, 2, unsigned, ("voltage",)),
            (23, 2, signed, ("current",)),
            (24, 1, signed8, ("battery_temperature",)),
            (25, 2, unsigned, ("battery_charge",)),
            (26, 2, unsigned, ("battery_capacity",)),
            (27, 2, unsigned, ("wall_signal",)),
            (28, 2, unsigned, ("cliff_left_signal",)),
            (29, 2, unsigned, ("cliff_front_left_signal",)),
            (30, 2, unsigned, ("cliff_front_right_signal",)),
            (31, 2, unsigned, ("cliff_right_signal",)),
            (34, 1, mask, ("na", "na", "na", "na", "na", "na", "home_base", "internal_charger")),
            (36, 1, unsigned, ("song_number",)),
            (37, 1, unsigned, ("song_playing",)),
        )

    def run(self):
        expected = sum([p[1] for p in self.packets])
        while len(self.watchdog) == 0:
            then = datetime.now()
            self.send_all()

            # Stream packet request
            self.create.send(149, len(self.packets), *[p[0] for p in self.packets])
            self.send_all()
            data_bytes = self.read(expected)
            if len(data_bytes) != expected:
                continue

            try:
                values = unpack("B" * expected, data_bytes)
                data = {}
                offset = 0
                d = None
                theta = None
                for packet in self.packets:
                    packet_size = packet[1]
                    if packet_size == 1:
                        results = packet[2](values[offset])
                    else:
                        results = packet[2]((values[offset] << 8) | values[offset + 1])

                    for idx, result in enumerate(results):
                        key = packet[3][idx]
                        data[key] = result
                        if key == "distance":
                            d = result
                        elif key == "angle":
                            theta = result
                    offset += packet_size

                if d is not None and theta is not None:
                    self.create.update_encoders(d, theta)
                data["encoder_left"], data["encoder_right"] = self.create.get_encoders()

                if len(self.packet_ref):
                    self.packet_ref[0] = data
                else:
                    self.packet_ref.append(data)
            except Exception:
                continue

            try:
                self.update()
            except Exception:
                # Node may already be shutting down; stop monitor loop quietly.
                break
            elapsed = datetime.now() - then
            elapsed_ms = elapsed.seconds * 1000.0 + elapsed.microseconds / 1000.0
            period_ms = self.create.period * 1000.0
            if elapsed_ms < period_ms:
                sleep((period_ms - elapsed_ms) / 1000.0)


class Create:
    def __init__(self, tty="/dev/ttyUSB0", baud=57600):
        self.timeout = 5
        self.period = 0.07
        self.run_ref = []
        self.packet_ref = []
        self.queue_lock = Lock()
        self.queue = []
        self.encoder_lock = Lock()
        self._encoder_left = 0.0
        self._encoder_right = 0.0
        self.port = Serial(tty, baud, timeout=self.timeout)
        self.port_lock = Lock()
        self.update = lambda: None
        self.reset()

    def start(self):
        # Start + full mode + store startup tone + play tone
        self._send_now(128, 128, 132, 140, 1, 5, 64, 16, 69, 16, 74, 16, 72, 40, 69, 60, 141, 1)
        sleep(1.0)
        self.send(139, 10, 0, 255)
        monitor = Monitor(self.run_ref, self.packet_ref, self, self._read, self._send_all, self.update)
        monitor.start()
        sleep(1.5)

    def stop(self):
        self.run_ref.append("quit")
        sleep(0.2)
        rh, rl = self._convert(0)
        lh, ll = self._convert(0)
        self._send_now(145, rh, rl, lh, ll)
        self._send_now(139, 0, 0, 255)

    def reset(self):
        self.run_ref = []
        sleep(0.1)
        self.port.flushOutput()
        self.port.flushInput()
        self._send_now(128, 7)
        # Opcode 7 (Reset) triggers a full hardware reset that takes several
        # seconds. The robot streams ~128 bytes back during init. We must wait
        # for this to finish before sending any further commands, otherwise
        # Start/Full-mode/song/drive opcodes are silently lost.
        sleep(4)
        self._read(self.port.inWaiting())
        self.port.flushInput()

    def _convert(self, num):
        return self._high_low(self._twos(num))

    @staticmethod
    def _high_low(num):
        return num >> 8, num & 0xFF

    @staticmethod
    def _twos(num, bits=16):
        if num >= 0:
            return num
        return 2**bits + num

    def send(self, *opcodes):
        with self.queue_lock:
            self.queue.append(lambda: self._send_now(*opcodes))

    def _send_now(self, *opcodes):
        with self.port_lock:
            data = pack("B" * len(opcodes), *opcodes)
            self.port.write(data)

    def _send_all(self):
        with self.queue_lock:
            self._read(self.port.inWaiting())
            for fn in self.queue:
                fn()
            self.queue = []

    def _read(self, num, block=False):
        with self.port_lock:
            if block:
                self.port.timeout = None
            data = self.port.read(num)
            if block:
                self.port.timeout = self.timeout
        return data

    def __getattr__(self, name):
        if len(self.packet_ref) and name in self.packet_ref[0]:
            return self.packet_ref[0][name]
        raise AttributeError(f"no {name} attribute")

    def update_encoders(self, d, theta):
        theta = theta / 180.0 * pi
        b = 258.0
        with self.encoder_lock:
            if theta == 0:
                self._encoder_left += d / 32.0
                self._encoder_right += d / 32.0
            else:
                r = d / theta - b / 2
                self._encoder_left += (r * theta) / 32.0
                self._encoder_right += ((r + b) * theta) / 32.0

    def get_encoders(self):
        with self.encoder_lock:
            return self._encoder_left, self._encoder_right

    def clear(self):
        with self.encoder_lock:
            self._encoder_left = 0.0
            self._encoder_right = 0.0

    def __del__(self):
        try:
            self.port.close()
        except Exception:
            pass

    def leds(self, play, advance, color, intensity):
        play = 1 if play else 0
        advance = 1 if advance else 0
        bits = play << 1 | advance << 3
        self._send_now(139, bits, color, intensity)

    def storeSong(self, num, *song):
        # OI song payload is max 16 note-duration pairs (32 bytes).
        # Keep an even number of bytes so the command stream stays aligned.
        song = list(song[:32])
        if len(song) % 2 != 0:
            song = song[:-1]
        self._send_now(140, num, int(len(song) / 2), *song)

    def playSong(self, num):
        self._send_now(141, num)

    def diffDrive(self, left, right):
        left *= 32.0
        right *= 32.0
        left = max(min(left, 500), -500)
        right = max(min(right, 500), -500)
        lh, ll = self._convert(int(left))
        rh, rl = self._convert(int(right))
        self._send_now(145, rh, rl, lh, ll)
