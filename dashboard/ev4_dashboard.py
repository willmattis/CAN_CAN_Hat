#!/usr/bin/env python3
"""
BER (Bearcats Electric Racing) - EV4 CAN Dashboard (Raspberry Pi native)
========================================================================

Runs directly on a Raspberry Pi with two MCP2515 CAN HATs. It reads both
CAN buses natively over SocketCAN (python-can), decodes every frame against
the EV4 / inverter DBCs, and shows a live Tkinter dashboard. It can also
write to the Cascadia inverters (command + parameter frames) for motor
calibration, and repeats the inverter command "heartbeat" itself (the job
the ESP32 firmware used to do) with a deadman safety.

The DBC is the single source of truth: the signal panels are
built dynamically from it, so editing the .dbc and restarting
the app is all that's needed when the bus layout changes.

Two CAN HATs come up as the SocketCAN interfaces can0 and can1 (bring them
up with setup_can.sh). Internally the dashboard uses bus index 0 and 1,
mapped to the two interfaces in order:
    bus 0 -> first channel  (Vehicle bus + Inverter 2 + IMD)
    bus 1 -> second channel (Inverter 1, on its own bus)

Usage:
    pip install -r requirements.txt
    sudo ./setup_can.sh                 # bring up can0/can1 @ 500 kbps
    python3 ev4_dashboard.py            # connects to can0,can1 by default
    # optional: python3 ev4_dashboard.py --channels can0 can1 --bitrate 500000
    # no hardware: python3 ev4_dashboard.py --demo
"""

import argparse
import collections
import csv
import datetime
import json
import os
import struct
import sys
import threading
import time
import queue
import tkinter as tk
from tkinter import ttk, simpledialog

try:
    import cantools
except ImportError:
    sys.exit("cantools is required:  pip install -r requirements.txt")

# python-can is only needed to talk to real hardware (SocketCAN on the Pi);
# import it lazily inside CanReader so --demo still runs on a dev laptop.

import cascadia_params as cparams


HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CHANNELS = ["can0", "can1"]   # SocketCAN interfaces for the two HATs
DEFAULT_BITRATE = 500000              # vehicle/inverter bus bit rate
STALE_AFTER = 1.0          # seconds without a frame -> message marked stale
PANEL_MIN_W = 320          # min px per message panel; tab columns = width // this
                           # (gives 3 cols at the 1280 windowed default, more wide)

# DBC sources, decoded together.  Each (filename, prefix); the prefix keeps
# signals/messages unique when two buses reuse the same names (the two
# inverters share every message + signal name, only their CAN IDs differ).
# Frame IDs do NOT collide across files (vehicle 0x02-0x07 + IMD 0x18FF01F4,
# INV1 0xA0-0xC1, INV2 0xD0-0xF1), so a known id is decoded on whichever bus it
# arrives -- the ECU echoes vehicle/IMD frames onto BOTH CAN buses.
# (filename, prefix, bus). The bus here is each source's PRIMARY/transmit bus:
#   bus 0 = can0: Vehicle/ECU + Inverter 1 (0xA0/0xC0)
#   bus 1 = can1: Inverter 2 (0xD0/0xF0), isolated (ECU also echoes vehicle here)
DBC_SOURCES = [
    ("EV4_Vehicle_Bus.dbc", "",     0),   # vehicle/ECU bus -> no prefix
    ("Inverter_1.dbc",      "INV1", 0),   # Inverter 1 shares the vehicle bus (can0)
    ("Inverter_2.dbc",      "INV2", 1),   # Inverter 2 on the isolated bus (can1)
]

# Friendly bus names by prefix (used for tab labels and the lookup table).
TAB_TITLES = {"": "Vehicle", "INV1": "Inverter 1", "INV2": "Inverter 2"}

LOOKUP_MAX = 150          # cap rows rendered per search
WATCH_LAST = "__last__"   # auto-restored group name


def _watch_store_path():
    """Saved watch groups live in a per-user config dir (outside the repo) so
    updating/re-flashing the project never resets them. Migrates the old
    in-repo file once if present."""
    base = os.environ.get("APPDATA") or os.path.join(os.path.expanduser("~"), ".config")
    cfg = os.path.join(base, "EV4_CAN_Viewer")
    try:
        os.makedirs(cfg, exist_ok=True)
        path = os.path.join(cfg, "watch_groups.json")
    except OSError:
        return os.path.join(HERE, "watch_groups.json")   # fallback
    old = os.path.join(HERE, "watch_groups.json")
    if os.path.exists(old) and not os.path.exists(path):
        try:
            import shutil
            shutil.copyfile(old, path)
        except OSError:
            pass
    return path


WATCH_FILE = _watch_store_path()

# Signals promoted to the big "driver" strip at the top.  Each entry is
# (label, qualified_signal_name, unit, format).  A signal is "qualified" with
# its source prefix, e.g. INV1_INV_Motor_Speed.  Missing signals are skipped.
KEY_SIGNALS = [
    ("SOC",     "BMS_SOC",              "%",   "{:.0f}"),
    ("POWER",   "PowerDraw_kW",         "kW",  "{:.0f}"),
    ("PACK",    "BMS_Pack_Voltage",     "V",   "{:.0f}"),
    ("MAX T",   "BMS_Max_Cell_Temp",    "C",   "{:.0f}"),
    ("APPS",    "APPS_Pct",             "%",   "{:.0f}"),
    ("TORQUE",  "Torque_Cmd",           "Nm",  "{:.0f}"),
    ("M1 RPM",  "INV1_INV_Motor_Speed", "rpm", "{:.0f}"),
    ("M2 RPM",  "INV2_INV_Motor_Speed", "rpm", "{:.0f}"),
]


def clean_unit(unit):
    """Cascadia DBC units look like 'temperature:C' / 'angular_speed:rpm'.
    Keep only the part after the colon for display."""
    if unit and ":" in unit:
        return unit.split(":", 1)[1]
    return unit or ""


def qualify(prefix, name):
    """Globally-unique signal key: 'INV1_INV_Motor_Speed', or just the raw
    name for the unprefixed vehicle bus."""
    return f"{prefix}_{name}" if prefix else name


def panel_key(prefix, msg):
    return f"{prefix} {msg.name}" if prefix else msg.name


class MsgInfo:
    """Everything needed to decode a frame and route it into the UI."""
    __slots__ = ("message", "prefix", "panel_key", "bus")

    def __init__(self, message, prefix, bus=0):
        self.message = message
        self.prefix = prefix
        self.bus = bus
        self.panel_key = panel_key(prefix, message)

# Colors
BG       = "#0d0d10"
PANEL    = "#16161c"
FG       = "#e6e6e6"
GREY     = "#8a8a92"
ACCENT   = "#ff2d2d"
GREEN    = "#27d17c"
YELLOW   = "#ffd23f"
CYAN     = "#36c5d6"


def is_fault_signal(name: str) -> bool:
    n = name.lower()
    return ("fault" in n or n.endswith("_fault") or "critical" in n
            or "alarm" in n)   # IMD iso/unbalance/undervoltage alarms


# ── Cascadia Motion (PM100) inverter write helpers ───────────────────
# Calibration / command parameter addresses (CAN Protocol v5.9 §2.3.3-2.3.4).
PARAM_RESOLVER_DELAY_CMD   = 11    # live Resolver PWM Delay (PM Gen3 only)
PARAM_GAMMA_ADJUST_CMD     = 12    # live Gamma Adjust, degrees x10
PARAM_FAULT_CLEAR          = 20    # write 0 to clear faults
PARAM_RESOLVER_DELAY_EEP   = 151   # save Resolver PWM Delay to EEPROM
PARAM_GAMMA_ADJUST_EEP     = 152   # save Gamma Adjust to EEPROM, degrees x10
PARAM_INVERTER_RUN_MODE    = 142   # EEPROM: 0=Torque, 1=Speed (immediate)
PARAM_INVERTER_CMD_MODE    = 143   # EEPROM: 0=CAN, 1=VSM (needs power-cycle)

# Fault bit -> name tables for the 0x0AB Fault Codes message (CAN Protocol v5.9
# §2.1, pp. 27-28). Bit index is within the 32-bit word formed as (Hi<<16)|Lo,
# i.e. Lo = bits 0-15, Hi = bits 16-31. Reserved bits are omitted.
POST_FAULTS = {
    0: "HW Gate/Desaturation", 1: "HW Over-current", 2: "Accelerator Shorted",
    3: "Accelerator Open", 4: "Current Sensor Low", 5: "Current Sensor High",
    6: "Module Temp Low", 7: "Module Temp High", 8: "Control PCB Temp Low",
    9: "Control PCB Temp High", 10: "Gate Drive PCB Temp Low",
    11: "Gate Drive PCB Temp High", 12: "5V Sense Low", 13: "5V Sense High",
    14: "12V Sense Low", 15: "12V Sense High", 16: "2.5V Sense Low",
    17: "2.5V Sense High", 18: "1.5V Sense Low", 19: "1.5V Sense High",
    20: "DC Bus Voltage High", 21: "DC Bus Voltage Low", 22: "Pre-charge Timeout",
    23: "Pre-charge Voltage Fail", 24: "EEPROM Checksum Invalid",
    25: "EEPROM Data Out of Range", 26: "EEPROM Update Required",
    27: "HW DC Bus Over-Voltage (init)", 28: "Gate Driver Init (Gen5)",
    30: "Brake Shorted", 31: "Brake Open",
}
RUN_FAULTS = {
    0: "Motor Over-speed", 1: "Over-current", 2: "Over-voltage",
    3: "Inverter Over-temp", 4: "Accelerator Input Shorted",
    5: "Accelerator Input Open", 6: "Direction Command",
    7: "Inverter Response Timeout", 8: "HW Gate/Desaturation",
    9: "HW Over-current", 10: "Under-voltage", 11: "CAN Command Message Lost",
    12: "Motor Over-temp", 16: "Brake Input Shorted", 17: "Brake Input Open",
    18: "Module A Over-temp", 19: "Module B Over-temp", 20: "Module C Over-temp",
    21: "PCB Over-temp", 22: "Gate Drive Board 1 Over-temp",
    23: "Gate Drive Board 2 Over-temp", 24: "Gate Drive Board 3 Over-temp",
    25: "Current Sensor", 26: "Gate Driver Over-Voltage (Gen5)",
    27: "HW DC Bus Over-Voltage (Gen3)", 28: "HW DC Bus Over-Voltage (Gen5)",
    30: "Resolver Not Connected",
}


def decode_faults(table, lo, hi):
    """Return (bits, [active fault names]) for a Lo/Hi fault word pair."""
    bits = (int(hi or 0) << 16) | int(lo or 0)
    names = [name for bit, name in table.items() if bits & (1 << bit)]
    return bits, names


def _clamp_s16(v):
    return max(-32768, min(32767, int(v)))


def build_command_frame(torque_nm=0.0, speed_rpm=0, forward=True, enable=False,
                        discharge=False, speed_mode=False, torque_limit_nm=0.0):
    """0x_C0 Command Message. torque/limit are N·m (x10 on the wire),
    speed is RPM. Byte 5 packs enable/discharge/speed-mode bits."""
    b = bytearray(8)
    struct.pack_into("<h", b, 0, _clamp_s16(round(torque_nm * 10)))
    struct.pack_into("<h", b, 2, _clamp_s16(round(speed_rpm)))
    b[4] = 1 if forward else 0
    b[5] = ((1 if enable else 0)
            | ((1 if discharge else 0) << 1)
            | ((1 if speed_mode else 0) << 2))
    struct.pack_into("<h", b, 6, _clamp_s16(round(torque_limit_nm * 10)))
    return bytes(b)


def build_param_write(address, value, signed=True):
    """0x_C1 Read/Write Parameter Command, write. Data goes in bytes 4-5."""
    b = bytearray(8)
    struct.pack_into("<H", b, 0, address & 0xFFFF)
    b[2] = 1                                   # 1 = write
    fmt = "<h" if signed else "<H"
    struct.pack_into(fmt, b, 4, _clamp_s16(value) if signed else (int(value) & 0xFFFF))
    return bytes(b)


def build_param_read(address):
    """0x_C1 Read/Write Parameter Command, read."""
    b = bytearray(8)
    struct.pack_into("<H", b, 0, address & 0xFFFF)
    b[2] = 0                                   # 0 = read
    return bytes(b)


def can_frame_bits(nbytes, extended):
    """Approx. on-wire bits for one CAN frame (overhead + data + IFS), used to
    estimate bus load. Standard ~47 + 8·DLC, extended ~67 + 8·DLC; bit stuffing
    is not modelled, so this slightly under-estimates."""
    return (67 if extended else 47) + 8 * nbytes


class CanReader(threading.Thread):
    """Native SocketCAN transport (replaces the old ESP32 serial bridge).

    Opens one python-can Bus per CAN HAT, runs a receive loop per bus that
    decodes frames and pushes them to the out_queue exactly like the old
    SerialReader did, and owns the inverter command *heartbeat*: it re-sends
    the command frame every HEARTBEAT_MS and drops it if the GUI stops
    refreshing it within DEADMAN_S — reproducing the ESP32 firmware's
    20 ms repeat + 500 ms deadman so the inverter's own CAN timeout disables
    the motor if the app hangs or is closed.

    channels is an ordered list of (bus_index, ifname); bus_index 0 maps to
    the first interface, 1 to the second, matching DBC_SOURCES.
    """

    HEARTBEAT_MS = 20      # inverter command re-send period (matches firmware)
    DEADMAN_S = 0.5        # drop heartbeat if the GUI hasn't refreshed it

    def __init__(self, channels, bitrate, frame_map, out_queue, status_queue):
        super().__init__(daemon=True)
        self.channels = channels          # [(bus_index, ifname), ...]
        self.bitrate = bitrate
        self.frame_map = frame_map
        # Bus-agnostic index: every frame id is unique across the DBCs, and the
        # ECU echoes vehicle/IMD frames onto both buses, so if (bus, id) misses
        # we still decode a known id seen on the "wrong" bus.
        self.frame_by_id = {}
        for (_b, fid), info in frame_map.items():
            self.frame_by_id.setdefault(fid, info)
        self.out_queue = out_queue
        self.status_queue = status_queue
        self._stop = threading.Event()
        self.buses = {}                   # bus_index -> can.Bus
        self._can = None                  # the imported python-can module
        self._hb_lock = threading.Lock()
        self._hb_frame = None             # (bus, can_id, data, extended) or None
        self._hb_last_update = 0.0

    def stop(self):
        self._stop.set()

    # ---- transmit API (used by the Control tab) -------------------------
    def send_frame(self, bus, can_id, data, extended=None):
        """Send one CAN frame now (param read/write, fault clear, disable)."""
        if extended is None:
            extended = can_id > 0x7FF
        return self._tx(bus, can_id, bytes(data), extended)

    def set_heartbeat(self, bus, can_id, data, extended=None):
        """Set/refresh the repeating command frame and pet the deadman."""
        if extended is None:
            extended = can_id > 0x7FF
        with self._hb_lock:
            self._hb_frame = (bus, can_id, bytes(data), extended)
            self._hb_last_update = time.time()

    def stop_heartbeat(self):
        with self._hb_lock:
            self._hb_frame = None

    def _tx(self, bus, can_id, data, extended):
        b = self.buses.get(bus)
        if b is None or self._can is None:
            return False
        try:
            b.send(self._can.Message(arbitration_id=can_id, data=data,
                                     is_extended_id=extended))
            return True
        except Exception:
            return False

    def _tick_heartbeat(self):
        with self._hb_lock:
            frame, last = self._hb_frame, self._hb_last_update
        if frame is None:
            return
        if time.time() - last > self.DEADMAN_S:
            # GUI went silent -> drop the heartbeat; the inverter's own CAN
            # timeout (>1 s) then disables the motor.
            with self._hb_lock:
                self._hb_frame = None
            return
        bus, can_id, data, ext = frame
        self._tx(bus, can_id, data, ext)

    # ---- threads --------------------------------------------------------
    def run(self):
        try:
            import can
        except ImportError:
            self.status_queue.put(
                ("error", "python-can not installed:  pip install -r requirements.txt"))
            return
        self._can = can

        opened, failed = [], []
        for bus_index, ifname in self.channels:
            try:
                bus = can.Bus(channel=ifname, interface="socketcan")
            except Exception as e:
                # One bad/missing interface must NOT take down the others;
                # warn and keep going with whatever opens (graceful degrade).
                self.status_queue.put(("error", f"Could not open {ifname}: {e}"))
                failed.append(ifname)
                continue
            self.buses[bus_index] = bus
            opened.append(ifname)
            threading.Thread(target=self._rx_loop, args=(bus_index, bus),
                             daemon=True).start()

        if not self.buses:
            self.status_queue.put(("error", "No CAN interfaces could be opened"))
            return

        label = " + ".join(opened) + (f"  (no {', '.join(failed)})" if failed else "")
        self.status_queue.put(("connected", label))
        try:
            while not self._stop.is_set():
                self._tick_heartbeat()
                time.sleep(self.HEARTBEAT_MS / 1000.0)
        finally:
            self._close_all()
            self.status_queue.put(("disconnected", " + ".join(opened)))

    def _close_all(self):
        for b in self.buses.values():
            try:
                b.shutdown()
            except Exception:
                pass
        self.buses = {}

    def _rx_loop(self, bus, can_bus):
        """Blocking receive loop for one CAN interface."""
        while not self._stop.is_set():
            try:
                msg = can_bus.recv(timeout=0.2)
            except Exception as e:
                if not self._stop.is_set():
                    self.status_queue.put(("error", f"recv error on bus {bus}: {e}"))
                return
            if msg is not None:
                self._handle_frame(bus, msg)

    def _handle_frame(self, bus, msg):
        frame_id = msg.arbitration_id
        data = bytes(msg.data)
        bits = can_frame_bits(len(data), msg.is_extended_id)   # bus-load estimate

        info = self.frame_map.get((bus, frame_id))
        if info is None:
            info = self.frame_by_id.get(frame_id)   # known id on the other bus
        if info is None:
            self.out_queue.put(("unknown", frame_id, None, bits, bus))
            return
        m = info.message

        # Pad/truncate to the expected length so decode never crashes on a
        # short frame.
        if len(data) < m.length:
            data = data + b"\x00" * (m.length - len(data))
        elif len(data) > m.length:
            data = data[: m.length]

        try:
            decoded = m.decode(data, decode_choices=False, scaling=True)
        except Exception:
            return
        # Qualify signal names with the source prefix so the two inverters
        # (identical signal names) never overwrite each other.
        qual = {qualify(info.prefix, k): v for k, v in decoded.items()}
        self.out_queue.put(("frame", info.panel_key, qual, bits, bus))


class DemoReader(threading.Thread):
    """Generates plausible values for EVERY message/signal on every bus so the
    whole UI (all tabs) can be exercised with no hardware.  Activated with
    --demo."""

    def __init__(self, ordered_msgs, prefix_bus, out_queue, status_queue):
        super().__init__(daemon=True)
        self.ordered_msgs = ordered_msgs      # list of (prefix, message)
        self.prefix_bus = prefix_bus
        self.out_queue = out_queue
        self.status_queue = status_queue
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    # Demo mode never transmits; these keep the Control-tab calls harmless.
    def send_frame(self, *a, **k):
        return False

    def set_heartbeat(self, *a, **k):
        return False

    def stop_heartbeat(self):
        return False

    @staticmethod
    def _sig_value(sig, t):
        import math
        import random
        name = sig.name
        if is_fault_signal(name):
            return 1 if random.random() < 0.01 else 0     # occasional red blip
        if sig.length == 1:                               # boolean-ish
            return 1 if (int(t) + (hash(name) % 7)) % 7 == 0 else 0
        lo = sig.minimum if sig.minimum is not None else 0.0
        hi = sig.maximum if sig.maximum is not None else 0.0
        if hi <= lo:                                       # no usable range
            lo, hi = 0.0, 100.0
        phase = (hash(name) % 100) / 100.0 * 2 * math.pi   # desync the waves
        wave = (math.sin(t + phase) + 1) / 2               # 0..1
        val = lo + (hi - lo) * wave
        return round(val, 3)

    def run(self):
        self.status_queue.put(("connected", "DEMO"))
        t = 0.0
        while not self._stop.is_set():
            t += 0.1
            for prefix, msg in self.ordered_msgs:
                decoded = {qualify(prefix, s.name): self._sig_value(s, t)
                           for s in msg.signals}
                bits = can_frame_bits(msg.length, msg.is_extended_frame)
                bus = self.prefix_bus.get(prefix, 0)
                self.out_queue.put(("frame", panel_key(prefix, msg), decoded, bits, bus))
            time.sleep(0.1)
        self.status_queue.put(("disconnected", "DEMO"))


class Dashboard(tk.Tk):
    def __init__(self, sources, channels, bitrate, demo=False):
        super().__init__()
        # sources: list of (prefix, cantools.Database, bus)
        self.sources = sources
        self.vehicle_db = sources[0][1]
        self.channels = channels        # ordered list of SocketCAN ifnames
        self.bitrate = bitrate
        self.demo = demo
        self.reader = None

        # Which CAN bus each source (prefix) lives on.
        self.prefix_bus = {prefix: bus for prefix, _db, bus in sources}
        self.buses = sorted(set(self.prefix_bus.values()))

        # frame routing table keyed by (bus, frame_id) so the same id can mean
        # different things on different buses; plus ordered (prefix, message).
        self.frame_map = {}
        self.ordered_msgs = []
        for prefix, db, bus in sources:
            # Keep real frames (incl. 29-bit extended, max 0x1FFFFFFF); the
            # VECTOR__INDEPENDENT_SIG_MSG placeholder (0xC0000000) is excluded.
            msgs = sorted((m for m in db.messages if m.frame_id <= 0x1FFFFFFF),
                          key=lambda m: m.frame_id)
            for m in msgs:
                if (bus, m.frame_id) not in self.frame_map:   # first source wins on clash
                    self.frame_map[(bus, m.frame_id)] = MsgInfo(m, prefix, bus)
                self.ordered_msgs.append((prefix, m))

        # Inverter write targets: command + parameter message IDs per inverter,
        # looked up from the DBC by message name (so they track the DBC).
        self.inverters = {}
        for prefix, db, bus in sources:
            if not prefix:
                continue
            try:
                cmd = db.get_message_by_name("M192_Command_Message")
                par = db.get_message_by_name("M193_Read_Write_Param_Command")
            except KeyError:
                continue
            self.inverters[prefix] = {"command_id": cmd.frame_id,
                                      "param_id": par.frame_id, "bus": bus}

        # Transmit / heartbeat state (set by the Control tab).
        self.tx_active = False          # heartbeat running?
        self.tx_command = None          # 8-byte command frame being repeated
        self.tx_command_id = None       # CAN id for the heartbeat
        self.tx_bus = 0                 # which CAN bus the heartbeat goes on

        # Searchable catalog of every signal (for the Lookup tab).
        self.catalog = []
        for prefix, m in self.ordered_msgs:
            bus = TAB_TITLES.get(prefix, prefix or "Vehicle")
            for s in m.signals:
                comment = (s.comment or "").replace("\n", " ").strip()
                if s.choices:
                    enum = ", ".join(f"{k}={v}" for k, v in list(s.choices.items())[:10])
                    comment = (comment + "  |  " if comment else "") + "states: " + enum
                key = qualify(prefix, s.name)
                # blob includes the CAN id (hex + decimal) so you can search by address
                self.catalog.append({
                    "key": key, "name": s.name, "bus": bus, "prefix": prefix,
                    "msg": m.name, "frame_id": m.frame_id,
                    "unit": clean_unit(s.unit),
                    "scale": s.scale, "offset": s.offset,
                    "min": s.minimum, "max": s.maximum,
                    "comment": comment,
                    "blob": (f"{key} {s.name} {bus} {m.name} {comment} "
                             f"0x{m.frame_id:x} {m.frame_id}").lower(),
                })
        self.lookup_value_labels = {}
        self.catalog_by_key = {c["key"]: c for c in self.catalog}

        # Watch panel state (pinned signals + saved groups).
        # A watch key is either a qualified signal name (e.g. "INV1_INV_Motor_Speed")
        # or a Cascadia parameter "@param:<INVx>:<addr>" added from the Param Manager.
        self.watch_signals = []          # ordered list of watch keys
        self.watch_value_labels = {}     # key -> (value label, unit)
        self.watch_popout = None         # Toplevel when popped out, else None
        self.charts = []                 # open chart windows (opt-in)
        self._watch_param_cache = {}     # param key -> last decoded value
        self._watch_rr = -1              # round-robin index for auto param reads
        self._watch_rr_last = 0.0

        self.out_queue = queue.Queue()
        self.status_queue = queue.Queue()

        self.values = {}          # signal_name -> latest value
        self.msg_last_rx = {}     # message_name -> timestamp
        self.msg_period = {}      # message_name -> averaged inter-frame period (s)
        self.frame_count = 0
        self.unknown_count = 0
        self._fps_window = []     # timestamps for frames/sec
        self._load_window = []    # (timestamp, frame_bits) for bus-load estimate

        self.value_labels = {}    # signal_name -> tk Label
        self.key_labels = {}      # signal_name -> tk Label
        self.panel_titles = {}    # message_name -> LabelFrame
        self._panel_base_text = {}  # message_name -> title text without the rate

        # CSV logging.  Column order = qualified signals grouped by message.
        self.log_signals = [qualify(prefix, s.name)
                            for prefix, m in self.ordered_msgs for s in m.signals]
        self.csv_file = None
        self.csv_writer = None
        self.log_rows = 0
        self.log_start = 0.0

        self.title("BER EV4 - Vehicle Bus Dashboard")
        self.configure(bg=BG)
        self.geometry("1280x820")
        self._build_ui()
        self._maximize()
        self.after(250, self._reflow_all)   # re-flow panels to the maximized width

        self.after(100, self._poll)
        self.after(1000, self._update_rates)

    def _maximize(self):
        """Open at full-screen size. Resizing/maximizing *after* the UI is
        painted can leave static widgets (the Watch panel, the top-strip
        labels/units) un-repainted over VNC/Wayland — only the live value
        labels keep refreshing. Painting at full size from the start avoids
        that broken resize-repaint. Falls back gracefully across WMs."""
        for attempt in (lambda: self.state("zoomed"),
                        lambda: self.attributes("-zoomed", True)):
            try:
                attempt()
                return
            except tk.TclError:
                continue
        self.update_idletasks()
        self.geometry(f"{self.winfo_screenwidth()}x{self.winfo_screenheight()}+0+0")

    # ---- UI construction -------------------------------------------------
    def _build_ui(self):
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        # --- connection bar ---
        bar = tk.Frame(self, bg=PANEL)
        bar.pack(side=tk.TOP, fill=tk.X)

        # The two CAN HATs come up as SocketCAN interfaces. Channel order maps
        # to bus index: channel 0 -> bus 0 (Vehicle/INV2), channel 1 -> bus 1 (INV1).
        ch0 = self.channels[0] if len(self.channels) > 0 else "can0"
        ch1 = self.channels[1] if len(self.channels) > 1 else "can1"
        tk.Label(bar, text="bus0", bg=PANEL, fg=GREY).pack(side=tk.LEFT, padx=(10, 2), pady=8)
        self.ch0_var = tk.StringVar(value=ch0)
        tk.Entry(bar, textvariable=self.ch0_var, width=7, bg=BG, fg=FG,
                 insertbackground=FG, relief=tk.FLAT).pack(side=tk.LEFT, padx=2)
        tk.Label(bar, text="bus1", bg=PANEL, fg=GREY).pack(side=tk.LEFT, padx=(8, 2))
        self.ch1_var = tk.StringVar(value=ch1)
        tk.Entry(bar, textvariable=self.ch1_var, width=7, bg=BG, fg=FG,
                 insertbackground=FG, relief=tk.FLAT).pack(side=tk.LEFT, padx=2)

        tk.Label(bar, text="bitrate", bg=PANEL, fg=GREY).pack(side=tk.LEFT, padx=(12, 4))
        self.bitrate_var = tk.StringVar(value=str(self.bitrate))
        tk.Entry(bar, textvariable=self.bitrate_var, width=8, bg=BG, fg=FG,
                 insertbackground=FG, relief=tk.FLAT).pack(side=tk.LEFT, padx=4)

        self.connect_btn = tk.Button(bar, text="Connect", command=self._toggle_connect,
                                     bg=GREEN, fg="#06210f", relief=tk.FLAT, width=10)
        self.connect_btn.pack(side=tk.LEFT, padx=10)

        self.status_lbl = tk.Label(bar, text="disconnected", bg=PANEL, fg=GREY)
        self.status_lbl.pack(side=tk.LEFT, padx=10)

        self.fps_lbl = tk.Label(bar, text="0 fps", bg=PANEL, fg=GREY)
        self.fps_lbl.pack(side=tk.RIGHT, padx=14)

        self.log_lbl = tk.Label(bar, text="", bg=PANEL, fg=GREY)
        self.log_lbl.pack(side=tk.RIGHT, padx=4)
        self.log_btn = tk.Button(bar, text="Log CSV", command=self._toggle_log,
                                 bg=PANEL, fg=FG, relief=tk.FLAT, width=10)
        self.log_btn.pack(side=tk.RIGHT, padx=6)

        # --- key metric strip ---
        strip = tk.Frame(self, bg=BG)
        strip.pack(side=tk.TOP, fill=tk.X, pady=(8, 4))
        all_sig_names = set(self.log_signals)
        col = 0
        for label, sig, unit, fmt in KEY_SIGNALS:
            if sig not in all_sig_names:
                continue
            cell = tk.Frame(strip, bg=PANEL, bd=0)
            cell.grid(row=0, column=col, padx=6, pady=2, sticky="nsew")
            strip.grid_columnconfigure(col, weight=1)
            tk.Label(cell, text=label, bg=PANEL, fg=GREY,
                     font=("Segoe UI", 11)).pack(anchor="w", padx=12, pady=(8, 0))
            val = tk.Label(cell, text="--", bg=PANEL, fg=FG,
                           font=("Consolas", 24, "bold"))
            val.pack(anchor="w", padx=12)
            tk.Label(cell, text=unit, bg=PANEL, fg=GREY,
                     font=("Segoe UI", 9)).pack(anchor="w", padx=12, pady=(0, 8))
            self.key_labels[sig] = (val, fmt)
            col += 1

        # --- tabbed pages: one per DBC source (Vehicle / Inverter 1 / 2) ---
        style.configure("TNotebook", background=BG, borderwidth=0)
        style.configure("TNotebook.Tab", background=PANEL, foreground=GREY,
                        padding=(16, 6), font=("Segoe UI", 10, "bold"))
        style.map("TNotebook.Tab",
                  background=[("selected", BG)], foreground=[("selected", FG)])

        # body: tabs on the left, persistent Watch panel docked on the right,
        # with a draggable sash between them (widen the Watch panel for big values).
        body = ttk.PanedWindow(self, orient=tk.HORIZONTAL)
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        left = tk.Frame(body, bg=BG)
        body.add(left, weight=1)
        nb = ttk.Notebook(left)
        nb.pack(fill=tk.BOTH, expand=True, padx=8, pady=(4, 8))
        self.nb = nb

        self._panel_pages = []   # message-panel pages that re-flow on resize
        for prefix, _db, _bus in self.sources:
            msgs = [(p, m) for (p, m) in self.ordered_msgs if p == prefix]
            page = tk.Frame(nb, bg=BG)
            nb.add(page, text=TAB_TITLES.get(prefix, prefix or "Vehicle"))
            tk.Label(page, text="Right-click any signal to add it to the Watch panel "
                     "or open a chart.", bg=BG, fg=GREY,
                     font=("Segoe UI", 8)).pack(side=tk.TOP, anchor="w", padx=12, pady=(4, 0))
            inner, canvas = self._make_scrollable(page, fill_width=True)
            self._build_message_panels(inner, msgs, canvas)

        self._build_lookup_tab(nb)
        if self.inverters:
            self._build_control_tab(nb)

        self._build_watch_panel(body)

    def _make_scrollable(self, parent, fill_width=False):
        """Create a vertically-scrollable frame inside parent; return
        (inner_frame, canvas).  With fill_width the inner frame tracks the
        canvas width so its columns spread to fill (no empty right margin).
        Mouse wheel works while the pointer is over it."""
        canvas = tk.Canvas(parent, bg=BG, highlightthickness=0)
        vsb = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        inner = tk.Frame(canvas, bg=BG)
        inner.bind("<Configure>",
                   lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        win_id = canvas.create_window((0, 0), window=inner, anchor="nw")
        if fill_width:
            canvas.bind("<Configure>",
                        lambda e: canvas.itemconfig(win_id, width=e.width))
        canvas.configure(yscrollcommand=vsb.set)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        vsb.pack(side=tk.RIGHT, fill=tk.Y)
        # Route the wheel to whichever page the pointer is currently over.
        canvas.bind("<Enter>", lambda e: canvas.bind_all(
            "<MouseWheel>", lambda ev: canvas.yview_scroll(int(-ev.delta / 120), "units")))
        canvas.bind("<Leave>", lambda e: canvas.unbind_all("<MouseWheel>"))
        return inner, canvas

    # ---- Watch panel (pinned signals, docked right) ---------------------
    def _build_watch_panel(self, parent):
        # parent is the horizontal PanedWindow; the Watch panel is a resizable pane.
        wp = tk.Frame(parent, bg=PANEL, width=270)
        self.watch_paned = parent
        parent.add(wp, weight=0)
        self.watch_dock = wp

        hdr = tk.Frame(wp, bg=PANEL)
        hdr.pack(fill=tk.X, padx=8, pady=(8, 2))
        tk.Label(hdr, text="\U0001F4CC Watch", bg=PANEL, fg=CYAN,
                 font=("Segoe UI", 12, "bold")).pack(side=tk.LEFT)
        self.watch_count = tk.Label(hdr, text="", bg=PANEL, fg=GREY, font=("Segoe UI", 9))
        self.watch_count.pack(side=tk.RIGHT)

        btns = tk.Frame(wp, bg=PANEL)
        btns.pack(fill=tk.X, padx=6, pady=2)
        for txt, cmd in [("Save", self._watch_save_group), ("Load", self._watch_load_group),
                         ("Clear", self._watch_clear), ("Pop ↗", self._watch_toggle_popout)]:
            tk.Button(btns, text=txt, command=cmd, bg=BG, fg=FG, relief=tk.FLAT,
                      width=5).pack(side=tk.LEFT, padx=1)
        self.watch_group_lbl = tk.Label(wp, text="right-click a row → chart",
                                        bg=PANEL, fg=GREY, font=("Segoe UI", 8), anchor="w")
        self.watch_group_lbl.pack(fill=tk.X, padx=8)

        self.watch_dock_inner = self._watch_make_list(wp)
        self.watch_inner = self.watch_dock_inner

        self._watch_rebuild()
        self._watch_restore_last()      # reload last session's pins

    def _watch_make_list(self, parent):
        """Build a scrollable region for pinned rows; return its inner frame.
        The inner frame tracks the canvas width so rows fill the (resizable)
        panel and the name↔value spacing grows when you widen it."""
        listwrap = tk.Frame(parent, bg=PANEL)
        listwrap.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)
        canvas = tk.Canvas(listwrap, bg=PANEL, highlightthickness=0)
        vsb = ttk.Scrollbar(listwrap, orient="vertical", command=canvas.yview)
        inner = tk.Frame(canvas, bg=PANEL)
        inner.grid_columnconfigure(0, weight=1)
        inner.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        win_id = canvas.create_window((0, 0), window=inner, anchor="nw")
        canvas.bind("<Configure>", lambda e: canvas.itemconfig(win_id, width=e.width))
        canvas.configure(yscrollcommand=vsb.set)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        vsb.pack(side=tk.RIGHT, fill=tk.Y)
        return inner

    def _watch_toggle_popout(self):
        if self.watch_popout is not None and self.watch_popout.winfo_exists():
            self._watch_dock_back()
            return
        # detach into an always-on-top floating window
        win = tk.Toplevel(self)
        win.title("📌 Watch")
        win.configure(bg=PANEL)
        win.geometry("280x420")
        win.attributes("-topmost", True)
        win.protocol("WM_DELETE_WINDOW", self._watch_dock_back)
        self.watch_popout = win
        self.watch_paned.forget(self.watch_dock)   # remove the docked pane
        self.watch_inner = self._watch_make_list(win)
        self._watch_rebuild()

    def _watch_dock_back(self):
        if self.watch_popout is not None:
            try:
                self.watch_popout.destroy()
            except Exception:
                pass
            self.watch_popout = None
        self.watch_paned.add(self.watch_dock, weight=0)   # re-attach the pane
        self.watch_inner = self.watch_dock_inner
        self._watch_rebuild()

    def _watch_entry_info(self, key):
        """Return (prefix, display_name, unit) for any watch key (signal or param)."""
        if key.startswith("@param:"):
            _, pfx, addr = key.split(":")
            p = cparams.BY_ADDR.get(int(addr))
            name = (p["name"] if p else f"param {addr}") + f"  (p{addr})"
            return pfx, name, (p["unit"] if p else "")
        c = self.catalog_by_key.get(key)
        if c:
            return c["prefix"], c["name"], c["unit"]
        return "", key, ""

    def _watch_key_valid(self, key):
        if key.startswith("@param:"):
            try:
                _, pfx, addr = key.split(":")
                return pfx in self.inverters and int(addr) in cparams.BY_ADDR
            except (ValueError, AttributeError):
                return False
        return key in self.catalog_by_key

    def _watch_rebuild(self):
        for w in self.watch_inner.winfo_children():
            w.destroy()
        self.watch_value_labels = {}
        if not self.watch_signals:
            tk.Label(self.watch_inner, text="Pin signals from the\n\U0001F50D Lookup tab "
                     "(\U0001F4CC), or add params\nfrom the Parameter Manager",
                     bg=PANEL, fg=GREY, justify="left",
                     font=("Segoe UI", 9)).grid(row=0, column=0, sticky="w", padx=8, pady=12)
            self.watch_count.config(text="")
            return

        # group keys by bus, in source order (Vehicle, INV1, INV2, …)
        order = [pfx for pfx, _db, _bus in self.sources]
        groups = {pfx: [] for pfx in order}
        for key in self.watch_signals:
            pfx = self._watch_entry_info(key)[0]
            groups.setdefault(pfx, []).append(key)

        r = 0
        for pfx in order:
            keys = groups.get(pfx, [])
            if not keys:
                continue
            color = self.SRC_COLOR.get(pfx, FG)
            tk.Label(self.watch_inner, text=TAB_TITLES.get(pfx, pfx or "Vehicle").upper(),
                     bg=PANEL, fg=color, anchor="w",
                     font=("Segoe UI", 8, "bold")).grid(row=r, column=0, sticky="ew",
                                                        padx=6, pady=(6, 1))
            r += 1
            for key in keys:
                _, name, unit = self._watch_entry_info(key)
                row = tk.Frame(self.watch_inner, bg=PANEL)
                row.grid(row=r, column=0, sticky="ew", padx=2, pady=1)
                row.grid_columnconfigure(1, weight=1)   # name column absorbs slack
                r += 1
                tk.Button(row, text="×", command=lambda k=key: self._watch_unpin(k),
                          bg=PANEL, fg=ACCENT, relief=tk.FLAT, bd=0, padx=2,
                          font=("Segoe UI", 9, "bold")).grid(row=0, column=0, sticky="w")
                namelbl = tk.Label(row, text=name, bg=PANEL, fg=color, anchor="w",
                                   font=("Consolas", 8))
                namelbl.grid(row=0, column=1, sticky="ew", padx=(2, 8))
                # no fixed width: the value gets the slack as the panel widens
                val = tk.Label(row, text="--", bg=PANEL, fg=FG, anchor="e",
                               font=("Consolas", 11, "bold"))
                val.grid(row=0, column=2, sticky="e")
                self.watch_value_labels[key] = (val, unit)
                for w in (row, namelbl, val):
                    w.bind("<Button-3>", lambda e, k=key: self._watch_context(e, k))
        self.watch_count.config(text=f"{len(self.watch_signals)}")
        self._refresh_values()

    def _watch_pin(self, key):
        if key not in self.watch_signals:
            self.watch_signals.append(key)
            self._watch_rebuild()
            self._watch_save_to(WATCH_LAST)

    def _watch_unpin(self, key):
        if key in self.watch_signals:
            self.watch_signals.remove(key)
            self._watch_rebuild()
            self._watch_save_to(WATCH_LAST)

    def _watch_clear(self):
        self.watch_signals = []
        self.watch_group_lbl.config(text="")
        self._watch_rebuild()
        self._watch_save_to(WATCH_LAST)

    def _watch_context(self, event, key):
        menu = tk.Menu(self, tearoff=0, bg=PANEL, fg=FG)
        if self.inverters:
            menu.add_command(label="⚙ Send to Control",
                             command=lambda: self._watch_to_control(key))
        menu.add_command(label="📈 Open chart", command=lambda: self._open_chart(key))
        menu.add_command(label="× Remove", command=lambda: self._watch_unpin(key))
        try:
            menu.tk_popup(event.x_root, event.y_root)
        finally:
            menu.grab_release()

    def _watch_to_control(self, key):
        """Load a watched variable into the Control tab's parameter editor."""
        if not self.inverters:
            return
        self.nb.select(self.control_page)
        if key.startswith("@param:"):
            _, pfx, addr = key.split(":")
            addr = int(addr)
            if pfx not in self.inverters:
                return
            if pfx != self.ctl_inv.get() and self.tx_active:
                self._ctl_estop()           # switching inverter context -> safe stop
            self.ctl_inv.set(pfx)
            self.par_addr.set(str(addr))
            if key in self._watch_param_cache:
                self.par_val.set(str(self._watch_param_cache[key]))
            p = cparams.BY_ADDR.get(addr)
            self.par_status.config(
                text=f"loaded {p['name'] if p else 'param'} (addr {addr}, {pfx}) "
                     "from Watch — Read / Write below", fg=CYAN)
        else:
            c = self.catalog_by_key.get(key)
            name = c["name"] if c else key
            self.par_status.config(
                text=f"{name} is a broadcast signal (read-only) — not a writable "
                     "parameter", fg=YELLOW)

    def _signal_context(self, event, key):
        """Right-click menu for a signal in the bus message panels."""
        menu = tk.Menu(self, tearoff=0, bg=PANEL, fg=FG)
        if key in self.watch_signals:
            menu.add_command(label="× Remove from Watch",
                             command=lambda: self._watch_unpin(key))
        else:
            menu.add_command(label="📌 Add to Watch",
                             command=lambda: self._watch_pin(key))
        menu.add_command(label="📈 Open chart", command=lambda: self._open_chart(key))
        try:
            menu.tk_popup(event.x_root, event.y_root)
        finally:
            menu.grab_release()

    # ---- live charts (opt-in, via right-click) --------------------------
    def _open_chart(self, key):
        c = self.catalog_by_key.get(key)
        name = c["name"] if c else key
        unit = c["unit"] if c else ""
        win = tk.Toplevel(self)
        win.title(f"📈 {name}")
        win.configure(bg=BG)
        win.geometry("520x300")

        stats = tk.Label(win, bg=BG, fg=GREY, anchor="w", font=("Consolas", 9))
        stats.pack(side=tk.TOP, fill=tk.X, padx=8, pady=(6, 0))
        canvas = tk.Canvas(win, bg="#0a0a0d", highlightthickness=0)
        canvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=8, pady=8)

        chart = {"key": key, "name": name, "unit": unit, "win": win,
                 "canvas": canvas, "stats": stats,
                 "data": collections.deque(maxlen=600)}
        self.charts.append(chart)
        win.protocol("WM_DELETE_WINDOW", lambda: self._close_chart(chart))

    def _close_chart(self, chart):
        if chart in self.charts:
            self.charts.remove(chart)
        try:
            chart["win"].destroy()
        except Exception:
            pass

    def _update_charts(self):
        for chart in list(self.charts):
            if not chart["win"].winfo_exists():
                self.charts.remove(chart)
                continue
            key = chart["key"]
            if key in self.values:
                try:
                    chart["data"].append(float(self.values[key]))
                except (TypeError, ValueError):
                    pass
            self._draw_chart(chart)

    def _draw_chart(self, chart):
        cv = chart["canvas"]
        cv.delete("all")
        data = chart["data"]
        w = cv.winfo_width() or 500
        h = cv.winfo_height() or 220
        if len(data) < 2:
            return
        lo, hi = min(data), max(data)
        rng = (hi - lo) or 1.0
        pad = 8
        n = len(data)
        sx = (w - 2 * pad) / (n - 1)

        def y(v):
            return h - pad - (v - lo) / rng * (h - 2 * pad)

        # zero line if range spans it
        if lo < 0 < hi:
            zy = y(0)
            cv.create_line(pad, zy, w - pad, zy, fill="#333344")
        pts = []
        for i, v in enumerate(data):
            pts += [pad + i * sx, y(v)]
        cv.create_line(*pts, fill=CYAN, width=1)
        cur = data[-1]
        u = f" {chart['unit']}" if chart["unit"] else ""
        chart["stats"].config(
            text=f"now {cur:.3g}{u}    min {lo:.3g}    max {hi:.3g}    Δ {hi - lo:.3g}")

    # ---- watch persistence (named groups in watch_groups.json) ----
    def _watch_load_file(self):
        try:
            with open(WATCH_FILE, encoding="utf-8") as f:
                return json.load(f)
        except (OSError, ValueError):
            return {}

    def _watch_write_file(self, groups):
        try:
            with open(WATCH_FILE, "w", encoding="utf-8") as f:
                json.dump(groups, f, indent=2)
        except OSError:
            pass

    def _watch_save_to(self, name):
        groups = self._watch_load_file()
        groups[name] = list(self.watch_signals)
        self._watch_write_file(groups)

    def _watch_restore_last(self):
        groups = self._watch_load_file()
        keys = groups.get(WATCH_LAST, [])
        # keep only keys that are still valid (signal in DBC, or known param)
        self.watch_signals = [k for k in keys if self._watch_key_valid(k)]
        self._watch_rebuild()

    def _watch_save_group(self):
        if not self.watch_signals:
            return
        name = simpledialog.askstring("Save watch group",
                                      "Name for this group:", parent=self)
        if not name:
            return
        self._watch_save_to(name)
        self.watch_group_lbl.config(text=f"saved as “{name}”")

    def _watch_load_group(self):
        groups = {k: v for k, v in self._watch_load_file().items() if k != WATCH_LAST}
        if not groups:
            self.watch_group_lbl.config(text="no saved groups yet")
            return
        self._watch_group_picker(groups)

    def _watch_group_picker(self, groups):
        win = tk.Toplevel(self)
        win.title("Load watch group")
        win.configure(bg=BG)
        win.geometry("300x320")
        tk.Label(win, text="Saved groups:", bg=BG, fg=GREY).pack(anchor="w", padx=10, pady=6)
        lb = tk.Listbox(win, bg=PANEL, fg=FG, selectbackground=CYAN, relief=tk.FLAT,
                        highlightthickness=0)
        for name in sorted(groups):
            lb.insert(tk.END, f"{name}  ({len(groups[name])})")
        lb.pack(fill=tk.BOTH, expand=True, padx=10, pady=4)
        names = sorted(groups)

        def do_load():
            sel = lb.curselection()
            if not sel:
                return
            name = names[sel[0]]
            self.watch_signals = [k for k in groups[name] if self._watch_key_valid(k)]
            self._watch_rebuild()
            self._watch_save_to(WATCH_LAST)
            self.watch_group_lbl.config(text=f"loaded “{name}”")
            win.destroy()

        def do_delete():
            sel = lb.curselection()
            if not sel:
                return
            name = names[sel[0]]
            allg = self._watch_load_file()
            allg.pop(name, None)
            self._watch_write_file(allg)
            win.destroy()

        bf = tk.Frame(win, bg=BG)
        bf.pack(fill=tk.X, padx=10, pady=8)
        tk.Button(bf, text="Load", command=do_load, bg="#33415c", fg=FG,
                  relief=tk.FLAT, width=8).pack(side=tk.LEFT, padx=4)
        tk.Button(bf, text="Delete", command=do_delete, bg="#5c3333", fg=FG,
                  relief=tk.FLAT, width=8).pack(side=tk.LEFT, padx=4)

    # Color the panel border by source so the buses are easy to tell apart.
    SRC_COLOR = {"": CYAN, "INV1": "#7ee081", "INV2": "#e0c97e"}

    def _build_message_panels(self, parent, msgs, canvas):
        frames = []
        for prefix, msg in msgs:
            tag = f"{prefix}  " if prefix else ""
            color = self.SRC_COLOR.get(prefix, CYAN)
            base_text = f"  {tag}{msg.name}  (0x{msg.frame_id:X})  "
            frame = tk.LabelFrame(parent, text=base_text,
                                  bg=PANEL, fg=color, bd=1, relief=tk.SOLID,
                                  font=("Segoe UI", 10, "bold"), labelanchor="nw")
            key = panel_key(prefix, msg)
            self.panel_titles[key] = (frame, color)
            self._panel_base_text[key] = base_text
            frames.append(frame)

            for r, sig in enumerate(msg.signals):
                key = qualify(prefix, sig.name)
                namelbl = tk.Label(frame, text=sig.name, bg=PANEL, fg=GREY, anchor="w",
                                   font=("Consolas", 9), cursor="hand2")
                namelbl.grid(row=r, column=0, sticky="w", padx=(8, 6), pady=1)
                u = clean_unit(sig.unit)
                val = tk.Label(frame, text="--" + (f" {u}" if u else ""), bg=PANEL,
                               fg=FG, anchor="e", font=("Consolas", 9, "bold"),
                               width=12, cursor="hand2")
                val.grid(row=r, column=1, sticky="e", padx=(6, 8), pady=1)
                frame.grid_columnconfigure(0, weight=1)   # name col absorbs slack
                self.value_labels[key] = (val, u)
                # right-click a signal anywhere in the panels -> add to Watch / chart
                for w in (namelbl, val):
                    w.bind("<Button-3>", lambda e, k=key: self._signal_context(e, k))

        # Masonry layout: column count follows the window width (3 at the
        # windowed default, more in fullscreen) and each panel keeps its own
        # height, dropping into the shortest column — so a tall panel (e.g.
        # MC_FAULTS) doesn't stretch its neighbours. Re-flows on resize.
        record = {"canvas": canvas, "inner": parent, "frames": frames,
                  "ncols": 0, "w": 0}
        self._panel_pages.append(record)
        canvas.bind("<Configure>", lambda e, rec=record: self._reflow_page(rec), add="+")
        self._reflow_page(record)

    def _reflow_page(self, record):
        """Place a page's message panels in masonry columns (width //
        PANEL_MIN_W of them). No-op when neither width nor column count changed."""
        canvas = record["canvas"]
        w = canvas.winfo_width()
        if w <= 1:
            return                              # not realized yet
        ncols = max(1, min(8, w // PANEL_MIN_W))
        if w == record["w"] and ncols == record["ncols"]:
            return
        record["w"], record["ncols"] = w, ncols
        inner = record["inner"]
        inner.update_idletasks()                # so reqheight() is accurate
        pad, col_w = 6, w // ncols
        col_h = [pad] * ncols                   # running height of each column
        for frame in record["frames"]:
            c = min(range(ncols), key=lambda i: col_h[i])   # shortest column
            frame.place(x=c * col_w + pad, y=col_h[c], width=col_w - 2 * pad)
            col_h[c] += frame.winfo_reqheight() + pad
        inner.configure(height=max(col_h) + pad)   # so the canvas scrolls right

    def _reflow_all(self):
        self.update_idletasks()
        for rec in self._panel_pages:
            rec["w"] = 0              # force a re-layout at the realized width
            self._reflow_page(rec)

    # ---- lookup / search tab --------------------------------------------
    def _build_lookup_tab(self, nb):
        page = tk.Frame(nb, bg=BG)
        nb.add(page, text="\U0001F50D Lookup")

        top = tk.Frame(page, bg=BG)
        top.pack(side=tk.TOP, fill=tk.X, padx=10, pady=10)
        tk.Label(top, text="Search signal:", bg=BG, fg=GREY,
                 font=("Segoe UI", 11)).pack(side=tk.LEFT)
        self.search_var = tk.StringVar()
        entry = tk.Entry(top, textvariable=self.search_var, bg=PANEL, fg=FG,
                         insertbackground=FG, relief=tk.FLAT,
                         font=("Consolas", 13), width=42)
        entry.pack(side=tk.LEFT, padx=10, ipady=3)
        entry.bind("<KeyRelease>", lambda e: self._do_search())
        tk.Button(top, text="Clear", command=lambda: (self.search_var.set(""), self._do_search()),
                  bg=PANEL, fg=FG, relief=tk.FLAT).pack(side=tk.LEFT, padx=4)
        self.search_count = tk.Label(top, text="", bg=BG, fg=GREY, font=("Segoe UI", 10))
        self.search_count.pack(side=tk.LEFT, padx=14)

        inner, _ = self._make_scrollable(page)
        self.lookup_results = tk.Frame(inner, bg=BG)
        self.lookup_results.pack(fill=tk.BOTH, expand=True)
        self._do_search()

    def _do_search(self):
        for w in self.lookup_results.winfo_children():
            w.destroy()
        self.lookup_value_labels = {}

        q = self.search_var.get().strip().lower()
        if not q:
            self.search_count.config(text=f"{len(self.catalog)} signals — type to filter")
            tk.Label(self.lookup_results,
                     text="Start typing part of a signal name, message, bus, or description…",
                     bg=BG, fg=GREY, font=("Segoe UI", 11)).grid(row=0, column=0, padx=12, pady=20)
            return

        terms = q.split()
        matches = [c for c in self.catalog if all(t in c["blob"] for t in terms)]
        shown = matches[:LOOKUP_MAX]
        extra = f"  (showing first {LOOKUP_MAX})" if len(matches) > LOOKUP_MAX else ""
        self.search_count.config(text=f"{len(matches)} match"
                                 f"{'' if len(matches) == 1 else 'es'}{extra}")

        headers = ["", "Signal", "Bus", "Message", "Value", "Scale / Range", "Description"]
        widths = [3, 30, 11, 26, 14, 22, 56]
        for col, (h, w) in enumerate(zip(headers, widths)):
            tk.Label(self.lookup_results, text=h, bg=BG, fg=CYAN, anchor="w",
                     font=("Segoe UI", 9, "bold"), width=w).grid(
                         row=0, column=col, sticky="w", padx=6, pady=(0, 4))

        for r, c in enumerate(shown, start=1):
            color = self.SRC_COLOR.get(c["prefix"], CYAN)
            tk.Button(self.lookup_results, text="\U0001F4CC", command=lambda k=c["key"]: self._watch_pin(k),
                      bg=BG, fg=GREY, relief=tk.FLAT, bd=0, padx=0,
                      font=("Segoe UI", 9)).grid(row=r, column=0, padx=(6, 0))
            tk.Label(self.lookup_results, text=c["name"], bg=BG, fg=FG, anchor="w",
                     font=("Consolas", 9, "bold")).grid(row=r, column=1, sticky="w", padx=6)
            tk.Label(self.lookup_results, text=c["bus"], bg=BG, fg=color, anchor="w",
                     font=("Consolas", 9)).grid(row=r, column=2, sticky="w", padx=6)
            tk.Label(self.lookup_results, text=f"{c['msg']} (0x{c['frame_id']:X})",
                     bg=BG, fg=GREY, anchor="w",
                     font=("Consolas", 9)).grid(row=r, column=3, sticky="w", padx=6)
            val = tk.Label(self.lookup_results, text="--", bg=BG, fg=FG, anchor="w",
                           font=("Consolas", 9, "bold"))
            val.grid(row=r, column=4, sticky="w", padx=6)
            self.lookup_value_labels[c["key"]] = (val, c["unit"])

            sr = f"×{c['scale']}"
            if c["offset"]:
                sr += f" {'+' if c['offset'] > 0 else ''}{c['offset']}"
            if c["min"] is not None and c["max"] is not None and c["max"] > c["min"]:
                sr += f"  [{c['min']}..{c['max']}]"
            tk.Label(self.lookup_results, text=sr, bg=BG, fg=GREY, anchor="w",
                     font=("Consolas", 8)).grid(row=r, column=5, sticky="w", padx=6)
            tk.Label(self.lookup_results, text=c["comment"], bg=BG, fg=GREY, anchor="w",
                     font=("Segoe UI", 8), justify="left", wraplength=440).grid(
                         row=r, column=6, sticky="w", padx=6)

        self._refresh_values()   # fill in current values right away

    # ---- control / write tab --------------------------------------------
    def _build_control_tab(self, nb):
        page = tk.Frame(nb, bg=BG)
        nb.add(page, text="⚙ Control")
        self.control_page = page
        outer, _ = self._make_scrollable(page)

        # Safety banner
        warn = tk.Frame(outer, bg="#3a0d0d")
        warn.grid(row=0, column=0, columnspan=2, sticky="ew", padx=8, pady=(8, 6))
        tk.Label(warn, bg="#3a0d0d", fg="#ff8a8a", justify="left",
                 font=("Segoe UI", 9, "bold"),
                 text=("⚠  WRITES TO LIVE INVERTERS — SPINS THE MOTOR.  "
                       "Drive wheels OFF the ground, area clear, physical e-stop ready.\n"
                       "STOP button disables the inverter and halts the heartbeat. "
                       "Closing the app or unplugging USB also safely stops the motor."
                       )).pack(anchor="w", padx=10, pady=8)

        # Inverter selector + torque cap
        sel = tk.Frame(outer, bg=BG)
        sel.grid(row=1, column=0, columnspan=2, sticky="w", padx=8, pady=4)
        tk.Label(sel, text="Inverter:", bg=BG, fg=GREY,
                 font=("Segoe UI", 11)).pack(side=tk.LEFT)
        self.ctl_inv = tk.StringVar(value=sorted(self.inverters)[0])
        for pfx in sorted(self.inverters):
            tk.Radiobutton(sel, text=TAB_TITLES.get(pfx, pfx), value=pfx,
                           variable=self.ctl_inv, bg=BG, fg=FG, selectcolor=PANEL,
                           activebackground=BG, activeforeground=FG,
                           command=self._ctl_inv_changed).pack(side=tk.LEFT, padx=6)
        tk.Label(sel, text="     Torque cap (Nm):", bg=BG, fg=GREY).pack(side=tk.LEFT)
        self.ctl_cap = tk.StringVar(value="30")
        tk.Entry(sel, textvariable=self.ctl_cap, width=6, bg=PANEL, fg=FG,
                 insertbackground=FG, relief=tk.FLAT).pack(side=tk.LEFT, padx=4)

        # ---- Command panel ----
        cmd = tk.LabelFrame(outer, text="  Command (motor)  ", bg=PANEL, fg=CYAN,
                            font=("Segoe UI", 10, "bold"), bd=1, relief=tk.SOLID)
        cmd.grid(row=2, column=0, sticky="nsew", padx=8, pady=6)

        self.ctl_armed = False
        self.ctl_enabled = False
        self.ctl_forward = tk.BooleanVar(value=True)
        self.ctl_torque = tk.StringVar(value="0")

        r = 0
        self.arm_btn = tk.Button(cmd, text="ARM (start heartbeat, disabled)",
                                 command=self._ctl_arm, bg="#33415c", fg=FG, relief=tk.FLAT)
        self.arm_btn.grid(row=r, column=0, columnspan=2, sticky="ew", padx=8, pady=(8, 4))
        r += 1
        self.enable_btn = tk.Button(cmd, text="ENABLE", command=self._ctl_enable,
                                    bg=PANEL, fg=GREY, relief=tk.FLAT, state=tk.DISABLED)
        self.enable_btn.grid(row=r, column=0, sticky="ew", padx=8, pady=2)
        self.disable_btn = tk.Button(cmd, text="DISABLE", command=self._ctl_disable,
                                     bg=PANEL, fg=FG, relief=tk.FLAT, state=tk.DISABLED)
        self.disable_btn.grid(row=r, column=1, sticky="ew", padx=8, pady=2)
        r += 1
        tk.Label(cmd, text="Direction:", bg=PANEL, fg=GREY).grid(row=r, column=0, sticky="w", padx=8)
        dirf = tk.Frame(cmd, bg=PANEL); dirf.grid(row=r, column=1, sticky="w")
        tk.Radiobutton(dirf, text="Forward", value=True, variable=self.ctl_forward, bg=PANEL,
                       fg=FG, selectcolor=BG).pack(side=tk.LEFT)
        tk.Radiobutton(dirf, text="Reverse", value=False, variable=self.ctl_forward, bg=PANEL,
                       fg=FG, selectcolor=BG).pack(side=tk.LEFT)
        r += 1
        tk.Label(cmd, text="Torque cmd (Nm):", bg=PANEL, fg=GREY).grid(row=r, column=0, sticky="w", padx=8, pady=4)
        tk.Entry(cmd, textvariable=self.ctl_torque, width=8, bg=BG, fg=FG,
                 insertbackground=FG, relief=tk.FLAT).grid(row=r, column=1, sticky="w", pady=4)
        r += 1
        # slider for quick torque control (clamped by the torque cap when sent)
        tk.Scale(cmd, from_=-50, to=50, orient=tk.HORIZONTAL, resolution=1,
                 bg=PANEL, fg=FG, troughcolor=BG, highlightthickness=0, showvalue=True,
                 command=lambda v: self.ctl_torque.set(v)).grid(
                     row=r, column=0, columnspan=2, sticky="ew", padx=8)
        r += 1
        self.estop_btn = tk.Button(cmd, text="⏹  STOP (disable + halt heartbeat)",
                                   command=self._ctl_estop, bg=ACCENT, fg="white",
                                   font=("Segoe UI", 10, "bold"), relief=tk.FLAT)
        self.estop_btn.grid(row=r, column=0, columnspan=2, sticky="ew", padx=8, pady=(6, 8))
        r += 1
        self.ctl_status = tk.Label(cmd, text="idle", bg=PANEL, fg=GREY, anchor="w")
        self.ctl_status.grid(row=r, column=0, columnspan=2, sticky="ew", padx=8, pady=(0, 8))

        # ---- Parameter panel ----
        par = tk.LabelFrame(outer, text="  Parameter read / write  ", bg=PANEL, fg=CYAN,
                            font=("Segoe UI", 10, "bold"), bd=1, relief=tk.SOLID)
        par.grid(row=2, column=1, sticky="nsew", padx=8, pady=6)
        self.par_addr = tk.StringVar(value="12")
        self.par_val = tk.StringVar(value="0")
        tk.Label(par, text="Address:", bg=PANEL, fg=GREY).grid(row=0, column=0, sticky="w", padx=8, pady=4)
        tk.Entry(par, textvariable=self.par_addr, width=8, bg=BG, fg=FG,
                 insertbackground=FG, relief=tk.FLAT).grid(row=0, column=1, sticky="w")
        tk.Label(par, text="Value:", bg=PANEL, fg=GREY).grid(row=1, column=0, sticky="w", padx=8, pady=4)
        tk.Entry(par, textvariable=self.par_val, width=8, bg=BG, fg=FG,
                 insertbackground=FG, relief=tk.FLAT).grid(row=1, column=1, sticky="w")
        bf = tk.Frame(par, bg=PANEL); bf.grid(row=2, column=0, columnspan=2, sticky="ew", padx=8, pady=4)
        tk.Button(bf, text="Read", command=self._par_read, bg="#33415c", fg=FG,
                  relief=tk.FLAT, width=8).pack(side=tk.LEFT, padx=2)
        tk.Button(bf, text="Write", command=self._par_write, bg="#5c4633", fg=FG,
                  relief=tk.FLAT, width=8).pack(side=tk.LEFT, padx=2)
        tk.Button(bf, text="Parameter Manager…", command=self._open_param_window,
                  bg="#2c4a3a", fg=FG, relief=tk.FLAT).pack(side=tk.LEFT, padx=8)
        self.par_status = tk.Label(par, text="", bg=PANEL, fg=GREY, anchor="w", wraplength=300, justify="left")
        self.par_status.grid(row=3, column=0, columnspan=2, sticky="ew", padx=8, pady=4)

        # calibration quick-actions
        qa = tk.Frame(par, bg=PANEL); qa.grid(row=4, column=0, columnspan=2, sticky="w", padx=8, pady=(8, 8))
        tk.Label(qa, text="Quick:", bg=PANEL, fg=GREY).grid(row=0, column=0, sticky="w")
        tk.Button(qa, text="Fault Clear", command=self._par_fault_clear,
                  bg=PANEL, fg=FG, relief=tk.FLAT).grid(row=0, column=1, padx=2, pady=2)
        tk.Button(qa, text="Set Gamma (live)", command=lambda: self._fill_param(PARAM_GAMMA_ADJUST_CMD),
                  bg=PANEL, fg=FG, relief=tk.FLAT).grid(row=0, column=2, padx=2, pady=2)
        tk.Button(qa, text="Save Gamma→EEPROM", command=lambda: self._fill_param(PARAM_GAMMA_ADJUST_EEP),
                  bg=PANEL, fg=FG, relief=tk.FLAT).grid(row=1, column=1, padx=2, pady=2)
        tk.Button(qa, text="Set Resolver Delay (live)", command=lambda: self._fill_param(PARAM_RESOLVER_DELAY_CMD),
                  bg=PANEL, fg=FG, relief=tk.FLAT).grid(row=1, column=2, padx=2, pady=2)
        tk.Button(qa, text="Save Resolver Delay→EEPROM", command=lambda: self._fill_param(PARAM_RESOLVER_DELAY_EEP),
                  bg=PANEL, fg=FG, relief=tk.FLAT).grid(row=2, column=2, padx=2, pady=2)
        tk.Button(qa, text="Enter CAN + Torque mode", command=self._ctl_set_can_mode,
                  bg="#33415c", fg=FG, relief=tk.FLAT).grid(row=2, column=1, padx=2, pady=2)

        # ---- Live calibration readouts ----
        cal = tk.LabelFrame(outer, text="  Live calibration readouts (selected inverter)  ",
                            bg=PANEL, fg=CYAN, font=("Segoe UI", 10, "bold"), bd=1, relief=tk.SOLID)
        cal.grid(row=3, column=0, columnspan=2, sticky="ew", padx=8, pady=6)
        self.cal_labels = {}
        readouts = [("Delta Resolver (deg)  → target +90 fwd / -90 rev", "INV_Delta_Resolver_Filtered"),
                    ("Motor Speed (rpm)", "INV_Motor_Speed"),
                    ("Motor Angle Electrical (deg)", "INV_Motor_Angle_Electrical"),
                    ("Last param Read response (Data_Response)", "INV_Data_Response")]
        for i, (label, sig) in enumerate(readouts):
            tk.Label(cal, text=label, bg=PANEL, fg=GREY, anchor="w",
                     font=("Consolas", 10)).grid(row=i, column=0, sticky="w", padx=8, pady=2)
            v = tk.Label(cal, text="--", bg=PANEL, fg=FG, anchor="e",
                         font=("Consolas", 13, "bold"), width=12)
            v.grid(row=i, column=1, sticky="e", padx=8, pady=2)
            if sig:
                self.cal_labels[sig] = v

        # ---- Live inverter status (so you can see why it will/won't spin) ----
        st = tk.LabelFrame(outer, text="  Inverter status (live, selected inverter)  ",
                           bg=PANEL, fg=CYAN, font=("Segoe UI", 10, "bold"),
                           bd=1, relief=tk.SOLID)
        st.grid(row=4, column=0, columnspan=2, sticky="ew", padx=8, pady=6)
        self.ctl_state = {}
        rows = [("Command mode", "cmd_mode"), ("Run mode", "run_mode"),
                ("Enable state", "enable"), ("Lockout", "lockout"),
                ("Inverter state", "state")]
        for i, (label, kk) in enumerate(rows):
            tk.Label(st, text=label, bg=PANEL, fg=GREY, anchor="w",
                     font=("Consolas", 9)).grid(row=i // 2, column=(i % 2) * 2,
                                                sticky="w", padx=(8, 4), pady=2)
            v = tk.Label(st, text="--", bg=PANEL, fg=FG, anchor="w",
                         font=("Consolas", 9, "bold"), width=18)
            v.grid(row=i // 2, column=(i % 2) * 2 + 1, sticky="w", padx=(0, 10), pady=2)
            self.ctl_state[kk] = v

        # Decoded fault lines (active fault names, wrapping across the panel).
        frow = (len(rows) + 1) // 2
        for label, kk in (("POST faults", "post_faults"), ("Run faults", "run_faults")):
            tk.Label(st, text=label, bg=PANEL, fg=GREY, anchor="nw",
                     font=("Consolas", 9)).grid(row=frow, column=0, sticky="nw",
                                                padx=(8, 4), pady=2)
            v = tk.Label(st, text="--", bg=PANEL, fg=FG, anchor="w", justify="left",
                         font=("Consolas", 9, "bold"), wraplength=620)
            v.grid(row=frow, column=1, columnspan=3, sticky="w", padx=(0, 10), pady=2)
            self.ctl_state[kk] = v
            frow += 1

        outer.grid_columnconfigure(0, weight=1)
        outer.grid_columnconfigure(1, weight=1)

    # ---- control helpers ----
    def _ctl_cmd_id(self):
        return self.inverters[self.ctl_inv.get()]["command_id"]

    def _ctl_param_id(self):
        return self.inverters[self.ctl_inv.get()]["param_id"]

    def _ctl_torque_capped(self):
        try:
            cap = abs(float(self.ctl_cap.get()))
        except ValueError:
            cap = 30.0
        try:
            t = float(self.ctl_torque.get())
        except ValueError:
            t = 0.0
        return max(-cap, min(cap, t)), cap

    def _ctl_update_command(self):
        """Rebuild the heartbeat command frame from the current UI state."""
        torque, cap = self._ctl_torque_capped()
        if not self.ctl_enabled:
            torque = 0.0
        self.tx_command_id = self._ctl_cmd_id()
        self.tx_bus = self._ctl_bus()
        self.tx_command = build_command_frame(
            torque_nm=torque, forward=self.ctl_forward.get(),
            enable=self.ctl_enabled, torque_limit_nm=cap)

    def _ctl_arm(self):
        if self.reader is None or self.demo:
            self.ctl_status.config(text="connect to a real ESP32 first", fg=YELLOW)
            return
        # Lockout release: heartbeat begins with a DISABLE command.
        self.ctl_armed = True
        self.ctl_enabled = False
        self.tx_active = True
        self._ctl_update_command()
        self.enable_btn.config(state=tk.NORMAL, bg=GREEN, fg="#06210f")
        self.disable_btn.config(state=tk.NORMAL)
        self.ctl_status.config(text="armed — heartbeat running, inverter DISABLED", fg=YELLOW)

    def _ctl_enable(self):
        if not self.ctl_armed:
            return
        self.ctl_enabled = True
        self._ctl_update_command()
        self.ctl_status.config(text="ENABLED — motor live", fg=ACCENT)

    def _ctl_disable(self):
        self.ctl_enabled = False
        self._ctl_update_command()
        self.ctl_status.config(text="armed — inverter DISABLED", fg=YELLOW)

    def _ctl_estop(self):
        self.ctl_enabled = False
        self.ctl_armed = False
        # send a final disable, then stop the heartbeat
        if self.reader is not None and not self.demo:
            self._send_oneshot(self._ctl_cmd_id(), build_command_frame(enable=False), self._ctl_bus())
            self.reader.stop_heartbeat()
        self.tx_active = False
        self.tx_command = None
        self.enable_btn.config(state=tk.DISABLED, bg=PANEL, fg=GREY)
        self.disable_btn.config(state=tk.DISABLED)
        self.ctl_status.config(text="STOPPED — heartbeat halted", fg=GREY)

    def _ctl_inv_changed(self):
        if self.tx_active:        # don't keep commanding a different inverter
            self._ctl_estop()

    def _ctl_set_can_mode(self):
        """Put the selected inverter into CAN command mode + Torque run mode so it
        will accept torque commands over CAN (it ignores them in the default VSM
        mode). Command mode is EEPROM — power-cycle the inverter to apply it."""
        if self.reader is None or self.demo:
            self.par_status.config(text="connect to a real ESP32 first", fg=YELLOW)
            return
        pid = self._ctl_param_id()
        self._send_oneshot(pid, build_param_write(PARAM_INVERTER_CMD_MODE, 0, signed=False), self._ctl_bus())
        self._send_oneshot(pid, build_param_write(PARAM_INVERTER_RUN_MODE, 0, signed=False), self._ctl_bus())
        self.par_status.config(
            text="Set CAN mode (143=0) + Torque mode (142=0) on "
                 f"{self.ctl_inv.get()}. POWER-CYCLE the inverter so CAN command "
                 "mode takes effect, then check 'Command mode' below reads CAN.",
            fg=YELLOW)

    def _refresh_ctl_state(self):
        """Update the live inverter-status readouts for the selected inverter."""
        if not getattr(self, "ctl_state", None):
            return
        pfx = self.ctl_inv.get()

        def g(name):
            return self.values.get(qualify(pfx, name))

        cm = g("INV_Inverter_Command_Mode")
        if cm is None:
            self.ctl_state["cmd_mode"].config(text="-- (no data)", fg=GREY)
        elif int(cm) == 0:
            self.ctl_state["cmd_mode"].config(text="CAN ✓", fg=GREEN)
        else:
            self.ctl_state["cmd_mode"].config(text="VSM — CMDs IGNORED", fg=ACCENT)

        rm = g("INV_Inverter_Run_Mode")
        self.ctl_state["run_mode"].config(
            text=("--" if rm is None else ("Torque" if int(rm) == 0 else "Speed")), fg=FG)

        en = g("INV_Inverter_Enable_State")
        if en is None:
            self.ctl_state["enable"].config(text="--", fg=GREY)
        else:
            self.ctl_state["enable"].config(text="ENABLED" if int(en) else "disabled",
                                            fg=GREEN if int(en) else GREY)

        lo = g("INV_Inverter_Enable_Lockout")
        if lo is None:
            self.ctl_state["lockout"].config(text="--", fg=GREY)
        else:
            self.ctl_state["lockout"].config(text="LOCKED" if int(lo) else "clear",
                                             fg=YELLOW if int(lo) else FG)

        sv = g("INV_Inverter_State")
        self.ctl_state["state"].config(text="--" if sv is None else f"{int(sv)}", fg=FG)

        self._set_fault_line("post_faults", POST_FAULTS,
                             g("INV_Post_Fault_Lo"), g("INV_Post_Fault_Hi"))
        self._set_fault_line("run_faults", RUN_FAULTS,
                             g("INV_Run_Fault_Lo"), g("INV_Run_Fault_Hi"))

    def _set_fault_line(self, key, table, lo, hi):
        """Show the active fault names (decoded) for a Lo/Hi fault word pair."""
        lbl = self.ctl_state.get(key)
        if lbl is None:
            return
        if lo is None and hi is None:
            lbl.config(text="-- (no data)", fg=GREY)
            return
        bits, names = decode_faults(table, lo, hi)
        if not bits:
            lbl.config(text="none", fg=GREEN)
        elif names:
            lbl.config(text=f"0x{bits:08X}  " + ", ".join(names), fg=ACCENT)
        else:
            lbl.config(text=f"0x{bits:08X}  (reserved bits)", fg=YELLOW)

    # ---- parameter helpers ----
    def _fill_param(self, addr):
        self.par_addr.set(str(addr))

    def _par_write(self):
        if self.reader is None or self.demo:
            self.par_status.config(text="connect to a real ESP32 first", fg=YELLOW)
            return
        try:
            addr = int(self.par_addr.get(), 0)
            val = int(float(self.par_val.get()))
        except ValueError:
            self.par_status.config(text="bad address/value", fg=YELLOW)
            return
        self._send_oneshot(self._ctl_param_id(), build_param_write(addr, val, signed=True), self._ctl_bus())
        self.par_status.config(text=f"wrote {val} to addr {addr} on {self.ctl_inv.get()}", fg=GREEN)

    def _par_read(self):
        if self.reader is None or self.demo:
            self.par_status.config(text="connect to a real ESP32 first", fg=YELLOW)
            return
        try:
            addr = int(self.par_addr.get(), 0)
        except ValueError:
            self.par_status.config(text="bad address", fg=YELLOW)
            return
        self._send_oneshot(self._ctl_param_id(), build_param_read(addr), self._ctl_bus())
        self.par_status.config(text=f"read addr {addr} — see Data_Response in panels", fg=CYAN)

    def _par_fault_clear(self):
        if self.reader is None or self.demo:
            self.par_status.config(text="connect to a real ESP32 first", fg=YELLOW)
            return
        self._send_oneshot(self._ctl_param_id(), build_param_write(PARAM_FAULT_CLEAR, 0, signed=False), self._ctl_bus())
        self.par_status.config(text=f"fault clear sent to {self.ctl_inv.get()}", fg=GREEN)

    # ---- Parameter Manager window ---------------------------------------
    def _open_param_window(self):
        if getattr(self, "_param_win", None) is not None and self._param_win.winfo_exists():
            self._param_win.lift()
            return

        win = tk.Toplevel(self)
        self._param_win = win
        win.title("Inverter Parameter Manager")
        win.configure(bg=BG)
        win.geometry("860x620")

        # top: inverter + search
        top = tk.Frame(win, bg=BG)
        top.pack(side=tk.TOP, fill=tk.X, padx=10, pady=8)
        tk.Label(top, text="Inverter:", bg=BG, fg=GREY).pack(side=tk.LEFT)
        self.pw_inv = tk.StringVar(value=self.ctl_inv.get())
        for pfx in sorted(self.inverters):
            tk.Radiobutton(top, text=TAB_TITLES.get(pfx, pfx), value=pfx, variable=self.pw_inv,
                           bg=BG, fg=FG, selectcolor=PANEL).pack(side=tk.LEFT, padx=4)
        tk.Label(top, text="   Search (name or address):", bg=BG, fg=GREY).pack(side=tk.LEFT)
        self.pw_search = tk.StringVar()
        e = tk.Entry(top, textvariable=self.pw_search, bg=PANEL, fg=FG,
                     insertbackground=FG, relief=tk.FLAT, font=("Consolas", 12), width=26)
        e.pack(side=tk.LEFT, padx=6, ipady=2)
        e.bind("<KeyRelease>", lambda ev: self._pw_filter())

        # middle: table of parameters
        mid = tk.Frame(win, bg=BG)
        mid.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=10, pady=4)
        cols = ("addr", "name", "cat", "unit", "note")
        tree = ttk.Treeview(mid, columns=cols, show="headings", height=14)
        for c, w, txt in [("addr", 60, "Addr"), ("name", 200, "Name"), ("cat", 55, "Cat"),
                          ("unit", 50, "Unit"), ("note", 380, "Note")]:
            tree.heading(c, text=txt)
            tree.column(c, width=w, anchor="w")
        vsb = ttk.Scrollbar(mid, orient="vertical", command=tree.yview)
        tree.configure(yscrollcommand=vsb.set)
        tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        vsb.pack(side=tk.RIGHT, fill=tk.Y)
        tree.bind("<<TreeviewSelect>>", lambda ev: self._pw_select())
        self.pw_tree = tree

        # bottom: detail + read/write
        bot = tk.LabelFrame(win, text="  Selected parameter  ", bg=PANEL, fg=CYAN,
                            font=("Segoe UI", 10, "bold"), bd=1, relief=tk.SOLID)
        bot.pack(side=tk.TOP, fill=tk.X, padx=10, pady=8)
        self.pw_detail = tk.Label(bot, text="select a parameter above", bg=PANEL, fg=GREY,
                                  anchor="w", justify="left", wraplength=820)
        self.pw_detail.grid(row=0, column=0, columnspan=6, sticky="w", padx=8, pady=(6, 2))
        tk.Label(bot, text="Value:", bg=PANEL, fg=GREY).grid(row=1, column=0, sticky="e", padx=(8, 2), pady=6)
        self.pw_value = tk.StringVar()
        tk.Entry(bot, textvariable=self.pw_value, width=12, bg=BG, fg=FG,
                 insertbackground=FG, relief=tk.FLAT).grid(row=1, column=1, sticky="w", pady=6)
        self.pw_unit = tk.Label(bot, text="", bg=PANEL, fg=GREY)
        self.pw_unit.grid(row=1, column=2, sticky="w")
        tk.Button(bot, text="Read", command=self._pw_read, bg="#33415c", fg=FG,
                  relief=tk.FLAT, width=8).grid(row=1, column=3, padx=4)
        tk.Button(bot, text="Write / Save", command=self._pw_write, bg="#5c4633", fg=FG,
                  relief=tk.FLAT, width=12).grid(row=1, column=4, padx=4)
        tk.Button(bot, text="\U0001F4CC Watch", command=self._pw_watch, bg="#2c4a3a", fg=FG,
                  relief=tk.FLAT, width=9).grid(row=1, column=5, padx=4)
        self.pw_status = tk.Label(bot, text="", bg=PANEL, fg=GREY, anchor="w",
                                  justify="left", wraplength=820)
        self.pw_status.grid(row=2, column=0, columnspan=6, sticky="w", padx=8, pady=(2, 8))

        self.pw_selected = None
        self._pw_filter()
        self._pw_poll()      # start the response updater

    def _pw_filter(self):
        q = self.pw_search.get().strip().lower()
        self.pw_tree.delete(*self.pw_tree.get_children())
        for p in cparams.PARAMS:
            if q and not (q in p["name"].lower() or q in str(p["addr"])
                          or q in hex(p["addr"])):
                continue
            self.pw_tree.insert("", "end", iid=str(p["addr"]),
                                values=(p["addr"], p["name"], p["cat"], p["unit"], p["note"]))

    def _pw_select(self):
        sel = self.pw_tree.selection()
        if not sel:
            return
        p = cparams.BY_ADDR.get(int(sel[0]))
        self.pw_selected = p
        self.pw_unit.config(text=p["unit"])
        cat = "EEPROM (write motor-off; effective after power-cycle unless immediate)" \
            if p["cat"] == "EEP" else "Command parameter (takes effect live)"
        self.pw_detail.config(
            text=f"{p['name']}   addr {p['addr']} (0x{p['addr']:X})   ×{p['scale']}"
                 f"{' signed' if p['signed'] else ''}   {cat}\n{p['note']}", fg=FG)
        self.pw_status.config(text="", fg=GREY)

    def _pw_read(self):
        if self.pw_selected is None:
            return
        if self.reader is None or self.demo:
            self.pw_status.config(text="connect to a real ESP32 first", fg=YELLOW)
            return
        pid = self.inverters[self.pw_inv.get()]["param_id"]
        self._send_oneshot(pid, build_param_read(self.pw_selected["addr"]),
                           self.inverters[self.pw_inv.get()]["bus"])
        self.pw_status.config(text=f"read addr {self.pw_selected['addr']} sent…", fg=CYAN)

    def _pw_write(self):
        if self.pw_selected is None:
            return
        if self.reader is None or self.demo:
            self.pw_status.config(text="connect to a real ESP32 first", fg=YELLOW)
            return
        p = self.pw_selected
        try:
            eng = float(self.pw_value.get())
        except ValueError:
            self.pw_status.config(text="enter a numeric value", fg=YELLOW)
            return
        if p["cat"] == "EEP" and getattr(self, "ctl_enabled", False):
            self.pw_status.config(text="DISABLE the inverter before writing EEPROM params", fg=ACCENT)
            return
        raw = cparams.to_wire(p, eng)
        pid = self.inverters[self.pw_inv.get()]["param_id"]
        self._send_oneshot(pid, build_param_write(p["addr"], raw, signed=p["signed"]),
                           self.inverters[self.pw_inv.get()]["bus"])
        self.pw_status.config(text=f"wrote {eng}{(' ' + p['unit']) if p['unit'] else ''} "
                              f"(raw {raw}) to addr {p['addr']} — click Read to confirm", fg=GREEN)

    def _pw_watch(self):
        """Pin the selected parameter to the Watch panel (auto-read while watched)."""
        if self.pw_selected is None:
            return
        key = f"@param:{self.pw_inv.get()}:{self.pw_selected['addr']}"
        self._watch_pin(key)
        self.pw_status.config(text=f"added {self.pw_selected['name']} to Watch "
                              "(auto-reads while pinned)", fg=GREEN)

    def _pw_poll(self):
        """Show the inverter's last parameter response in the window."""
        win = getattr(self, "_param_win", None)
        if win is None or not win.winfo_exists():
            return
        pfx = self.pw_inv.get()
        addr = self.values.get(qualify(pfx, "INV_Parameter_Address_Response"))
        data = self.values.get(qualify(pfx, "INV_Data_Response"))
        ok = self.values.get(qualify(pfx, "INV_Write_Success"))
        if addr is not None and data is not None:
            a = int(addr)
            p = cparams.BY_ADDR.get(a)
            if p is not None:
                eng = cparams.from_wire(p, int(data))
                txt = f"response: addr {a} ({p['name']}) = {eng}{(' ' + p['unit']) if p['unit'] else ''}  (raw {int(data)})"
            else:
                txt = f"response: addr {a} = raw {int(data)}"
            if ok is not None:
                txt += f"   write_success={int(ok)}"
            self.pw_status.config(text=txt, fg=GREEN if (ok in (None, 1)) else ACCENT)
        win.after(300, self._pw_poll)

    # ---- connection ------------------------------------------------------
    def _toggle_connect(self):
        if self.reader and self.reader.is_alive():
            if getattr(self, "tx_active", False):
                self._ctl_estop()       # never leave the motor commanded
            self.reader.stop()
            self.reader = None
            self.connect_btn.config(text="Connect", bg=GREEN, fg="#06210f")
            self.status_lbl.config(text="disconnecting...", fg=GREY)
            return

        if self.demo:
            self.reader = DemoReader(self.ordered_msgs, self.prefix_bus,
                                     self.out_queue, self.status_queue)
            self.reader.start()
            self.connect_btn.config(text="Disconnect", bg=ACCENT, fg="white")
            self.status_lbl.config(text="DEMO mode", fg=YELLOW)
            return

        # Map the (ordered) interface names to bus indices: first -> bus 0, etc.
        names = [self.ch0_var.get().strip(), self.ch1_var.get().strip()]
        channels = [(i, n) for i, n in enumerate(names) if n]
        if not channels:
            self.status_lbl.config(text="enter a CAN interface (e.g. can0)", fg=YELLOW)
            return
        try:
            self.bitrate = int(self.bitrate_var.get())   # used for bus-load %
        except ValueError:
            self.status_lbl.config(text="bad bitrate", fg=YELLOW)
            return

        self.reader = CanReader(channels, self.bitrate, self.frame_map,
                                self.out_queue, self.status_queue)
        self.reader.start()
        self.connect_btn.config(text="Disconnect", bg=ACCENT, fg="white")
        self.status_lbl.config(text=f"opening {', '.join(n for _, n in channels)}...",
                               fg=YELLOW)

    # ---- main poll loop --------------------------------------------------
    def _poll(self):
        # drain status messages
        while True:
            try:
                kind, payload = self.status_queue.get_nowait()
            except queue.Empty:
                break
            if kind == "connected":
                self.status_lbl.config(text=f"connected {payload}", fg=GREEN)
            elif kind == "disconnected":
                self.status_lbl.config(text="disconnected", fg=GREY)
                self.connect_btn.config(text="Connect", bg=GREEN, fg="#06210f")
            elif kind == "error":
                self.status_lbl.config(text=payload, fg=ACCENT)
                self.connect_btn.config(text="Connect", bg=GREEN, fg="#06210f")

        # drain decoded frames
        now = time.time()
        got = False
        while True:
            try:
                item = self.out_queue.get_nowait()
            except queue.Empty:
                break
            got = True
            if item[0] == "unknown":
                self.unknown_count += 1
                self._fps_window.append(now)
                bus = item[4] if len(item) > 4 else 0
                self._load_window.append((now, item[3] if len(item) > 3 else 0, bus))
                continue
            _, msg_name, decoded, bits, bus = item
            prev = self.msg_last_rx.get(msg_name)
            if prev is not None:
                dt = now - prev
                if 0 < dt < 5:            # ignore gaps from reconnects/pauses
                    p = self.msg_period.get(msg_name)
                    self.msg_period[msg_name] = dt if p is None else 0.85 * p + 0.15 * dt
            self.msg_last_rx[msg_name] = now
            self.frame_count += 1
            self._fps_window.append(now)
            self._load_window.append((now, bits, bus))
            for name, value in decoded.items():
                self.values[name] = value
            if self.csv_writer is not None:
                self._write_log_row(now, msg_name)

        if got:
            self._refresh_values()

        # frames/sec + estimated per-bus load over a 1s sliding window
        self._fps_window = [t for t in self._fps_window if now - t <= 1.0]
        self._load_window = [(t, b, k) for (t, b, k) in self._load_window if now - t <= 1.0]
        per_bus = {}
        for _t, b, k in self._load_window:
            per_bus[k] = per_bus.get(k, 0) + b
        worst = 0.0
        parts = []
        for k in self.buses:
            ld = min(100.0, per_bus.get(k, 0) / self.bitrate * 100)
            worst = max(worst, ld)
            parts.append(f"b{k} {ld:.0f}%")
        load_color = GREEN if worst < 50 else (YELLOW if worst < 80 else ACCENT)
        self.fps_lbl.config(
            text=f"{len(self._fps_window)} fps  |  {'  '.join(parts)}  |  unk {self.unknown_count}",
            fg=load_color)
        if self.csv_writer is not None:
            self.log_lbl.config(text=f"{os.path.basename(self.log_path)}  ({self.log_rows} rows)",
                                fg=GREEN)

        self._mark_stale(now)
        self._service_heartbeat()
        self._service_watch_params(now)
        if self.charts:
            self._update_charts()
        self.after(100, self._poll)

    def _service_watch_params(self, now):
        """Round-robin a read request for each watched parameter (~3 Hz) so
        their values refresh. Reads are safe at any time."""
        if self.reader is None or self.demo or now - self._watch_rr_last < 0.33:
            return
        params = [k for k in self.watch_signals if k.startswith("@param:")]
        if not params:
            return
        self._watch_rr_last = now
        self._watch_rr = (self._watch_rr + 1) % len(params)
        _, pfx, addr = params[self._watch_rr].split(":")
        if pfx in self.inverters:
            self._send_oneshot(self.inverters[pfx]["param_id"], build_param_read(int(addr)),
                               self.inverters[pfx]["bus"])

    # ---- inverter transmit / heartbeat ----------------------------------
    def _service_heartbeat(self):
        """Refresh the repeating command frame (~10 Hz) and pet the deadman.
        CanReader re-sends it on CAN every 20 ms and drops it if this stops."""
        if not self.tx_active:
            return
        if self.reader is None or not getattr(self.reader, "is_alive", lambda: False)():
            return
        self._ctl_update_command()      # reflect live torque/direction edits
        if self.tx_command is not None:
            self.reader.set_heartbeat(self.tx_bus, self.tx_command_id, self.tx_command)

    def _send_oneshot(self, can_id, data, bus=0):
        """Send a single CAN frame on a given bus (param write/read, fault clear)."""
        if self.reader is None or self.demo:
            return False
        return self.reader.send_frame(bus, can_id, data)

    def _ctl_bus(self):
        return self.inverters[self.ctl_inv.get()]["bus"]

    # ---- CSV logging -----------------------------------------------------
    def _toggle_log(self):
        if self.csv_writer is not None:
            self._stop_log()
            return
        os.makedirs(os.path.join(HERE, "logs"), exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(HERE, "logs", f"ev4_log_{stamp}.csv")
        try:
            self.csv_file = open(path, "w", newline="", encoding="utf-8")
        except OSError as e:
            self.log_lbl.config(text=f"log error: {e}", fg=ACCENT)
            return
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["datetime", "elapsed_s", "trigger_msg"] + self.log_signals)
        self.log_rows = 0
        self.log_start = time.time()
        self.log_path = path
        self.log_btn.config(text="Stop Log", bg=ACCENT, fg="white")
        self.log_lbl.config(text=os.path.basename(path), fg=GREEN)

    def _stop_log(self):
        if self.csv_file is not None:
            try:
                self.csv_file.close()
            except OSError:
                pass
        self.csv_file = None
        self.csv_writer = None
        self.log_btn.config(text="Log CSV", bg=PANEL, fg=FG)
        self.log_lbl.config(text=f"saved {self.log_rows} rows", fg=GREY)

    def _write_log_row(self, now, msg_name):
        row = [datetime.datetime.now().isoformat(timespec="milliseconds"),
               f"{now - self.log_start:.3f}", msg_name]
        for s in self.log_signals:
            v = self.values.get(s, "")
            row.append(float(v) if isinstance(v, bool) else v)
        try:
            self.csv_writer.writerow(row)
            self.log_rows += 1
        except (OSError, ValueError):
            self._stop_log()

    def _fmt(self, value, unit):
        try:
            f = float(value)
        except (TypeError, ValueError):
            return f"{value}{(' ' + unit) if unit else ''}"
        if f == int(f):
            txt = f"{int(f)}"
        else:
            txt = f"{f:.2f}"
        return txt + (f" {unit}" if unit else "")

    def _refresh_values(self):
        for name, (lbl, unit) in self.value_labels.items():
            if name not in self.values:
                continue
            value = self.values[name]
            lbl.config(text=self._fmt(value, unit))
            if is_fault_signal(name):
                try:
                    active = float(value) != 0
                except (TypeError, ValueError):
                    active = bool(value)
                lbl.config(fg=ACCENT if active else GREEN)

        for sig, (lbl, fmt) in self.key_labels.items():
            if sig in self.values:
                try:
                    lbl.config(text=fmt.format(float(self.values[sig])))
                except (TypeError, ValueError):
                    lbl.config(text=str(self.values[sig]))

        # live values in the Lookup tab
        for name, (lbl, unit) in self.lookup_value_labels.items():
            if name not in self.values:
                continue
            value = self.values[name]
            lbl.config(text=self._fmt(value, unit))
            if is_fault_signal(name):
                try:
                    active = float(value) != 0
                except (TypeError, ValueError):
                    active = bool(value)
                lbl.config(fg=ACCENT if active else GREEN)

        # live values in the docked Watch panel (signals + params)
        for key, (lbl, unit) in self.watch_value_labels.items():
            if key.startswith("@param:"):
                _, pfx, addr = key.split(":")
                addr = int(addr)
                rxaddr = self.values.get(qualify(pfx, "INV_Parameter_Address_Response"))
                data = self.values.get(qualify(pfx, "INV_Data_Response"))
                if rxaddr is not None and data is not None and int(rxaddr) == addr:
                    p = cparams.BY_ADDR.get(addr)
                    self._watch_param_cache[key] = (
                        cparams.from_wire(p, int(data)) if p else int(data))
                if key in self._watch_param_cache:
                    lbl.config(text=self._fmt(self._watch_param_cache[key], unit))
            elif key in self.values:
                value = self.values[key]
                lbl.config(text=self._fmt(value, unit))
                # match the bus/lookup tabs: fault signals red when set, green clear
                if is_fault_signal(key):
                    try:
                        active = float(value) != 0
                    except (TypeError, ValueError):
                        active = bool(value)
                    lbl.config(fg=ACCENT if active else GREEN)

        # live calibration readouts on the Control tab (selected inverter)
        cal = getattr(self, "cal_labels", None)
        if cal:
            pfx = self.ctl_inv.get()
            for base, lbl in cal.items():
                key = qualify(pfx, base)
                if key in self.values:
                    lbl.config(text=self._fmt(self.values[key], ""))
            self._refresh_ctl_state()

    def _mark_stale(self, now):
        for name, (frame, base_color) in self.panel_titles.items():
            last = self.msg_last_rx.get(name)
            # "stale" is relative to the message's own rate, so a normally-slow
            # message (e.g. 0.5 Hz) doesn't keep flashing yellow — it only goes
            # yellow after missing ~3 expected frames (min 1 s).
            period = self.msg_period.get(name)
            stale_after = max(STALE_AFTER, 3 * period) if period else STALE_AFTER
            if last is None:
                frame.config(fg=GREY)               # never seen
            elif now - last > stale_after:
                frame.config(fg=YELLOW)             # data actually stalled
            else:
                frame.config(fg=base_color)         # live (source color)

    @staticmethod
    def _fmt_rate(period):
        """Compact 'how often' string from an averaged period (s)."""
        if not period or period <= 0:
            return ""
        hz = 1.0 / period
        if hz >= 100:
            return f"{hz:.0f} Hz"
        if hz >= 1:
            return f"{hz:.0f} Hz ({period * 1000:.0f} ms)"
        return f"{hz:.2f} Hz ({period * 1000:.0f} ms)"

    def _update_rates(self):
        """Refresh the averaged send-rate shown in each panel title (~1 Hz)."""
        now = time.time()
        for name, (frame, _color) in self.panel_titles.items():
            last = self.msg_last_rx.get(name)
            period = self.msg_period.get(name)
            if last is None or period is None or now - last > max(STALE_AFTER, 3 * period):
                rate = ""                            # not live -> no rate
            else:
                rate = self._fmt_rate(period)
            base = self._panel_base_text.get(name, "")
            frame.config(text=base + ("  " + rate if rate else ""))
        self.after(1000, self._update_rates)

    def on_close(self):
        if getattr(self, "tx_active", False):
            self._ctl_estop()           # disable motor + stop heartbeat
        if self.csv_writer is not None:
            self._stop_log()
        if self.reader:
            self.reader.stop()
        self.destroy()


def load_sources():
    """Load every DBC in DBC_SOURCES. Returns list of (prefix, Database, bus).
    Missing/broken files are warned about and skipped rather than fatal."""
    sources = []
    for fname, prefix, bus in DBC_SOURCES:
        path = os.path.join(HERE, fname)
        if not os.path.exists(path):
            print(f"WARN: DBC not found, skipping: {path}")
            continue
        try:
            sources.append((prefix, cantools.database.load_file(path), bus))
            print(f"loaded {fname} as '{prefix or 'vehicle'}' on bus {bus}")
        except Exception as e:
            print(f"WARN: failed to load {fname}: {e}")
    if not sources:
        sys.exit("No DBC files could be loaded.")
    return sources


def main():
    ap = argparse.ArgumentParser(description="BER EV4 CAN dashboard (Raspberry Pi / SocketCAN)")
    ap.add_argument("--channels", nargs="+", default=DEFAULT_CHANNELS,
                    metavar="IFNAME",
                    help="SocketCAN interfaces in bus order (default: can0 can1). "
                         "First -> bus 0 (Vehicle/INV2), second -> bus 1 (INV1).")
    ap.add_argument("--bitrate", type=int, default=DEFAULT_BITRATE,
                    help="bus bit rate, used for the bus-load %% estimate "
                         "(the interface itself is configured by setup_can.sh)")
    ap.add_argument("--no-connect", action="store_true",
                    help="start without auto-connecting (click Connect in the UI)")
    ap.add_argument("--demo", action="store_true",
                    help="generate fake data instead of reading CAN (no hardware needed)")
    args = ap.parse_args()

    sources = load_sources()
    app = Dashboard(sources, args.channels, args.bitrate, demo=args.demo)
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    if not args.no_connect:
        app.after(300, app._toggle_connect)   # auto-connect / auto-start demo
    app.mainloop()


if __name__ == "__main__":
    main()
