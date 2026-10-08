#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
SRNE Integrated Inverter + 4x Ping1D + MQTT
Version: v1.7.0

Features
--------
- SRNE inverter via RS232 / Modbus RTU FC03
- Robust Modbus frame resynchronization
- 4 independent Ping1D TCP connections
- Ping1D distance, confidence, range, gain, and profile (message 1300)
- Tabs: Dashboard, Ping1D 1..4, Communication
- MQTT publisher with electrical + 4 Ping1D telemetry
- Read-only Modbus operation

Requirements
------------
    pip install pyserial paho-mqtt
"""

import json
import queue
import socket
import struct
import threading
import time
import tkinter as tk
from collections import deque
from tkinter import ttk, messagebox

import serial
from serial.tools import list_ports
import paho.mqtt.client as mqtt


# =============================================================================
# APPLICATION / MQTT CONFIGURATION
# =============================================================================

APP_NAME = "SRNE Integrated Inverter + Ping1D"
APP_VERSION = "v1.7.0"

MQTT_BROKER_HOST = "9bb0622065e44988ba6ba779032dd29d.s1.eu.hivemq.cloud"
MQTT_BROKER_PORT = 8883
MQTT_USERNAME = "ha_rmi"
MQTT_PASSWORD = "Password#1"
MQTT_TOPIC = "hydrophone/electrical/solarpanel"

DEFAULT_PING_ENDPOINTS = [
    ("PING1D 1", "192.168.3.112", 8080),
    ("PING1D 2", "192.168.3.122", 8080),
    ("PING1D 3", "192.168.3.132", 8080),
    ("PING1D 4", "192.168.3.142", 8080),
]

PING_GENERAL_REQUEST = 6
PING1D_DISTANCE_SIMPLE = 1211
PING1D_DISTANCE = 1212
PING1D_PROFILE = 1300

PING_DISTANCE_REQUEST_PERIOD_S = 0.10
PING_PROFILE_REQUEST_PERIOD_S = 0.50
PING_RECONNECT_DELAY_S = 1.0
PING_STREAM_TIMEOUT_S = 2.0
MAX_MQTT_PROFILE_SAMPLES = 128


# =============================================================================
# SRNE REGISTER MAP
# =============================================================================

REGISTER_MAP = {
    "ac_output_voltage": {
        "addr": 0x00D2, "scale": 0.1, "signed": False, "unit": "V",
        "label": "AC Output Voltage",
    },
    "ac_output_current": {
        "addr": 0x00D3, "scale": 0.1, "signed": False, "unit": "A",
        "label": "AC Output Current",
    },
    "ac_output_active_power": {
        "addr": 0x00D5, "scale": 1.0, "signed": True, "unit": "W",
        "label": "AC Output Active Power",
    },
    "battery_voltage": {
        "addr": 0x00D7, "scale": 0.1, "signed": False, "unit": "V",
        "label": "Battery Voltage",
    },
    # Battery current is read only to determine energy direction.
    # It is intentionally NOT included in the MQTT electrical payload.
    "battery_current_internal": {
        "addr": 0x00D8, "scale": 0.1, "signed": True, "unit": "A",
        "label": "Battery Current",
    },
    "battery_power": {
        "addr": 0x00D9, "scale": 1.0, "signed": True, "unit": "W",
        "label": "Battery Power",
    },
}


# =============================================================================
# MODBUS RTU
# =============================================================================

def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_read_request(slave_id: int, start_addr: int, count: int) -> bytes:
    frame = struct.pack(">BBHH", slave_id, 0x03, start_addr, count)
    return frame + struct.pack("<H", crc16_modbus(frame))


def to_signed16(value: int) -> int:
    return value - 0x10000 if value & 0x8000 else value


class SerialModbusClient:
    def __init__(self):
        self.ser = None
        self.lock = threading.Lock()

    @property
    def is_open(self):
        return self.ser is not None and self.ser.is_open

    def connect(self, port: str, baudrate: int, timeout: float = 0.7):
        self.disconnect()
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
            write_timeout=timeout,
        )
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def disconnect(self):
        if self.ser is not None:
            try:
                if self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass
        self.ser = None

    @staticmethod
    def _find_valid_response(buffer: bytes, slave_id: int, byte_count: int):
        n = len(buffer)
        for i in range(n):
            if buffer[i] != slave_id:
                continue
            if i + 3 > n:
                continue

            fc = buffer[i + 1]

            if fc == 0x03:
                if buffer[i + 2] != byte_count:
                    continue
                frame_len = 3 + byte_count + 2
                if i + frame_len > n:
                    continue
                frame = buffer[i:i + frame_len]
                if struct.unpack("<H", frame[-2:])[0] == crc16_modbus(frame[:-2]):
                    return frame

            elif fc == 0x83:
                if i + 5 > n:
                    continue
                frame = buffer[i:i + 5]
                if struct.unpack("<H", frame[-2:])[0] == crc16_modbus(frame[:-2]):
                    raise RuntimeError(
                        f"Modbus exception 0x{frame[2]:02X}"
                    )

        return None

    def read_registers(self, slave_id: int, start_addr: int, count: int):
        if not self.is_open:
            raise RuntimeError("Serial port is not connected.")

        if not 1 <= count <= 125:
            raise ValueError("Register count must be 1..125.")

        request = build_read_request(slave_id, start_addr, count)
        expected_bytes = count * 2

        with self.lock:
            self.ser.reset_input_buffer()
            self.ser.write(request)
            self.ser.flush()

            deadline = time.monotonic() + max(float(self.ser.timeout or 0.7), 0.25)
            rx = bytearray()
            response = None

            while time.monotonic() < deadline:
                waiting = self.ser.in_waiting
                if waiting:
                    rx.extend(self.ser.read(waiting))
                else:
                    old_timeout = self.ser.timeout
                    try:
                        self.ser.timeout = 0.05
                        chunk = self.ser.read(1)
                    finally:
                        self.ser.timeout = old_timeout

                    if chunk:
                        rx.extend(chunk)
                    else:
                        time.sleep(0.003)

                response = self._find_valid_response(
                    bytes(rx), slave_id, expected_bytes
                )
                if response is not None:
                    break

        if response is None:
            raw_hex = bytes(rx).hex(" ").upper() if rx else "<empty>"
            raise TimeoutError(
                f"No valid Modbus response. Slave={slave_id}, RX={raw_hex}"
            )

        payload = response[3:-2]
        values = [
            struct.unpack(">H", payload[i:i + 2])[0]
            for i in range(0, len(payload), 2)
        ]
        return values, request, response


# =============================================================================
# PING PROTOCOL
# =============================================================================

def ping_build_message(message_id: int, payload=b"", src_id=0, dst_id=0):
    payload = bytes(payload)
    header = struct.pack(
        "<BBHHBB",
        ord("B"),
        ord("R"),
        len(payload),
        int(message_id),
        int(src_id),
        int(dst_id),
    )
    body = header + payload
    checksum = sum(body) & 0xFFFF
    return body + struct.pack("<H", checksum)


def ping_build_request(message_id: int):
    return ping_build_message(
        PING_GENERAL_REQUEST,
        struct.pack("<H", int(message_id)),
    )


class PingStreamParser:
    def __init__(self):
        self.buffer = bytearray()
        self.good_packets = 0
        self.bad_checksums = 0

    def reset(self):
        self.buffer.clear()
        self.good_packets = 0
        self.bad_checksums = 0

    def feed(self, data: bytes):
        if data:
            self.buffer.extend(data)

        messages = []
        while True:
            idx = self.buffer.find(b"BR")
            if idx < 0:
                if self.buffer[-1:] == b"B":
                    self.buffer[:] = b"B"
                else:
                    self.buffer.clear()
                break

            if idx > 0:
                del self.buffer[:idx]

            if len(self.buffer) < 8:
                break

            payload_len = struct.unpack_from("<H", self.buffer, 2)[0]
            if payload_len > 65500:
                del self.buffer[0]
                continue

            frame_len = 8 + payload_len + 2
            if len(self.buffer) < frame_len:
                break

            frame = bytes(self.buffer[:frame_len])
            rx_checksum = struct.unpack_from("<H", frame, frame_len - 2)[0]
            calc_checksum = sum(frame[:-2]) & 0xFFFF

            if rx_checksum != calc_checksum:
                self.bad_checksums += 1
                del self.buffer[0]
                continue

            msg_id = struct.unpack_from("<H", frame, 4)[0]
            payload = frame[8:-2]
            messages.append((msg_id, payload))
            self.good_packets += 1
            del self.buffer[:frame_len]

        return messages


def downsample_profile(profile, max_samples=MAX_MQTT_PROFILE_SAMPLES):
    if not profile:
        return []
    n = len(profile)
    if n <= max_samples:
        return list(profile)

    out = []
    for i in range(max_samples):
        idx = int(i * (n - 1) / max(1, max_samples - 1))
        out.append(int(profile[idx]))
    return out


class Ping1DClient(threading.Thread):
    def __init__(self, index, name, host, port, event_queue):
        super().__init__(daemon=True)
        self.index = index
        self.name = name
        self.host = host
        self.port = int(port)
        self.event_queue = event_queue

        self.stop_event = threading.Event()
        self.sock = None
        self.parser = PingStreamParser()

        self.lock = threading.Lock()
        self.status = "DISCONNECTED"
        self.last_error = ""
        self.last_data_monotonic = None

        self.distance_m = None
        self.confidence = None
        self.scan_start_m = None
        self.scan_length_m = None
        self.gain = None
        self.profile = []
        self.profile_seq = 0
        self.profile_time = None

    def _set_status(self, status, error=""):
        with self.lock:
            self.status = status
            self.last_error = error
        self.event_queue.put(
            ("ping_status", self.index, status, error)
        )

    def stop(self):
        self.stop_event.set()
        sock = self.sock
        self.sock = None
        if sock:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                sock.close()
            except Exception:
                pass

    def _handle_message(self, msg_id, payload):
        now = time.monotonic()

        if msg_id == PING1D_DISTANCE and len(payload) >= 24:
            distance, confidence, _tx, ping_no, start_mm, length_mm, gain = \
                struct.unpack_from("<IHHIIII", payload, 0)

            with self.lock:
                self.distance_m = float(distance) / 1000.0
                self.confidence = float(confidence)
                self.scan_start_m = float(start_mm) / 1000.0
                self.scan_length_m = float(length_mm) / 1000.0
                self.gain = int(gain)
                self.last_data_monotonic = now
                self.status = "STREAMING"

            self.event_queue.put(
                (
                    "ping_data",
                    self.index,
                    {
                        "distance_m": float(distance) / 1000.0,
                        "confidence": float(confidence),
                        "scan_start_m": float(start_mm) / 1000.0,
                        "scan_length_m": float(length_mm) / 1000.0,
                        "gain": int(gain),
                        "ping_number": int(ping_no),
                    },
                )
            )

        elif msg_id == PING1D_DISTANCE_SIMPLE and len(payload) >= 5:
            distance, confidence = struct.unpack_from("<IB", payload, 0)
            with self.lock:
                self.distance_m = float(distance) / 1000.0
                self.confidence = float(confidence)
                self.last_data_monotonic = now
                self.status = "STREAMING"

            self.event_queue.put(
                (
                    "ping_data",
                    self.index,
                    {
                        "distance_m": float(distance) / 1000.0,
                        "confidence": float(confidence),
                    },
                )
            )

        elif msg_id == PING1D_PROFILE and len(payload) >= 26:
            (
                distance,
                confidence,
                _tx,
                ping_no,
                start_mm,
                length_mm,
                gain,
                profile_len,
            ) = struct.unpack_from("<IHHIIIIH", payload, 0)

            available = max(0, len(payload) - 26)
            n = min(int(profile_len), available)
            profile = list(payload[26:26 + n])

            with self.lock:
                self.distance_m = float(distance) / 1000.0
                self.confidence = float(confidence)
                self.scan_start_m = float(start_mm) / 1000.0
                self.scan_length_m = float(length_mm) / 1000.0
                self.gain = int(gain)
                self.profile = profile
                self.profile_seq += 1
                self.profile_time = time.time()
                self.last_data_monotonic = now
                self.status = "STREAMING"
                seq = self.profile_seq

            self.event_queue.put(
                (
                    "ping_profile",
                    self.index,
                    {
                        "distance_m": float(distance) / 1000.0,
                        "confidence": float(confidence),
                        "scan_start_m": float(start_mm) / 1000.0,
                        "scan_length_m": float(length_mm) / 1000.0,
                        "gain": int(gain),
                        "ping_number": int(ping_no),
                        "profile": downsample_profile(profile, 160),
                        "profile_seq": seq,
                    },
                )
            )

    def snapshot(self):
        with self.lock:
            if self.last_data_monotonic is None:
                age_s = None
            else:
                age_s = max(0.0, time.monotonic() - self.last_data_monotonic)

            status = self.status
            if status == "STREAMING" and age_s is not None and age_s > PING_STREAM_TIMEOUT_S:
                status = "NO DATA"

            return {
                "name": self.name,
                "ip": self.host,
                "port": self.port,
                "status": status,
                "distance_m": self.distance_m,
                "confidence": self.confidence,
                "scan_start_m": self.scan_start_m,
                "scan_length_m": self.scan_length_m,
                "gain": self.gain,
                "profile": downsample_profile(
                    self.profile, MAX_MQTT_PROFILE_SAMPLES
                ),
                "profile_seq": self.profile_seq,
                "profile_timestamp": self.profile_time,
                "age_s": age_s,
            }

    def run(self):
        while not self.stop_event.is_set():
            self._set_status("CONNECTING")
            self.parser.reset()

            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(3.0)
                sock.connect((self.host, self.port))
                sock.settimeout(0.05)
                self.sock = sock
                self._set_status("WAITING DATA")

                next_distance = 0.0
                next_profile = 0.0

                while not self.stop_event.is_set():
                    now = time.monotonic()

                    if now >= next_distance:
                        sock.sendall(ping_build_request(PING1D_DISTANCE))
                        next_distance = now + PING_DISTANCE_REQUEST_PERIOD_S

                    if now >= next_profile:
                        sock.sendall(ping_build_request(PING1D_PROFILE))
                        next_profile = now + PING_PROFILE_REQUEST_PERIOD_S

                    try:
                        chunk = sock.recv(8192)
                        if chunk == b"":
                            raise ConnectionError("TCP peer closed connection")

                        for msg_id, payload in self.parser.feed(chunk):
                            self._handle_message(msg_id, payload)

                    except socket.timeout:
                        pass

                    time.sleep(0.002)

            except Exception as exc:
                if not self.stop_event.is_set():
                    self._set_status("ERROR", str(exc))

            finally:
                sock = self.sock
                self.sock = None
                if sock:
                    try:
                        sock.close()
                    except Exception:
                        pass

            if self.stop_event.is_set():
                break

            self._set_status("RECONNECTING")
            self.stop_event.wait(PING_RECONNECT_DELAY_S)

        self._set_status("DISCONNECTED")


# =============================================================================
# MQTT PUBLISHER
# =============================================================================

class IntegratedMQTTPublisher:
    def __init__(self):
        self.client = mqtt.Client(
            client_id="srne_ping1d_integrated_publisher",
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
        )
        self.client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        self.client.tls_set()
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect

        self.connected = False
        self.started = False
        self.last_error = ""
        self.lock = threading.Lock()

    def _on_connect(self, client, userdata, flags, reason_code, properties=None):
        try:
            failed = bool(reason_code.is_failure)
        except Exception:
            failed = reason_code != 0

        self.connected = not failed
        self.last_error = "" if self.connected else str(reason_code)

    def _on_disconnect(
        self, client, userdata, disconnect_flags, reason_code, properties=None
    ):
        self.connected = False
        try:
            failed = bool(reason_code.is_failure)
        except Exception:
            failed = reason_code != 0
        if failed:
            self.last_error = str(reason_code)

    def start(self):
        if self.started:
            return
        self.client.connect_async(
            MQTT_BROKER_HOST,
            MQTT_BROKER_PORT,
            keepalive=60,
        )
        self.client.loop_start()
        self.started = True

    def stop(self):
        if not self.started:
            return
        try:
            self.client.disconnect()
            self.client.loop_stop()
        finally:
            self.started = False
            self.connected = False

    def publish(self, payload):
        if not self.connected:
            raise RuntimeError("MQTT broker is not connected.")

        text = json.dumps(payload, separators=(",", ":"), allow_nan=False)
        with self.lock:
            result = self.client.publish(
                MQTT_TOPIC,
                text,
                qos=1,
                retain=False,
            )
        return result, len(text.encode("utf-8"))


# =============================================================================
# GUI
# =============================================================================

class IntegratedApp(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title(f"{APP_NAME} - {APP_VERSION}")

        self.update_idletasks()
        self.screen_w = max(800, int(self.winfo_screenwidth()))
        self.screen_h = max(500, int(self.winfo_screenheight()))
        self.compact = self.screen_w <= 1440 or self.screen_h <= 850

        target_w = min(1560, max(980, int(self.screen_w * 0.97)))
        target_h = min(960, max(620, int(self.screen_h * 0.92)))
        self.geometry(
            f"{target_w}x{target_h}+"
            f"{max(0, (self.screen_w-target_w)//2)}+"
            f"{max(0, (self.screen_h-target_h)//3)}"
        )
        self.minsize(min(980, target_w), min(620, target_h))

        self.event_queue = queue.Queue()
        self.serial_client = SerialModbusClient()

        self.modbus_thread = None
        self.modbus_stop_event = threading.Event()
        self.modbus_polling = False

        self.latest_electrical = {
            "ac_output_voltage": None,
            "ac_output_current": None,
            "ac_output_active_power": None,
            "battery_voltage": None,
            "battery_power": None,
            "battery_energy_direction": "UNKNOWN",
        }
        self.latest_modbus_timestamp = None

        self.ping_clients = [None, None, None, None]
        self.ping_history = [deque(maxlen=120) for _ in range(4)]

        self.mqtt = IntegratedMQTTPublisher()
        self.mqtt_publish_interval_ms = 1000
        self.last_mqtt_payload_size = 0

        self._setup_style()
        self._build_ui()
        self.refresh_ports()

        try:
            self.mqtt.start()
        except Exception as exc:
            self.log(f"MQTT INIT ERROR: {exc}")

        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(100, self._process_events)
        self.after(500, self._refresh_statuses)
        self.after(self.mqtt_publish_interval_ms, self._mqtt_tick)

    # -------------------------------------------------------------------------
    # Style
    # -------------------------------------------------------------------------

    def _setup_style(self):
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except Exception:
            pass

        self.bg = "#EEF2F6"
        self.panel = "#FFFFFF"
        self.border = "#D6DEE8"
        self.text = "#14243A"
        self.muted = "#68788D"
        self.accent = "#0A6E99"
        self.accent_dark = "#074F70"
        self.header = "#102A43"
        self.header2 = "#173F61"
        self.success = "#15805C"
        self.warning = "#B7791F"
        self.danger = "#B42318"

        self.configure(bg=self.bg)

        base = 8 if self.compact else 10
        style.configure(".", font=("Segoe UI", base))
        style.configure("TFrame", background=self.bg)
        style.configure("TLabel", background=self.bg, foreground=self.text)
        style.configure("TButton", padding=(8, 5) if self.compact else (12, 7))
        style.configure(
            "Primary.TButton",
            font=("Segoe UI Semibold", base),
            padding=(9, 5) if self.compact else (13, 7),
        )
        style.configure(
            "Modern.TNotebook",
            background=self.bg,
            borderwidth=0,
        )
        style.configure(
            "Modern.TNotebook.Tab",
            background="#DCE5EE",
            foreground="#42566D",
            padding=(10, 6) if self.compact else (14, 8),
            font=("Segoe UI Semibold", 8 if self.compact else 9),
        )
        style.map(
            "Modern.TNotebook.Tab",
            background=[("selected", self.panel), ("active", "#E7EDF4")],
            foreground=[("selected", self.accent_dark)],
        )

    # -------------------------------------------------------------------------
    # UI construction
    # -------------------------------------------------------------------------

    def _build_ui(self):
        root = tk.Frame(self, bg=self.bg)
        root.pack(fill="both", expand=True)

        # Header
        header = tk.Frame(root, bg=self.header)
        header.pack(fill="x")

        left = tk.Frame(header, bg=self.header)
        left.pack(side="left", padx=16, pady=10)

        tk.Label(
            left,
            text="SRNE INVERTER + 4× PING1D INTEGRATED STATION",
            bg=self.header,
            fg="white",
            font=("Segoe UI Semibold", 16 if self.compact else 20),
        ).pack(anchor="w")

        tk.Label(
            left,
            text="RS232 / Modbus RTU FC03  ·  4 TCP Ping1D  ·  MQTT Telemetry",
            bg=self.header,
            fg="#BCD0E2",
            font=("Segoe UI", 8 if self.compact else 9),
        ).pack(anchor="w", pady=(2, 0))

        self.mqtt_badge = tk.Label(
            header,
            text="MQTT CONNECTING",
            bg="#344F69",
            fg="#E7EFF6",
            font=("Segoe UI Semibold", 8 if self.compact else 9),
            padx=12,
            pady=6,
        )
        self.mqtt_badge.pack(side="right", padx=16, pady=12)

        # Connection panel
        self._build_connection_panel(root)

        # Tabs
        self.notebook = ttk.Notebook(root, style="Modern.TNotebook")
        self.notebook.pack(fill="both", expand=True, padx=8, pady=(0, 6))

        self.dashboard_tab = tk.Frame(self.notebook, bg=self.bg)
        self.ping_tabs = [
            tk.Frame(self.notebook, bg=self.bg) for _ in range(4)
        ]
        self.comm_tab = tk.Frame(self.notebook, bg=self.bg)

        self.notebook.add(self.dashboard_tab, text="Electrical Dashboard")
        for i, tab in enumerate(self.ping_tabs):
            self.notebook.add(tab, text=f"Ping1D {i+1}")
        self.notebook.add(self.comm_tab, text="Communication")

        self._build_dashboard()
        for i in range(4):
            self._build_ping_tab(i)
        self._build_comm_tab()

        footer = tk.Frame(root, bg=self.bg)
        footer.pack(fill="x", padx=10, pady=(0, 6))

        self.footer_var = tk.StringVar(value="Ready")
        tk.Label(
            footer,
            textvariable=self.footer_var,
            bg=self.bg,
            fg=self.muted,
            font=("Segoe UI", 8 if self.compact else 9),
        ).pack(side="left")

        self.mqtt_tx_var = tk.StringVar(value="MQTT TX: --")
        tk.Label(
            footer,
            textvariable=self.mqtt_tx_var,
            bg=self.bg,
            fg=self.accent_dark,
            font=("Segoe UI Semibold", 8 if self.compact else 9),
        ).pack(side="right")

    def _build_connection_panel(self, parent):
        panel = tk.Frame(
            parent,
            bg=self.panel,
            highlightthickness=1,
            highlightbackground=self.border,
        )
        panel.pack(fill="x", padx=8, pady=8)

        # Inverter / serial row
        row1 = tk.Frame(panel, bg=self.panel)
        row1.pack(fill="x", padx=10, pady=(8, 4))

        tk.Label(
            row1, text="INVERTER",
            bg=self.panel, fg=self.text,
            font=("Segoe UI Semibold", 9),
            width=11, anchor="w",
        ).pack(side="left")

        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(
            row1, textvariable=self.port_var,
            state="readonly", width=12,
        )
        self.port_combo.pack(side="left", padx=(0, 5))

        ttk.Button(
            row1, text="Refresh", command=self.refresh_ports
        ).pack(side="left", padx=(0, 8))

        tk.Label(row1, text="Baud", bg=self.panel, fg=self.muted).pack(side="left")
        self.baud_var = tk.StringVar(value="9600")
        ttk.Combobox(
            row1,
            textvariable=self.baud_var,
            state="readonly",
            width=8,
            values=("1200", "2400", "4800", "9600", "19200", "38400"),
        ).pack(side="left", padx=(4, 8))

        tk.Label(row1, text="Slave", bg=self.panel, fg=self.muted).pack(side="left")
        self.slave_var = tk.StringVar(value="1")
        ttk.Spinbox(
            row1, from_=1, to=247,
            textvariable=self.slave_var, width=6,
        ).pack(side="left", padx=(4, 10))

        self.serial_btn = ttk.Button(
            row1,
            text="Connect Inverter",
            command=self.toggle_serial,
            style="Primary.TButton",
        )
        self.serial_btn.pack(side="left")

        self.serial_status = tk.Label(
            row1,
            text="DISCONNECTED",
            bg="#E9EEF4",
            fg="#536479",
            font=("Segoe UI Semibold", 8),
            padx=8, pady=4,
        )
        self.serial_status.pack(side="left", padx=(8, 0))

        self.monitor_btn = ttk.Button(
            row1,
            text="Start Monitoring",
            command=self.toggle_monitoring,
            style="Primary.TButton",
        )
        self.monitor_btn.pack(side="right")

        self.poll_var = tk.StringVar(value="1.0")
        ttk.Combobox(
            row1,
            textvariable=self.poll_var,
            state="readonly",
            width=5,
            values=("0.5", "1.0", "2.0", "5.0"),
        ).pack(side="right", padx=(4, 4))
        tk.Label(
            row1, text="Poll s",
            bg=self.panel, fg=self.muted,
        ).pack(side="right")

        # Ping connection rows
        self.ping_ip_vars = []
        self.ping_port_vars = []
        self.ping_connect_buttons = []
        self.ping_status_labels = []

        for i, (name, ip, port) in enumerate(DEFAULT_PING_ENDPOINTS):
            row = tk.Frame(panel, bg=self.panel)
            row.pack(fill="x", padx=10, pady=2)

            tk.Label(
                row, text=name.upper(),
                bg=self.panel, fg=self.text,
                font=("Segoe UI Semibold", 8),
                width=11, anchor="w",
            ).pack(side="left")

            ip_var = tk.StringVar(value=ip)
            port_var = tk.StringVar(value=str(port))
            self.ping_ip_vars.append(ip_var)
            self.ping_port_vars.append(port_var)

            ttk.Entry(
                row, textvariable=ip_var, width=16
            ).pack(side="left", padx=(0, 5))

            ttk.Entry(
                row, textvariable=port_var, width=7
            ).pack(side="left", padx=(0, 6))

            btn = ttk.Button(
                row,
                text="Connect",
                command=lambda idx=i: self.toggle_ping(idx),
            )
            btn.pack(side="left")
            self.ping_connect_buttons.append(btn)

            status = tk.Label(
                row,
                text="DISCONNECTED",
                bg="#E9EEF4",
                fg="#536479",
                font=("Segoe UI Semibold", 8),
                padx=8, pady=3,
            )
            status.pack(side="left", padx=(8, 0))
            self.ping_status_labels.append(status)

        actions = tk.Frame(panel, bg=self.panel)
        actions.pack(fill="x", padx=10, pady=(4, 8))

        ttk.Button(
            actions,
            text="Connect All Ping1D",
            command=self.connect_all_ping,
        ).pack(side="right", padx=(5, 0))

        ttk.Button(
            actions,
            text="Disconnect All Ping1D",
            command=self.disconnect_all_ping,
        ).pack(side="right")

    def _build_dashboard(self):
        self.dashboard_tab.grid_columnconfigure((0, 1, 2), weight=1)
        self.dashboard_tab.grid_rowconfigure((0, 1), weight=1)

        cards = [
            ("ac_output_voltage", "AC OUTPUT VOLTAGE", "V"),
            ("ac_output_current", "AC OUTPUT CURRENT", "A"),
            ("ac_output_active_power", "AC OUTPUT ACTIVE POWER", "W"),
            ("battery_voltage", "BATTERY VOLTAGE", "V"),
            ("battery_power", "BATTERY POWER", "W"),
        ]

        self.electrical_vars = {}

        for idx, (key, title, unit) in enumerate(cards):
            r, c = divmod(idx, 3)
            frame = tk.Frame(
                self.dashboard_tab,
                bg=self.panel,
                highlightthickness=1,
                highlightbackground=self.border,
            )
            frame.grid(
                row=r, column=c,
                sticky="nsew", padx=6, pady=6,
            )

            tk.Label(
                frame, text=title,
                bg=self.panel, fg=self.muted,
                font=("Segoe UI Semibold", 8 if self.compact else 9),
            ).pack(anchor="w", padx=12, pady=(10, 2))

            var = tk.StringVar(value="--")
            self.electrical_vars[key] = var

            value_row = tk.Frame(frame, bg=self.panel)
            value_row.pack(fill="x", padx=12, pady=(2, 10))

            tk.Label(
                value_row, textvariable=var,
                bg=self.panel, fg=self.accent_dark,
                font=("Segoe UI Semibold", 23 if self.compact else 30),
            ).pack(side="left")

            tk.Label(
                value_row, text=unit,
                bg=self.panel, fg=self.muted,
                font=("Segoe UI", 10 if self.compact else 12),
            ).pack(side="left", padx=(5, 0), pady=(8, 0))

        # Battery direction card occupies last cell.
        frame = tk.Frame(
            self.dashboard_tab,
            bg=self.header2,
            highlightthickness=1,
            highlightbackground=self.header2,
        )
        frame.grid(row=1, column=2, sticky="nsew", padx=6, pady=6)

        tk.Label(
            frame,
            text="BATTERY ENERGY DIRECTION",
            bg=self.header2, fg="#BCD0E2",
            font=("Segoe UI Semibold", 8 if self.compact else 9),
        ).pack(anchor="w", padx=12, pady=(10, 2))

        self.direction_var = tk.StringVar(value="--")
        tk.Label(
            frame,
            textvariable=self.direction_var,
            bg=self.header2, fg="white",
            font=("Segoe UI Semibold", 18 if self.compact else 24),
        ).pack(anchor="w", padx=12, pady=(4, 12))

        status = tk.Frame(
            self.dashboard_tab,
            bg=self.panel,
            highlightthickness=1,
            highlightbackground=self.border,
        )
        status.grid(row=2, column=0, columnspan=3, sticky="ew", padx=6, pady=6)

        self.live_status_var = tk.StringVar(value="Waiting for inverter data")
        self.ping_summary_var = tk.StringVar(value="Ping1D: 0/4 streaming")

        tk.Label(
            status,
            textvariable=self.live_status_var,
            bg=self.panel, fg=self.accent_dark,
            font=("Segoe UI Semibold", 9),
        ).pack(side="left", padx=12, pady=8)

        tk.Label(
            status,
            textvariable=self.ping_summary_var,
            bg=self.panel, fg=self.muted,
            font=("Segoe UI", 9),
        ).pack(side="right", padx=12, pady=8)

    def _build_ping_tab(self, idx):
        tab = self.ping_tabs[idx]

        top = tk.Frame(
            tab, bg=self.panel,
            highlightthickness=1,
            highlightbackground=self.border,
        )
        top.pack(fill="x", padx=6, pady=6)

        title_var = tk.StringVar(value=f"PING1D {idx+1}")
        self.ping_tab_title_vars = getattr(self, "ping_tab_title_vars", [])
        self.ping_tab_title_vars.append(title_var)

        tk.Label(
            top,
            textvariable=title_var,
            bg=self.panel, fg=self.text,
            font=("Segoe UI Semibold", 11),
        ).pack(side="left", padx=12, pady=8)

        self._ensure_ping_ui_arrays()

        self.ping_tab_status_vars[idx] = tk.StringVar(value="DISCONNECTED")
        tk.Label(
            top,
            textvariable=self.ping_tab_status_vars[idx],
            bg=self.panel, fg=self.accent_dark,
            font=("Segoe UI Semibold", 9),
        ).pack(side="right", padx=12, pady=8)

        metrics = tk.Frame(tab, bg=self.bg)
        metrics.pack(fill="x", padx=2, pady=2)

        metric_defs = [
            ("Distance / Altimeter", "distance", "m"),
            ("Confidence", "confidence", "%"),
            ("Scan Start", "scan_start", "m"),
            ("Scan Length", "scan_length", "m"),
            ("Gain", "gain", ""),
        ]

        for col in range(5):
            metrics.grid_columnconfigure(col, weight=1, uniform=f"ping{idx}")

        for col, (title, key, unit) in enumerate(metric_defs):
            card = tk.Frame(
                metrics,
                bg=self.panel,
                highlightthickness=1,
                highlightbackground=self.border,
            )
            card.grid(row=0, column=col, sticky="nsew", padx=4, pady=4)

            tk.Label(
                card, text=title.upper(),
                bg=self.panel, fg=self.muted,
                font=("Segoe UI Semibold", 7 if self.compact else 8),
            ).pack(anchor="w", padx=9, pady=(8, 2))

            var = tk.StringVar(value="--")
            self.ping_metric_vars[idx][key] = var

            row = tk.Frame(card, bg=self.panel)
            row.pack(fill="x", padx=9, pady=(1, 8))

            tk.Label(
                row,
                textvariable=var,
                bg=self.panel, fg=self.accent_dark,
                font=("Segoe UI Semibold", 17 if self.compact else 22),
            ).pack(side="left")

            tk.Label(
                row, text=unit,
                bg=self.panel, fg=self.muted,
                font=("Segoe UI", 8 if self.compact else 9),
            ).pack(side="left", padx=(4, 0), pady=(6, 0))

        # Simple live echo profile
        profile_panel = tk.Frame(
            tab,
            bg=self.panel,
            highlightthickness=1,
            highlightbackground=self.border,
        )
        profile_panel.pack(fill="both", expand=True, padx=6, pady=6)

        tk.Label(
            profile_panel,
            text="LIVE ECHO PROFILE",
            bg=self.panel, fg=self.text,
            font=("Segoe UI Semibold", 9),
        ).pack(anchor="w", padx=10, pady=(8, 2))

        canvas = tk.Canvas(
            profile_panel,
            bg="#0F1722",
            highlightthickness=0,
            height=280,
        )
        canvas.pack(fill="both", expand=True, padx=10, pady=(2, 10))
        self.ping_profile_canvases[idx] = canvas

    def _ensure_ping_ui_arrays(self):
        if hasattr(self, "ping_metric_vars"):
            return

        self.ping_metric_vars = [
            {"distance": None, "confidence": None, "scan_start": None,
             "scan_length": None, "gain": None}
            for _ in range(4)
        ]
        self.ping_tab_status_vars = [None] * 4
        self.ping_profile_canvases = [None] * 4

    def _build_comm_tab(self):
        toolbar = tk.Frame(self.comm_tab, bg=self.bg)
        toolbar.pack(fill="x", padx=6, pady=(6, 2))

        ttk.Button(
            toolbar,
            text="Clear Log",
            command=lambda: self.log_text.delete("1.0", "end"),
        ).pack(side="left")

        self.log_text = tk.Text(
            self.comm_tab,
            bg="#0E1722",
            fg="#C9D8E6",
            insertbackground="white",
            font=("Consolas", 9),
            wrap="none",
            relief="flat",
        )
        self.log_text.pack(fill="both", expand=True, padx=6, pady=(2, 6))

    # -------------------------------------------------------------------------
    # Serial / Modbus
    # -------------------------------------------------------------------------

    def refresh_ports(self):
        values = [p.device for p in list_ports.comports()]
        self.port_combo["values"] = values
        if values and self.port_var.get() not in values:
            self.port_var.set(values[0])

    def toggle_serial(self):
        if self.serial_client.is_open:
            self.stop_monitoring()
            self.serial_client.disconnect()
            self.serial_btn.configure(text="Connect Inverter")
            self._set_badge(
                self.serial_status, "DISCONNECTED",
                "#E9EEF4", "#536479"
            )
            self.log("Serial inverter disconnected.")
            return

        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("COM Port", "Select COM port first.")
            return

        try:
            baud = int(self.baud_var.get())
            self.serial_client.connect(port, baud)
            self.serial_btn.configure(text="Disconnect Inverter")
            self._set_badge(
                self.serial_status, "CONNECTED",
                "#DDF6EB", "#137A55"
            )
            self.log(f"Serial connected: {port} @ {baud} baud, 8N1.")
        except Exception as exc:
            messagebox.showerror("Serial Connection", str(exc))
            self.log(f"SERIAL ERROR: {exc}")

    def toggle_monitoring(self):
        if self.modbus_polling:
            self.stop_monitoring()
        else:
            self.start_monitoring()

    def start_monitoring(self):
        if not self.serial_client.is_open:
            messagebox.showwarning(
                "Inverter",
                "Connect inverter COM first.",
            )
            return

        try:
            interval = float(self.poll_var.get())
            slave = int(self.slave_var.get())
        except Exception as exc:
            messagebox.showerror("Monitoring", str(exc))
            return

        self.modbus_stop_event.clear()
        self.modbus_polling = True
        self.monitor_btn.configure(text="Stop Monitoring")

        self.modbus_thread = threading.Thread(
            target=self._modbus_worker,
            args=(slave, interval),
            daemon=True,
        )
        self.modbus_thread.start()

    def stop_monitoring(self):
        self.modbus_stop_event.set()
        self.modbus_polling = False
        if hasattr(self, "monitor_btn"):
            self.monitor_btn.configure(text="Start Monitoring")

    def _modbus_worker(self, slave, interval):
        start_addr = min(v["addr"] for v in REGISTER_MAP.values())
        end_addr = max(v["addr"] for v in REGISTER_MAP.values())
        count = end_addr - start_addr + 1

        while (
            not self.modbus_stop_event.is_set()
            and self.serial_client.is_open
        ):
            t0 = time.time()

            try:
                values, tx, rx = self.serial_client.read_registers(
                    slave, start_addr, count
                )
                reg_map = {
                    start_addr + i: val
                    for i, val in enumerate(values)
                }
                self.event_queue.put(
                    ("modbus_data", reg_map, tx, rx)
                )

            except Exception as exc:
                self.event_queue.put(
                    ("modbus_error", str(exc))
                )

            elapsed = time.time() - t0
            self.modbus_stop_event.wait(
                max(0.05, interval - elapsed)
            )

        self.modbus_polling = False
        self.event_queue.put(("modbus_stopped",))

    def _engineering(self, reg_map, key):
        info = REGISTER_MAP[key]
        raw = reg_map.get(info["addr"])
        if raw is None:
            return None
        val = to_signed16(raw) if info["signed"] else raw
        return val * info["scale"]

    def _handle_modbus_data(self, reg_map, tx, rx):
        ac_v = self._engineering(reg_map, "ac_output_voltage")
        ac_i = self._engineering(reg_map, "ac_output_current")
        ac_p = self._engineering(reg_map, "ac_output_active_power")
        bat_v = self._engineering(reg_map, "battery_voltage")
        bat_i = self._engineering(reg_map, "battery_current_internal")
        bat_p = self._engineering(reg_map, "battery_power")

        if bat_i is None:
            direction = "UNKNOWN"
        elif bat_i > 0.05:
            direction = "CHARGING"
        elif bat_i < -0.05:
            direction = "DISCHARGING"
        else:
            direction = "IDLE"

        self.latest_electrical = {
            "ac_output_voltage": ac_v,
            "ac_output_current": ac_i,
            "ac_output_active_power": ac_p,
            "battery_voltage": bat_v,
            "battery_power": bat_p,
            "battery_energy_direction": direction,
        }
        self.latest_modbus_timestamp = time.time()

        fmt = {
            "ac_output_voltage": (ac_v, 1),
            "ac_output_current": (ac_i, 1),
            "ac_output_active_power": (ac_p, 0),
            "battery_voltage": (bat_v, 1),
            "battery_power": (bat_p, 0),
        }
        for key, (val, decimals) in fmt.items():
            if val is None:
                self.electrical_vars[key].set("--")
            else:
                self.electrical_vars[key].set(
                    f"{val:.{decimals}f}"
                )

        self.direction_var.set(direction)
        self.live_status_var.set(
            "LIVE OK · " + time.strftime("%H:%M:%S")
        )

        self.log(
            f"MODBUS RX OK · AC={ac_v:.1f}V {ac_i:.1f}A "
            f"{ac_p:.0f}W · BAT={bat_v:.1f}V {bat_p:.0f}W · {direction}"
        )

    # -------------------------------------------------------------------------
    # Ping1D connection management
    # -------------------------------------------------------------------------

    def toggle_ping(self, idx):
        client = self.ping_clients[idx]
        if client and client.is_alive():
            self.disconnect_ping(idx)
        else:
            self.connect_ping(idx)

    def connect_ping(self, idx):
        self.disconnect_ping(idx)

        host = self.ping_ip_vars[idx].get().strip()
        try:
            port = int(self.ping_port_vars[idx].get())
        except Exception:
            messagebox.showerror(
                "Ping1D",
                f"Invalid TCP port for Ping1D {idx+1}.",
            )
            return

        if not host:
            messagebox.showwarning(
                "Ping1D",
                f"Enter IP for Ping1D {idx+1}.",
            )
            return

        name = f"PING1D {idx+1}"
        client = Ping1DClient(
            idx, name, host, port, self.event_queue
        )
        self.ping_clients[idx] = client
        self.ping_connect_buttons[idx].configure(text="Disconnect")
        client.start()
        self.log(f"{name} connecting to {host}:{port}")

    def disconnect_ping(self, idx):
        client = self.ping_clients[idx]
        if client:
            client.stop()
            if client.is_alive():
                client.join(timeout=1.5)
        self.ping_clients[idx] = None
        if hasattr(self, "ping_connect_buttons"):
            self.ping_connect_buttons[idx].configure(text="Connect")
        if hasattr(self, "ping_status_labels"):
            self._set_badge(
                self.ping_status_labels[idx],
                "DISCONNECTED",
                "#E9EEF4", "#536479",
            )
        if (
            hasattr(self, "ping_tab_status_vars")
            and self.ping_tab_status_vars[idx] is not None
        ):
            self.ping_tab_status_vars[idx].set("DISCONNECTED")

    def connect_all_ping(self):
        for i in range(4):
            client = self.ping_clients[i]
            if not client or not client.is_alive():
                self.connect_ping(i)

    def disconnect_all_ping(self):
        for i in range(4):
            self.disconnect_ping(i)

    def _handle_ping_status(self, idx, status, error):
        if status in ("STREAMING",):
            bg, fg = "#DDF6EB", "#137A55"
        elif status in ("CONNECTING", "WAITING DATA", "RECONNECTING", "NO DATA"):
            bg, fg = "#FFF1D6", "#9B6514"
        elif status == "ERROR":
            bg, fg = "#FDE7E5", "#A82A20"
        else:
            bg, fg = "#E9EEF4", "#536479"

        self._set_badge(
            self.ping_status_labels[idx],
            status,
            bg, fg,
        )

        if self.ping_tab_status_vars[idx] is not None:
            text = status
            if error:
                text += f" · {error}"
            self.ping_tab_status_vars[idx].set(text)

        if error:
            self.log(f"PING1D {idx+1} {status}: {error}")

    def _handle_ping_data(self, idx, d):
        vars_ = self.ping_metric_vars[idx]

        if "distance_m" in d:
            vars_["distance"].set(f"{d['distance_m']:.3f}")
            self.ping_history[idx].append(d["distance_m"])

        if "confidence" in d:
            vars_["confidence"].set(f"{d['confidence']:.0f}")

        if "scan_start_m" in d:
            vars_["scan_start"].set(f"{d['scan_start_m']:.3f}")

        if "scan_length_m" in d:
            vars_["scan_length"].set(f"{d['scan_length_m']:.3f}")

        if "gain" in d:
            vars_["gain"].set(str(d["gain"]))

    def _draw_profile(self, idx, d):
        canvas = self.ping_profile_canvases[idx]
        if canvas is None:
            return

        profile = d.get("profile") or []
        if len(profile) < 2:
            return

        canvas.update_idletasks()
        w = max(40, canvas.winfo_width())
        h = max(40, canvas.winfo_height())

        canvas.delete("all")

        # Grid
        for k in range(1, 5):
            x = int(w * k / 5)
            y = int(h * k / 5)
            canvas.create_line(
                x, 0, x, h,
                fill="#1E3144", width=1
            )
            canvas.create_line(
                0, y, w, y,
                fill="#1E3144", width=1
            )

        n = len(profile)
        points = []
        for i, amp in enumerate(profile):
            x = i * (w - 1) / max(1, n - 1)
            y = h - 8 - (float(amp) / 255.0) * (h - 18)
            points.extend((x, y))

        canvas.create_line(
            *points,
            fill="#4CC9F0",
            width=2,
            smooth=False,
        )

        start_m = d.get("scan_start_m", 0.0)
        length_m = d.get("scan_length_m", 0.0)
        dist_m = d.get("distance_m")

        if dist_m is not None and length_m and length_m > 0:
            frac = (dist_m - start_m) / length_m
            if 0 <= frac <= 1:
                x = frac * w
                canvas.create_line(
                    x, 0, x, h,
                    fill="#FFCC66",
                    width=2,
                )

        canvas.create_text(
            8, 8,
            anchor="nw",
            fill="#AFC2D5",
            font=("Segoe UI", 8),
            text=(
                f"{start_m:.2f}–{start_m + length_m:.2f} m"
                if length_m
                else "Profile"
            ),
        )

    # -------------------------------------------------------------------------
    # MQTT payload
    # -------------------------------------------------------------------------

    def _mqtt_tick(self):
        try:
            if self.mqtt.connected:
                payload = {
                    "schema": "rmi_srne_ping1d_v1",
                    "timestamp": int(time.time()),
                    "electrical": self._safe_electrical_payload(),
                    "ping1d": {},
                }

                for i in range(4):
                    client = self.ping_clients[i]
                    if client:
                        snap = client.snapshot()
                    else:
                        snap = {
                            "name": f"PING1D {i+1}",
                            "ip": self.ping_ip_vars[i].get().strip(),
                            "port": int(self.ping_port_vars[i].get() or 0),
                            "status": "DISCONNECTED",
                            "distance_m": None,
                            "confidence": None,
                            "scan_start_m": None,
                            "scan_length_m": None,
                            "gain": None,
                            "profile": [],
                            "profile_seq": 0,
                            "profile_timestamp": None,
                            "age_s": None,
                        }

                    payload["ping1d"][str(i + 1)] = snap

                _, size = self.mqtt.publish(payload)
                self.last_mqtt_payload_size = size
                self.mqtt_tx_var.set(
                    f"MQTT TX {size/1024:.1f} kB · "
                    + time.strftime("%H:%M:%S")
                )

        except Exception as exc:
            self.log(f"MQTT PUBLISH ERROR: {exc}")

        finally:
            try:
                self.after(
                    self.mqtt_publish_interval_ms,
                    self._mqtt_tick,
                )
            except tk.TclError:
                pass

    def _safe_electrical_payload(self):
        def clean(value, digits=2):
            if value is None:
                return None
            return round(float(value), digits)

        e = self.latest_electrical
        return {
            "ac_output_voltage": clean(e["ac_output_voltage"], 2),
            "ac_output_current": clean(e["ac_output_current"], 2),
            "ac_output_active_power": clean(e["ac_output_active_power"], 2),
            "battery_voltage": clean(e["battery_voltage"], 2),
            "battery_power": clean(e["battery_power"], 2),
            "battery_energy_direction": str(
                e["battery_energy_direction"]
            ),
        }

    # -------------------------------------------------------------------------
    # Periodic UI / events
    # -------------------------------------------------------------------------

    def _refresh_statuses(self):
        if self.mqtt.connected:
            self._set_badge(
                self.mqtt_badge,
                "MQTT CONNECTED",
                "#DDF6EB", "#137A55",
            )
        else:
            text = "MQTT CONNECTING"
            if self.mqtt.last_error:
                text = "MQTT ERROR"
            self._set_badge(
                self.mqtt_badge,
                text,
                "#FFF1D6", "#9B6514",
            )

        streaming = 0
        for i, client in enumerate(self.ping_clients):
            if not client:
                continue

            snap = client.snapshot()
            status = snap["status"]
            if status == "STREAMING":
                streaming += 1

            # Refresh stale state visually.
            if status == "NO DATA":
                self._handle_ping_status(i, "NO DATA", "")

        self.ping_summary_var.set(
            f"Ping1D: {streaming}/4 streaming"
        )

        try:
            self.after(500, self._refresh_statuses)
        except tk.TclError:
            pass

    def _process_events(self):
        try:
            while True:
                event = self.event_queue.get_nowait()
                kind = event[0]

                if kind == "modbus_data":
                    self._handle_modbus_data(
                        event[1], event[2], event[3]
                    )

                elif kind == "modbus_error":
                    self.live_status_var.set(
                        "LIVE ERROR · " + event[1]
                    )
                    self.log("MODBUS ERROR: " + event[1])

                elif kind == "modbus_stopped":
                    self.monitor_btn.configure(
                        text="Start Monitoring"
                    )

                elif kind == "ping_status":
                    self._handle_ping_status(
                        event[1], event[2], event[3]
                    )

                elif kind == "ping_data":
                    self._handle_ping_data(
                        event[1], event[2]
                    )

                elif kind == "ping_profile":
                    self._handle_ping_data(
                        event[1], event[2]
                    )
                    self._draw_profile(
                        event[1], event[2]
                    )

        except queue.Empty:
            pass

        try:
            self.after(80, self._process_events)
        except tk.TclError:
            pass

    @staticmethod
    def _set_badge(widget, text, bg, fg):
        widget.configure(
            text=text,
            bg=bg,
            fg=fg,
        )

    def log(self, message):
        if not hasattr(self, "log_text"):
            return
        stamp = time.strftime("%H:%M:%S")
        self.log_text.insert(
            "end",
            f"[{stamp}] {message}\n",
        )
        self.log_text.see("end")

    def on_close(self):
        self.stop_monitoring()
        self.disconnect_all_ping()
        self.serial_client.disconnect()

        try:
            self.mqtt.stop()
        except Exception:
            pass

        self.destroy()


if __name__ == "__main__":
    app = IntegratedApp()
    app.mainloop()
