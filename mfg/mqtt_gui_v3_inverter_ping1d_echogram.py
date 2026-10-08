#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
MQTT Inverter + 4x Ping1D Monitor with Echograms
Version: v3.0.0

Receives the combined payload published by:
    srne_integrated_inverter_controller_v1_7_0_ping1d_mqtt.py

Requirements:
    pip install paho-mqtt
"""

import json
import time
import tkinter as tk
from collections import deque
from tkinter import ttk, messagebox, scrolledtext

import paho.mqtt.client as mqtt


BROKER_HOST = "9bb0622065e44988ba6ba779032dd29d.s1.eu.hivemq.cloud"
BROKER_PORT = 8883
USERNAME = "ha_rmi"
PASSWORD = "Password#1"
TOPIC = "hydrophone/electrical/solarpanel"

APP_NAME = "Integrated Inverter + Ping1D MQTT Monitor"
APP_VERSION = "v3.0.0"

ECHOGRAM_WIDTH = 128
ECHOGRAM_ROWS = 140


def sonar_color(v):
    """
    Dark -> blue -> cyan -> yellow -> white.
    Input v: 0..255
    """
    v = max(0, min(255, int(v)))
    x = v / 255.0

    stops = [
        (0.00, (5, 8, 18)),
        (0.20, (20, 45, 100)),
        (0.50, (0, 160, 200)),
        (0.78, (255, 185, 60)),
        (1.00, (255, 255, 240)),
    ]

    for i in range(len(stops) - 1):
        p0, c0 = stops[i]
        p1, c1 = stops[i + 1]
        if p0 <= x <= p1:
            t = (x - p0) / max(1e-9, p1 - p0)
            r = int(c0[0] + (c1[0] - c0[0]) * t)
            g = int(c0[1] + (c1[1] - c0[1]) * t)
            b = int(c0[2] + (c1[2] - c0[2]) * t)
            return f"#{r:02x}{g:02x}{b:02x}"

    return "#ffffff"


COLOR_LUT = [sonar_color(i) for i in range(256)]


class EchogramWidget(tk.Frame):
    def __init__(self, parent, title):
        super().__init__(parent, bg="#101B28")

        self.title = title
        self.last_profile_seq = None
        self.rows = deque(maxlen=ECHOGRAM_ROWS)

        self.distance_var = tk.StringVar(value="--")
        self.confidence_var = tk.StringVar(value="--")
        self.status_var = tk.StringVar(value="DISCONNECTED")
        self.range_var = tk.StringVar(value="--")
        self.gain_var = tk.StringVar(value="--")

        self._build()

    def _build(self):
        header = tk.Frame(self, bg="#101B28")
        header.pack(fill="x", padx=10, pady=(8, 4))

        tk.Label(
            header,
            text=self.title,
            bg="#101B28",
            fg="#F3F8FF",
            font=("Segoe UI Semibold", 11),
        ).pack(side="left")

        tk.Label(
            header,
            textvariable=self.status_var,
            bg="#101B28",
            fg="#7EE787",
            font=("Segoe UI Semibold", 8),
        ).pack(side="right")

        metrics = tk.Frame(self, bg="#101B28")
        metrics.pack(fill="x", padx=10, pady=(0, 4))

        self._metric(metrics, "DISTANCE", self.distance_var, "m", 0)
        self._metric(metrics, "CONFIDENCE", self.confidence_var, "%", 1)
        self._metric(metrics, "RANGE", self.range_var, "m", 2)
        self._metric(metrics, "GAIN", self.gain_var, "", 3)

        for c in range(4):
            metrics.grid_columnconfigure(c, weight=1, uniform="m")

        plot_frame = tk.Frame(self, bg="#101B28")
        plot_frame.pack(fill="both", expand=True, padx=10, pady=(4, 10))
        plot_frame.grid_columnconfigure((0, 1), weight=1)
        plot_frame.grid_rowconfigure(0, weight=1)

        # Live echo profile
        left = tk.Frame(plot_frame, bg="#0F1722")
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 4))

        tk.Label(
            left,
            text="LIVE ECHO PROFILE",
            bg="#0F1722",
            fg="#CFE2F5",
            font=("Segoe UI Semibold", 8),
        ).pack(anchor="w", padx=8, pady=(7, 2))

        self.profile_canvas = tk.Canvas(
            left,
            bg="#0F1722",
            highlightthickness=0,
            height=250,
        )
        self.profile_canvas.pack(fill="both", expand=True, padx=5, pady=(0, 5))

        # Waterfall / echogram
        right = tk.Frame(plot_frame, bg="#0F1722")
        right.grid(row=0, column=1, sticky="nsew", padx=(4, 0))

        tk.Label(
            right,
            text="WATERFALL / ECHOGRAM",
            bg="#0F1722",
            fg="#CFE2F5",
            font=("Segoe UI Semibold", 8),
        ).pack(anchor="w", padx=8, pady=(7, 2))

        self.echo_canvas = tk.Canvas(
            right,
            bg="#050812",
            highlightthickness=0,
            height=250,
        )
        self.echo_canvas.pack(fill="both", expand=True, padx=5, pady=(0, 5))

        self.echo_photo = tk.PhotoImage(
            width=ECHOGRAM_WIDTH,
            height=ECHOGRAM_ROWS,
        )
        self.echo_image_item = self.echo_canvas.create_image(
            0, 0, anchor="nw", image=self.echo_photo
        )

        self.echo_canvas.bind("<Configure>", self._resize_echo_image)

    def _metric(self, parent, title, var, unit, col):
        card = tk.Frame(parent, bg="#0B1520")
        card.grid(row=0, column=col, sticky="nsew", padx=3)

        tk.Label(
            card,
            text=title,
            bg="#0B1520",
            fg="#7F94AA",
            font=("Segoe UI Semibold", 7),
        ).pack(anchor="w", padx=7, pady=(5, 0))

        row = tk.Frame(card, bg="#0B1520")
        row.pack(fill="x", padx=7, pady=(0, 5))

        tk.Label(
            row,
            textvariable=var,
            bg="#0B1520",
            fg="#F5F9FF",
            font=("Segoe UI Semibold", 14),
        ).pack(side="left")

        tk.Label(
            row,
            text=unit,
            bg="#0B1520",
            fg="#8EA3B7",
            font=("Segoe UI", 8),
        ).pack(side="left", padx=(3, 0), pady=(4, 0))

    def _resize_echo_image(self, event=None):
        # PhotoImage itself remains low-resolution for efficiency.
        # Canvas stretches by using a nearest-neighbor zoom only when practical.
        self._draw_echogram()

    def update_data(self, data):
        status = str(data.get("status") or "DISCONNECTED")
        self.status_var.set(status)

        distance = data.get("distance_m")
        confidence = data.get("confidence")
        start = data.get("scan_start_m")
        length = data.get("scan_length_m")
        gain = data.get("gain")

        self.distance_var.set("--" if distance is None else f"{float(distance):.3f}")
        self.confidence_var.set("--" if confidence is None else f"{float(confidence):.0f}")
        self.gain_var.set("--" if gain is None else str(gain))

        if start is not None and length is not None:
            self.range_var.set(f"{float(start):.1f}–{float(start)+float(length):.1f}")
        else:
            self.range_var.set("--")

        profile = data.get("profile") or []
        seq = data.get("profile_seq")

        if profile:
            self._draw_profile(
                profile,
                start,
                length,
                distance,
            )

            if seq != self.last_profile_seq:
                self.last_profile_seq = seq
                row = self._resample_profile(profile, ECHOGRAM_WIDTH)
                self.rows.append(row)
                self._draw_echogram()

    @staticmethod
    def _resample_profile(profile, width):
        n = len(profile)
        if n <= 0:
            return [0] * width

        out = []
        for i in range(width):
            idx = int(i * (n - 1) / max(1, width - 1))
            out.append(max(0, min(255, int(profile[idx]))))
        return out

    def _draw_profile(self, profile, start, length, distance):
        c = self.profile_canvas
        c.update_idletasks()
        w = max(80, c.winfo_width())
        h = max(60, c.winfo_height())

        c.delete("all")

        for k in range(1, 5):
            x = w * k / 5
            y = h * k / 5
            c.create_line(x, 0, x, h, fill="#1F3042")
            c.create_line(0, y, w, y, fill="#1F3042")

        points = []
        n = len(profile)
        for i, amp in enumerate(profile):
            x = i * (w - 1) / max(1, n - 1)
            y = h - 8 - (float(amp) / 255.0) * (h - 18)
            points.extend((x, y))

        c.create_line(
            *points,
            fill="#4CC9F0",
            width=2,
        )

        if (
            distance is not None
            and start is not None
            and length not in (None, 0)
        ):
            frac = (float(distance) - float(start)) / float(length)
            if 0 <= frac <= 1:
                x = frac * w
                c.create_line(
                    x, 0, x, h,
                    fill="#FFCC66",
                    width=2,
                )

    def _draw_echogram(self):
        if not self.rows:
            return

        rows = list(self.rows)
        blank_rows = ECHOGRAM_ROWS - len(rows)
        image_rows = [[0] * ECHOGRAM_WIDTH for _ in range(blank_rows)] + rows

        # Efficient row update using PhotoImage.put string rows.
        for y, row in enumerate(image_rows):
            colors = "{" + " ".join(COLOR_LUT[v] for v in row) + "}"
            self.echo_photo.put(colors, to=(0, y))

        c = self.echo_canvas
        c.update_idletasks()
        cw = max(1, c.winfo_width())
        ch = max(1, c.winfo_height())

        # Use integer zoom for visibility when space permits.
        zx = max(1, cw // ECHOGRAM_WIDTH)
        zy = max(1, ch // ECHOGRAM_ROWS)
        zoom = min(zx, zy, 4)

        if zoom > 1:
            img = self.echo_photo.zoom(zoom, zoom)
        else:
            img = self.echo_photo

        self._display_photo = img
        c.itemconfig(self.echo_image_item, image=img)
        c.coords(self.echo_image_item, 0, 0)


class MQTTIntegratedMonitor(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title(f"{APP_NAME} - {APP_VERSION}")

        self.update_idletasks()
        sw = max(900, self.winfo_screenwidth())
        sh = max(600, self.winfo_screenheight())
        compact = sw <= 1440 or sh <= 850

        w = min(1560, max(1000, int(sw * 0.97)))
        h = min(960, max(650, int(sh * 0.92)))
        self.geometry(
            f"{w}x{h}+{max(0,(sw-w)//2)}+{max(0,(sh-h)//3)}"
        )
        self.minsize(min(1000, w), min(650, h))

        self.bg = "#09111B"
        self.panel = "#101B28"
        self.border = "#1F3042"
        self.text = "#DCE7F3"
        self.muted = "#8194A8"
        self.accent = "#4CC9F0"

        self.configure(bg=self.bg)

        self.client = None
        self.is_connected = False

        self.electrical_vars = {
            "ac_output_voltage": tk.StringVar(value="--"),
            "ac_output_current": tk.StringVar(value="--"),
            "ac_output_active_power": tk.StringVar(value="--"),
            "battery_voltage": tk.StringVar(value="--"),
            "battery_power": tk.StringVar(value="--"),
            "battery_energy_direction": tk.StringVar(value="--"),
        }

        self.last_update_var = tk.StringVar(value="No telemetry received")
        self.rx_count = 0

        self._setup_style(compact)
        self._build_ui()

    def _setup_style(self, compact):
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except Exception:
            pass

        style.configure(
            "Dark.TNotebook",
            background=self.bg,
            borderwidth=0,
        )
        style.configure(
            "Dark.TNotebook.Tab",
            background="#0C1621",
            foreground="#8599AD",
            padding=(10, 6) if compact else (14, 8),
            font=("Segoe UI Semibold", 8 if compact else 9),
        )
        style.map(
            "Dark.TNotebook.Tab",
            background=[("selected", "#152437"), ("active", "#132031")],
            foreground=[("selected", "#EEF6FF")],
        )

    def _build_ui(self):
        header = tk.Frame(self, bg="#102A43")
        header.pack(fill="x")

        left = tk.Frame(header, bg="#102A43")
        left.pack(side="left", padx=18, pady=12)

        tk.Label(
            left,
            text="INVERTER + 4× PING1D MQTT MONITOR",
            bg="#102A43",
            fg="white",
            font=("Segoe UI Semibold", 18),
        ).pack(anchor="w")

        tk.Label(
            left,
            text=f"Topic: {TOPIC}",
            bg="#102A43",
            fg="#BCD0E2",
            font=("Segoe UI", 9),
        ).pack(anchor="w", pady=(2, 0))

        self.status_badge = tk.Label(
            header,
            text="DISCONNECTED",
            bg="#324D67",
            fg="#E7EEF5",
            font=("Segoe UI Semibold", 9),
            padx=12,
            pady=6,
        )
        self.status_badge.pack(side="right", padx=18, pady=14)

        toolbar = tk.Frame(
            self,
            bg=self.panel,
            highlightthickness=1,
            highlightbackground=self.border,
        )
        toolbar.pack(fill="x", padx=10, pady=(10, 6))

        self.connect_btn = tk.Button(
            toolbar,
            text="CONNECT MQTT",
            command=self.toggle_connection,
            bg="#1261A0",
            fg="white",
            activebackground="#1774BA",
            activeforeground="white",
            relief="flat",
            font=("Segoe UI Semibold", 9),
            padx=14,
            pady=7,
            cursor="hand2",
        )
        self.connect_btn.pack(side="left", padx=10, pady=9)

        tk.Label(
            toolbar,
            text=f"{BROKER_HOST}:{BROKER_PORT} · TLS · QoS 1",
            bg=self.panel,
            fg=self.muted,
            font=("Segoe UI", 9),
        ).pack(side="left", padx=8)

        tk.Label(
            toolbar,
            textvariable=self.last_update_var,
            bg=self.panel,
            fg=self.accent,
            font=("Segoe UI Semibold", 9),
        ).pack(side="right", padx=12)

        self.notebook = ttk.Notebook(
            self,
            style="Dark.TNotebook",
        )
        self.notebook.pack(
            fill="both",
            expand=True,
            padx=10,
            pady=(0, 8),
        )

        self.electrical_tab = tk.Frame(self.notebook, bg=self.bg)
        self.ping_tabs = [
            tk.Frame(self.notebook, bg=self.bg) for _ in range(4)
        ]
        self.log_tab = tk.Frame(self.notebook, bg=self.bg)

        self.notebook.add(self.electrical_tab, text="Electrical")
        for i, tab in enumerate(self.ping_tabs):
            self.notebook.add(tab, text=f"Ping1D {i+1}")
        self.notebook.add(self.log_tab, text="MQTT Log")

        self._build_electrical_tab()

        self.ping_widgets = []
        for i, tab in enumerate(self.ping_tabs):
            widget = EchogramWidget(tab, f"PING1D {i+1}")
            widget.pack(fill="both", expand=True, padx=6, pady=6)
            self.ping_widgets.append(widget)

        self._build_log_tab()

    def _build_electrical_tab(self):
        cards = tk.Frame(self.electrical_tab, bg=self.bg)
        cards.pack(fill="both", expand=True, padx=4, pady=4)

        keys = [
            ("ac_output_voltage", "AC OUTPUT VOLTAGE", "V"),
            ("ac_output_current", "AC OUTPUT CURRENT", "A"),
            ("ac_output_active_power", "AC OUTPUT ACTIVE POWER", "W"),
            ("battery_voltage", "BATTERY VOLTAGE", "V"),
            ("battery_power", "BATTERY POWER", "W"),
        ]

        for c in range(3):
            cards.grid_columnconfigure(c, weight=1, uniform="e")
        for r in range(2):
            cards.grid_rowconfigure(r, weight=1, uniform="e")

        for idx, (key, title, unit) in enumerate(keys):
            r, c = divmod(idx, 3)
            frame = tk.Frame(
                cards,
                bg=self.panel,
                highlightthickness=1,
                highlightbackground=self.border,
            )
            frame.grid(
                row=r, column=c,
                sticky="nsew",
                padx=6, pady=6,
            )

            tk.Label(
                frame,
                text=title,
                bg=self.panel,
                fg="#7F94AA",
                font=("Segoe UI Semibold", 8),
            ).pack(anchor="w", padx=14, pady=(14, 3))

            row = tk.Frame(frame, bg=self.panel)
            row.pack(fill="x", padx=14, pady=(3, 14))

            tk.Label(
                row,
                textvariable=self.electrical_vars[key],
                bg=self.panel,
                fg="#F5F9FF",
                font=("Segoe UI Semibold", 30),
            ).pack(side="left")

            tk.Label(
                row,
                text=unit,
                bg=self.panel,
                fg="#8EA3B7",
                font=("Segoe UI", 11),
            ).pack(side="left", padx=(5, 0), pady=(10, 0))

        direction = tk.Frame(
            cards,
            bg="#173F61",
            highlightthickness=1,
            highlightbackground="#173F61",
        )
        direction.grid(
            row=1, column=2,
            sticky="nsew",
            padx=6, pady=6,
        )

        tk.Label(
            direction,
            text="BATTERY ENERGY DIRECTION",
            bg="#173F61",
            fg="#BCD0E2",
            font=("Segoe UI Semibold", 8),
        ).pack(anchor="w", padx=14, pady=(14, 3))

        self.direction_label = tk.Label(
            direction,
            textvariable=self.electrical_vars["battery_energy_direction"],
            bg="#173F61",
            fg="white",
            font=("Segoe UI Semibold", 24),
        )
        self.direction_label.pack(anchor="w", padx=14, pady=(5, 14))

    def _build_log_tab(self):
        top = tk.Frame(self.log_tab, bg=self.bg)
        top.pack(fill="x", padx=6, pady=(6, 2))

        tk.Button(
            top,
            text="CLEAR LOG",
            command=lambda: self.log_area.delete("1.0", "end"),
            bg="#172638",
            fg=self.text,
            relief="flat",
            padx=10,
            pady=5,
        ).pack(side="right")

        self.log_area = scrolledtext.ScrolledText(
            self.log_tab,
            bg="#0B131D",
            fg="#BCD0E3",
            insertbackground="white",
            font=("Consolas", 9),
            wrap="word",
            relief="flat",
        )
        self.log_area.pack(fill="both", expand=True, padx=6, pady=(2, 6))

    def log(self, text):
        stamp = time.strftime("%H:%M:%S")
        self.after(0, self._log_ui, f"[{stamp}] {text}")

    def _log_ui(self, text):
        self.log_area.insert("end", text + "\n")
        self.log_area.see("end")

    def toggle_connection(self):
        if self.is_connected:
            try:
                self.client.disconnect()
                self.client.loop_stop()
            except Exception:
                pass
            return

        try:
            self.client = mqtt.Client(
                client_id="rmi_integrated_monitor",
                callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            )
            self.client.username_pw_set(USERNAME, PASSWORD)
            self.client.tls_set()
            self.client.on_connect = self.on_connect
            self.client.on_disconnect = self.on_disconnect
            self.client.on_message = self.on_message

            self.client.connect_async(
                BROKER_HOST,
                BROKER_PORT,
                60,
            )
            self.client.loop_start()

            self.log(
                f"Connecting to {BROKER_HOST}:{BROKER_PORT}..."
            )

        except Exception as exc:
            messagebox.showerror("MQTT Connection", str(exc))

    def on_connect(self, client, userdata, flags, reason_code, properties=None):
        try:
            failed = bool(reason_code.is_failure)
        except Exception:
            failed = reason_code != 0

        if not failed:
            self.is_connected = True
            client.subscribe(TOPIC, qos=1)

            self.after(
                0,
                lambda: self.status_badge.config(
                    text="CONNECTED",
                    bg="#DDF6EB",
                    fg="#137A55",
                )
            )
            self.after(
                0,
                lambda: self.connect_btn.config(
                    text="DISCONNECT",
                    bg="#8B1E2D",
                )
            )
            self.log(f"Subscribed to {TOPIC}")
        else:
            self.log(f"MQTT connection failed: {reason_code}")

    def on_disconnect(
        self,
        client,
        userdata,
        disconnect_flags,
        reason_code,
        properties=None,
    ):
        self.is_connected = False
        self.after(
            0,
            lambda: self.status_badge.config(
                text="DISCONNECTED",
                bg="#324D67",
                fg="#E7EEF5",
            )
        )
        self.after(
            0,
            lambda: self.connect_btn.config(
                text="CONNECT MQTT",
                bg="#1261A0",
            )
        )
        self.log(f"MQTT disconnected: {reason_code}")

    def on_message(self, client, userdata, msg):
        try:
            text = msg.payload.decode("utf-8", errors="replace")
            data = json.loads(text)
        except Exception as exc:
            self.log(f"Payload decode error: {exc}")
            return

        if data.get("schema") != "rmi_srne_ping1d_v1":
            self.log("Ignored payload with unsupported schema.")
            return

        self.rx_count += 1
        self.after(0, self._update_ui, data)

    def _update_ui(self, data):
        electrical = data.get("electrical") or {}

        def set_num(key, decimals):
            val = electrical.get(key)
            self.electrical_vars[key].set(
                "--" if val is None else f"{float(val):.{decimals}f}"
            )

        set_num("ac_output_voltage", 1)
        set_num("ac_output_current", 1)
        set_num("ac_output_active_power", 0)
        set_num("battery_voltage", 1)
        set_num("battery_power", 0)

        direction = str(
            electrical.get("battery_energy_direction") or "UNKNOWN"
        ).upper()
        self.electrical_vars["battery_energy_direction"].set(direction)

        if direction == "CHARGING":
            self.direction_label.config(fg="#86EFAC")
        elif direction == "DISCHARGING":
            self.direction_label.config(fg="#FDE68A")
        elif direction == "IDLE":
            self.direction_label.config(fg="#E7EEF5")
        else:
            self.direction_label.config(fg="#CBD5E1")

        ping = data.get("ping1d") or {}
        for i in range(4):
            d = ping.get(str(i + 1)) or {}
            self.ping_widgets[i].update_data(d)

        ts = data.get("timestamp")
        if isinstance(ts, (int, float)):
            stamp = time.strftime("%H:%M:%S", time.localtime(ts))
        else:
            stamp = time.strftime("%H:%M:%S")

        self.last_update_var.set(
            f"Last telemetry {stamp} · RX {self.rx_count}"
        )

        # Compact log line rather than printing large profile arrays.
        statuses = [
            str((ping.get(str(i+1)) or {}).get("status", "-"))
            for i in range(4)
        ]
        self.log(
            "RX telemetry · "
            f"AC={electrical.get('ac_output_voltage')}V "
            f"{electrical.get('ac_output_active_power')}W · "
            f"BAT={electrical.get('battery_voltage')}V "
            f"{electrical.get('battery_power')}W · "
            f"PING={','.join(statuses)}"
        )

    def on_close(self):
        try:
            if self.client is not None:
                self.client.disconnect()
                self.client.loop_stop()
        except Exception:
            pass
        self.destroy()


if __name__ == "__main__":
    app = MQTTIntegratedMonitor()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()
