"""
Automatic Server-Room Temperature Control  -  SIMULATION
=========================================================
Embedded-systems course project (pure-Python simulation, standard library only).

WHAT IS BEING SIMULATED
-----------------------------------------------------------------------------
  * ESP32  (SLAVE)        : reads a temperature/humidity sensor and drives the
                            cooling-fan motor (PWM duty cycle).
  * Raspberry Pi (MASTER) : runs the control rules, the operator display
                            (this GUI) and the audio-visual alarm.
  * Wireless link         : modelled by `WirelessChannel`, a pair of message
                            queues with a small transport latency.

  The Pi is the bus MASTER and polls the ESP32 SLAVE every cycle:

        Pi   --  READ_SENSOR   -->   ESP32     (master requests a reading)
        Pi   <-- SENSOR_DATA   --    ESP32     (slave answers temp + humidity)
        Pi   --  SET_FAN(spd)  -->   ESP32     (master commands the fan speed)
        Pi   <-- FAN_FEEDBACK  --    ESP32     (slave confirms duty + rpm)

CONTROL RULE  (fan speed vs. temperature)
-----------------------------------------------------------------------------
  * 16 C  -> 0 %   (ideal temperature, fans off)
  * +2 C  -> +5 % of the maximum motor speed
        24 C -> 20 %      |      50 C -> 85 %
  * Maximum allowable temperature = 60 C.  Above it the operator gets:
        - a warning message on the display,
        - a red warning light,
        - a siren sound through the speaker.

RUN:   python server_room_control.py
       (sound uses winsound on Windows; on Linux/Mac it falls back to the
        terminal bell, while the visual alarm always works.)
"""

import tkinter as tk
import threading
import queue
import time
import math
import random

# ----- optional real sound on Windows ---------------------------------------
try:
    import winsound
    _HAS_WINSOUND = True
except Exception:
    _HAS_WINSOUND = False


# =============================================================================
# 1. SYSTEM CONSTANTS  (the "predefined rules" living on the Raspberry Pi)
# =============================================================================
IDEAL_TEMP   = 16.0      # C   -> fans off
TEMP_STEP    = 2.0       # C   per speed step
SPEED_STEP   = 5.0       # %   added per TEMP_STEP
MAX_TEMP     = 60.0      # C   maximum allowable; alarm above this value
FAN_MAX_RPM  = 3000      # rpm at 100 %

POLL_PERIOD  = 0.4       # s   master polling period
LINK_LATENCY = 0.04      # s   simulated wireless transport delay


def compute_fan_speed(temp_c):
    """Map a temperature to a fan-speed percentage using the project rule.
    16 C -> 0 %, then +5 % for every +2 C, clamped to the 0..100 % range."""
    if temp_c <= IDEAL_TEMP:
        return 0.0
    pct = ((temp_c - IDEAL_TEMP) / TEMP_STEP) * SPEED_STEP
    return max(0.0, min(100.0, pct))


# =============================================================================
# 2. WIRELESS MASTER-SLAVE CHANNEL
# =============================================================================
class Message:
    __slots__ = ("kind", "data")

    def __init__(self, kind, data=None):
        self.kind = kind
        self.data = data or {}

    def __repr__(self):
        return f"{self.kind}{self.data if self.data else ''}"


class WirelessChannel:
    """Two one-way links with a small latency, simulating the RF transport.
    `link_down` lets the operator inject a communication fault on purpose."""

    def __init__(self, latency=LINK_LATENCY):
        self.downlink = queue.Queue()    # master (Pi)  -> slave (ESP32)
        self.uplink   = queue.Queue()    # slave (ESP32) -> master (Pi)
        self.latency  = latency
        self.link_down = False

    def _send_later(self, q, msg):
        # deliver after `latency` seconds without blocking the caller
        threading.Timer(self.latency, q.put, args=(msg,)).start()

    # ---- master (Raspberry Pi) side -----------------------------------------
    def master_send(self, msg):
        if self.link_down:
            return                       # packet lost -> master will time out
        self._send_later(self.downlink, msg)

    def master_recv(self, timeout=None):
        return self.uplink.get(timeout=timeout)

    # ---- slave (ESP32) side -------------------------------------------------
    def slave_send(self, msg):
        if self.link_down:
            return
        self._send_later(self.uplink, msg)

    def slave_get_nowait(self):
        return self.downlink.get_nowait()


# =============================================================================
# 3. PHYSICAL MODEL OF THE ROOM + ESP32 (SLAVE DEVICE)
# =============================================================================
class ServerRoom:
    """A tiny thermal model so that cooling actually 'works'.
    `base_temp` is what the operator dials in (server heat level); running
    the fans pulls the measured temperature down by up to a few degrees."""

    def __init__(self):
        self.temp = 22.0
        self.base_temp = 22.0
        self.add_noise = True

    def update(self, dt, fan_fraction):
        cooling = 6.0 * fan_fraction              # up to -6 C at full speed
        target = self.base_temp - cooling
        self.temp += (target - self.temp) * min(1.0, dt * 0.8)   # 1st-order lag


class ESP32:
    """The slave micro-controller: owns the sensor and the fan motor and
    answers the master's requests."""

    def __init__(self, room):
        self.room = room
        self.fan_pct = 0.0
        self.fan_rpm = 0.0

    def read_sensor(self):
        t = self.room.temp
        h = 45 + (t - 22) * 0.4                    # humidity loosely tied to temp
        if self.room.add_noise:
            t += random.uniform(-0.3, 0.3)
            h += random.uniform(-1.0, 1.0)
        return round(t, 1), round(max(0.0, min(100.0, h)), 1)

    def drive_fan(self, pct):
        self.fan_pct = max(0.0, min(100.0, pct))
        self.fan_rpm = FAN_MAX_RPM * self.fan_pct / 100.0
        if self.fan_pct > 0:
            self.fan_rpm *= random.uniform(0.98, 1.02)   # small mechanical ripple
        return round(self.fan_pct, 1), int(round(self.fan_rpm))

    @property
    def fan_fraction(self):
        return self.fan_pct / 100.0


# =============================================================================
# 4. SIMULATION BACKEND  (ESP32 thread + Raspberry-Pi thread)
# =============================================================================
class Simulation:
    """Runs the ESP32 slave loop and the Raspberry-Pi master loop in
    background threads and publishes a state snapshot for the GUI."""

    def __init__(self):
        self.room = ServerRoom()
        self.esp32 = ESP32(self.room)
        self.channel = WirelessChannel()
        self.state_q = queue.Queue()      # snapshots  -> GUI
        self.log_q   = queue.Queue()      # comm log    -> GUI
        self._run = threading.Event()
        self._threads = []

    # ---- public control -----------------------------------------------------
    def start(self):
        self._run.set()
        self._threads = [
            threading.Thread(target=self._esp32_loop, daemon=True),
            threading.Thread(target=self._pi_loop,    daemon=True),
        ]
        for t in self._threads:
            t.start()

    def stop(self):
        self._run.clear()

    def log(self, who, msg):
        self.log_q.put(f"{time.strftime('%H:%M:%S')}  {who:<6}{msg}")

    # ---- ESP32 (slave) thread ----------------------------------------------
    def _esp32_loop(self):
        last = time.time()
        while self._run.is_set():
            now = time.time()
            dt, last = now - last, now
            self.room.update(dt, self.esp32.fan_fraction)     # physics tick
            try:                                              # serve requests
                while True:
                    self._esp32_handle(self.channel.slave_get_nowait())
            except queue.Empty:
                pass
            time.sleep(0.02)

    def _esp32_handle(self, req):
        if req.kind == "READ_SENSOR":
            t, h = self.esp32.read_sensor()
            self.channel.slave_send(Message("SENSOR_DATA",
                                            {"temp": t, "humidity": h}))
        elif req.kind == "SET_FAN":
            pct, rpm = self.esp32.drive_fan(req.data["speed"])
            self.channel.slave_send(Message("FAN_FEEDBACK",
                                            {"pct": pct, "rpm": rpm}))

    # ---- Raspberry Pi (master) thread --------------------------------------
    def _pi_loop(self):
        while self._run.is_set():
            link_ok = True
            speed = 0.0
            alarm = False
            fb = {"pct": 0, "rpm": 0}
            temp = hum = 0.0
            try:
                # 1) request a sensor reading
                self.channel.master_send(Message("READ_SENSOR"))
                self.log("Pi", "->  READ_SENSOR")
                data = self.channel.master_recv(timeout=1.0).data
                temp, hum = data["temp"], data["humidity"]
                self.log("ESP32", f"<-  SENSOR_DATA   {temp} C / {hum}% RH")

                # 2) apply the predefined rules
                speed = compute_fan_speed(temp)
                alarm = temp > MAX_TEMP

                # 3) command the fan
                self.channel.master_send(Message("SET_FAN", {"speed": speed}))
                self.log("Pi", f"->  SET_FAN  {speed:.0f}%")
                fb = self.channel.master_recv(timeout=1.0).data
                self.log("ESP32",
                         f"<-  FAN_FEEDBACK  {fb['pct']:.0f}% / {fb['rpm']} rpm")

            except queue.Empty:
                link_ok = False
                self.log("Pi", "!!  LINK TIMEOUT - no answer from ESP32")

            # 4) publish a snapshot for the operator display
            self.state_q.put({
                "temp": temp, "humidity": hum,
                "speed_cmd": speed, "fan_pct": fb["pct"], "fan_rpm": fb["rpm"],
                "alarm": alarm, "link_ok": link_ok,
            })
            time.sleep(POLL_PERIOD)


# =============================================================================
# 5. SIREN  (audio part of the alarm)
# =============================================================================
class Siren:
    def __init__(self):
        self._on = False
        self._thread = None

    def start(self):
        if self._on:
            return
        self._on = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._on = False

    def _run(self):
        while self._on:
            if _HAS_WINSOUND:
                for f in (880, 1180):
                    if not self._on:
                        break
                    winsound.Beep(f, 250)
            else:
                print("\a", end="", flush=True)   # terminal bell fallback
                time.sleep(0.5)


# =============================================================================
# 6. OPERATOR DASHBOARD  (the Raspberry-Pi display, with GUI)
# =============================================================================
BG     = "#0f1419"
PANEL  = "#161d29"
PANEL2 = "#1d2735"
ACCENT = "#3ba6ff"
GREEN  = "#37d67a"
ORANGE = "#ffae42"
RED    = "#ff5252"
TEXT   = "#e6edf3"
MUTED  = "#8b98a5"
TRACK  = "#26313f"


class Dashboard:
    def __init__(self, root):
        self.root = root
        self.sim = Simulation()
        self.siren = Siren()

        self.state = None
        self.fan_angle = 0.0
        self.blink = False
        self._last_blink = 0.0

        self.noise   = tk.BooleanVar(value=True)
        self.muted   = tk.BooleanVar(value=False)
        self.linkcut = tk.BooleanVar(value=False)

        root.title("Automatic Server-Room Temperature Control  -  Simulation")
        root.configure(bg=BG)
        root.geometry("1180x760")
        root.minsize(1040, 700)

        self._build()
        self.sim.start()
        self._tick()
        root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------- small UI helpers --------------------------------------------
    def _card(self, parent, title):
        outer = tk.Frame(parent, bg=PANEL, highlightbackground="#243044",
                         highlightthickness=1)
        tk.Label(outer, text=title, bg=PANEL, fg=ACCENT,
                 font=("Segoe UI Semibold", 12)).pack(anchor="w",
                                                       padx=14, pady=(10, 4))
        return outer

    def _row(self, parent, name):
        f = tk.Frame(parent, bg=PANEL)
        f.pack(fill="x", padx=14, pady=3)
        tk.Label(f, text=name, bg=PANEL, fg=MUTED,
                 font=("Segoe UI", 11)).pack(side="left")
        v = tk.Label(f, text="--", bg=PANEL, fg=TEXT,
                     font=("Segoe UI Semibold", 12))
        v.pack(side="right")
        return v

    # ---------- build the whole layout --------------------------------------
    def _build(self):
        # header ---------------------------------------------------------------
        header = tk.Frame(self.root, bg=BG)
        header.pack(fill="x", padx=16, pady=(14, 6))
        tk.Label(header, text="Automatic Server-Room Temperature Control",
                 bg=BG, fg=TEXT, font=("Segoe UI Semibold", 20)).pack(side="left")
        tk.Label(header, text="ESP32 (slave)  -  wireless  -  Raspberry Pi (master)",
                 bg=BG, fg=MUTED, font=("Segoe UI", 11)).pack(side="right", pady=8)

        # body : three columns -------------------------------------------------
        body = tk.Frame(self.root, bg=BG)
        body.pack(fill="both", expand=True, padx=16, pady=6)
        body.columnconfigure(0, weight=3)
        body.columnconfigure(1, weight=3)
        body.columnconfigure(2, weight=4)
        body.rowconfigure(0, weight=1)

        self._build_esp_card(body)
        self._build_gauge_card(body)
        self._build_pi_card(body)

        # controls bar ---------------------------------------------------------
        self._build_controls()

    def _build_esp_card(self, body):
        card = self._card(body, "ESP32  -  Slave Device")
        card.grid(row=0, column=0, sticky="nsew", padx=(0, 10))

        self.fan_canvas = tk.Canvas(card, width=170, height=170, bg=PANEL,
                                    highlightthickness=0)
        self.fan_canvas.pack(pady=(6, 4))

        self.esp_fan  = self._row(card, "Fan duty cycle")
        self.esp_rpm  = self._row(card, "Fan speed")
        self.esp_temp = self._row(card, "Sensor temperature")
        self.esp_hum  = self._row(card, "Sensor humidity")

        link = tk.Frame(card, bg=PANEL)
        link.pack(fill="x", padx=14, pady=(10, 12))
        self.link_dot = tk.Canvas(link, width=16, height=16, bg=PANEL,
                                  highlightthickness=0)
        self.link_dot.pack(side="left")
        self.link_lbl = tk.Label(link, text="wireless link", bg=PANEL, fg=MUTED,
                                 font=("Segoe UI", 11))
        self.link_lbl.pack(side="left", padx=8)

    def _build_gauge_card(self, body):
        card = self._card(body, "System Gauges")
        card.grid(row=0, column=1, sticky="nsew", padx=10)

        self.temp_gauge = tk.Canvas(card, width=230, height=210, bg=PANEL,
                                    highlightthickness=0)
        self.temp_gauge.pack(pady=(4, 0))
        self.fan_gauge = tk.Canvas(card, width=230, height=210, bg=PANEL,
                                   highlightthickness=0)
        self.fan_gauge.pack()

        rule = ("Rule:  16 C -> 0 %   |   +5 % per +2 C   |   max 60 C\n"
                "examples:   24 C -> 20 %      50 C -> 85 %")
        tk.Label(card, text=rule, bg=PANEL, fg=MUTED, justify="center",
                 font=("Segoe UI", 9)).pack(pady=(2, 12))

    def _build_pi_card(self, body):
        card = self._card(body, "Raspberry Pi  -  Operator Console")
        card.grid(row=0, column=2, sticky="nsew", padx=(10, 0))

        # warning banner
        self.banner = tk.Label(card, text="  SYSTEM NORMAL  ", bg="#143a24",
                               fg=GREEN, font=("Segoe UI Semibold", 13))
        self.banner.pack(fill="x", padx=14, pady=(2, 8))

        mid = tk.Frame(card, bg=PANEL)
        mid.pack(fill="x", padx=14)

        left = tk.Frame(mid, bg=PANEL)
        left.pack(side="left", anchor="n")
        self.pi_temp = tk.Label(left, text="--", bg=PANEL, fg=TEXT,
                                font=("Segoe UI Semibold", 40))
        self.pi_temp.pack(anchor="w")
        self.status = tk.Label(left, text="STARTING...", bg=PANEL, fg=MUTED,
                               font=("Segoe UI Semibold", 14))
        self.status.pack(anchor="w")

        lampf = tk.Frame(left, bg=PANEL)
        lampf.pack(anchor="w", pady=8)
        self.lamp = tk.Canvas(lampf, width=46, height=46, bg=PANEL,
                              highlightthickness=0)
        self.lamp.pack(side="left")
        tk.Label(lampf, text="warning light", bg=PANEL, fg=MUTED,
                 font=("Segoe UI", 10)).pack(side="left", padx=8)

        right = tk.Frame(mid, bg=PANEL)
        right.pack(side="right", anchor="n")
        self.pi_hum = self._row(right, "Humidity")
        self.pi_cmd = self._row(right, "Commanded speed")
        self.pi_fan = self._row(right, "Actual fan duty")
        self.pi_rpm = self._row(right, "Fan rpm")

        tk.Label(card, text="Master <-> Slave communication", bg=PANEL, fg=ACCENT,
                 font=("Segoe UI Semibold", 11)).pack(anchor="w",
                                                       padx=14, pady=(12, 2))
        logf = tk.Frame(card, bg=PANEL)
        logf.pack(fill="both", expand=True, padx=14, pady=(0, 12))
        self.log = tk.Text(logf, bg="#0c1118", fg="#9fd0ff", height=8,
                           font=("Consolas", 9), bd=0, highlightthickness=0,
                           wrap="none", state="disabled")
        self.log.pack(side="left", fill="both", expand=True)
        sb = tk.Scrollbar(logf, command=self.log.yview)
        sb.pack(side="right", fill="y")
        self.log.config(yscrollcommand=sb.set)

    def _build_controls(self):
        bar = tk.Frame(self.root, bg=PANEL2)
        bar.pack(fill="x", padx=16, pady=(6, 14))

        tk.Label(bar, text="Server heat level  (sets room temperature)",
                 bg=PANEL2, fg=TEXT, font=("Segoe UI", 11)).pack(side="left",
                                                                 padx=(14, 8),
                                                                 pady=12)
        self.heat = tk.Scale(bar, from_=10, to=75, orient="horizontal",
                             length=320, resolution=0.5, command=self._on_heat,
                             bg=PANEL2, fg=TEXT, troughcolor="#0c1118",
                             highlightthickness=0, bd=0,
                             activebackground=ACCENT, font=("Segoe UI", 9))
        self.heat.set(self.sim.room.base_temp)
        self.heat.pack(side="left", pady=8)

        def chk(text, var, cmd):
            return tk.Checkbutton(bar, text=text, variable=var, command=cmd,
                                  bg=PANEL2, fg=TEXT, selectcolor="#0c1118",
                                  activebackground=PANEL2, activeforeground=TEXT,
                                  font=("Segoe UI", 10), bd=0,
                                  highlightthickness=0)

        chk("Sensor noise", self.noise,
            lambda: setattr(self.sim.room, "add_noise",
                            self.noise.get())).pack(side="left", padx=10)
        chk("Mute siren", self.muted, lambda: None).pack(side="left", padx=10)
        chk("Cut wireless link", self.linkcut,
            lambda: setattr(self.sim.channel, "link_down",
                            self.linkcut.get())).pack(side="left", padx=10)

        tk.Label(bar, text=f"max allowable: {MAX_TEMP:.0f} C", bg=PANEL2,
                 fg=ORANGE, font=("Segoe UI", 11)).pack(side="right", padx=14)

    # ---------- drawing primitives ------------------------------------------
    def _temp_color(self, t):
        if t > MAX_TEMP:
            return RED
        if t > 40:
            return ORANGE
        return GREEN

    def _draw_gauge(self, canvas, value, vmin, vmax, unit, label, color):
        canvas.delete("all")
        cx, cy, r = 115, 118, 80
        bb = (cx - r, cy - r, cx + r, cy + r)
        canvas.create_arc(*bb, start=225, extent=-270, style="arc",
                          width=16, outline=TRACK)
        frac = 0 if vmax == vmin else max(0.0, min(1.0,
                                                   (value - vmin) / (vmax - vmin)))
        if frac > 0:
            canvas.create_arc(*bb, start=225, extent=-270 * frac, style="arc",
                              width=16, outline=color)
        canvas.create_text(cx, cy - 6, text=f"{value:.1f}", fill=TEXT,
                           font=("Segoe UI Semibold", 30))
        canvas.create_text(cx, cy + 22, text=unit, fill=MUTED,
                           font=("Segoe UI", 12))
        canvas.create_text(cx, cy + r + 6, text=label, fill=MUTED,
                           font=("Segoe UI", 11))

    def _draw_fan(self, angle, pct):
        c = self.fan_canvas
        c.delete("all")
        cx, cy, R = 85, 85, 72
        c.create_oval(cx - R - 6, cy - R - 6, cx + R + 6, cy + R + 6,
                      outline="#2c3a4d", width=3)
        color = ACCENT if pct > 0 else "#3a4654"
        local = [(0.15, -0.10), (0.85, -0.22), (0.98, 0.04), (0.22, 0.13)]
        for i in range(4):
            a = math.radians(angle + i * 90)
            pts = []
            for lx, ly in local:
                rx = lx * math.cos(a) - ly * math.sin(a)
                ry = lx * math.sin(a) + ly * math.cos(a)
                pts += [cx + rx * R, cy + ry * R]
            c.create_polygon(pts, fill=color, outline="", smooth=True)
        c.create_oval(cx - 12, cy - 12, cx + 12, cy + 12, fill=BG,
                      outline=color, width=3)

    def _light(self, color):
        self.lamp.delete("all")
        self.lamp.create_oval(5, 5, 41, 41, fill=color, outline="#0a0d12",
                              width=2)

    def _set_link(self, ok):
        self.link_dot.delete("all")
        self.link_dot.create_oval(2, 2, 14, 14,
                                  fill=GREEN if ok else RED, outline="")
        self.link_lbl.config(text="wireless link OK" if ok
                             else "wireless link LOST",
                             fg=GREEN if ok else RED)

    def _append_log(self, lines):
        self.log.config(state="normal")
        for ln in lines:
            self.log.insert("end", ln + "\n")
        # keep only the last ~200 lines
        total = int(self.log.index("end-1c").split(".")[0])
        if total > 200:
            self.log.delete("1.0", f"{total - 200}.0")
        self.log.see("end")
        self.log.config(state="disabled")

    # ---------- main animation / refresh loop -------------------------------
    def _tick(self):
        try:
            while True:
                self.state = self.sim.state_q.get_nowait()
        except queue.Empty:
            pass

        lines = []
        try:
            while True:
                lines.append(self.sim.log_q.get_nowait())
        except queue.Empty:
            pass
        if lines:
            self._append_log(lines)

        now = time.time()
        if now - self._last_blink > 0.4:
            self.blink = not self.blink
            self._last_blink = now

        s = self.state
        if s:
            temp, hum = s["temp"], s["humidity"]
            cmd, fan, rpm = s["speed_cmd"], s["fan_pct"], s["fan_rpm"]
            alarm, link = s["alarm"], s["link_ok"]

            self._draw_gauge(self.temp_gauge, temp, 0, 80, "C",
                             "ROOM TEMPERATURE", self._temp_color(temp))
            self._draw_gauge(self.fan_gauge, fan, 0, 100, "%",
                             "FAN SPEED", ACCENT if fan > 0 else TRACK)

            self.esp_temp.config(text=f"{temp:.1f} C")
            self.esp_hum.config(text=f"{hum:.1f} %")
            self.esp_fan.config(text=f"{fan:.0f} %")
            self.esp_rpm.config(text=f"{rpm} rpm")

            self.pi_temp.config(text=f"{temp:.1f}\u00b0", fg=self._temp_color(temp))
            self.pi_hum.config(text=f"{hum:.1f} %")
            self.pi_cmd.config(text=f"{cmd:.0f} %")
            self.pi_fan.config(text=f"{fan:.0f} %")
            self.pi_rpm.config(text=f"{rpm} rpm")

            self._set_link(link)
            self._update_alarm(alarm, temp, fan, link)

            self.fan_angle = (self.fan_angle + 0.6 + fan * 0.12) % 360
            self._draw_fan(self.fan_angle, fan)

        self.root.after(33, self._tick)

    def _update_alarm(self, alarm, temp, fan, link):
        if alarm:
            on = self.blink
            self.banner.config(
                text="  /!\\  OVER-TEMPERATURE ALARM - CHECK COOLING SYSTEM  ",
                bg=RED if on else "#7a1414", fg="white")
            self._light(RED if on else "#3a0d0d")
            self.status.config(text="ALARM", fg=RED)
            if self.muted.get():
                self.siren.stop()
            else:
                self.siren.start()
        else:
            self.siren.stop()
            self._light("#143a1f")
            if not link:
                self.status.config(text="LINK FAULT", fg=ORANGE)
                self.banner.config(text="  COMMUNICATION FAULT WITH ESP32  ",
                                   bg="#5a3a00", fg=ORANGE)
            elif fan <= 0:
                self.status.config(text="IDLE - FANS OFF", fg=GREEN)
                self.banner.config(text="  SYSTEM NORMAL  ",
                                   bg="#143a24", fg=GREEN)
            else:
                self.status.config(text="COOLING", fg=ACCENT)
                self.banner.config(text="  SYSTEM NORMAL - COOLING ACTIVE  ",
                                   bg="#143a24", fg=GREEN)

    # ---------- shutdown -----------------------------------------------------
    def _on_heat(self, val):
        self.sim.room.base_temp = float(val)

    def _on_close(self):
        self.siren.stop()
        self.sim.stop()
        self.root.destroy()


# =============================================================================
# 7. ENTRY POINT
# =============================================================================
def main():
    root = tk.Tk()
    Dashboard(root)
    root.mainloop()


if __name__ == "__main__":
    main()
