"""
field_gamemaster.py -- scrimmage field control, GAMEMASTER laptop

A stand-in for the VEX field controller / Tournament Manager for practice
matches. This window owns the clock and decides when each alliance's
robots may move; the alliance laptops run tools/field_relay.py, which
forwards that to every controller plugged into them (USB -> controller ->
radio -> brain). On the brain, src/field_control.cpp obeys it.

    gamemaster laptop   python tools/field_gamemaster.py
         |  same Wi-Fi / LAN (found automatically, or relay --host <ip>)
    red laptop          python tools/field_relay.py --alliance red
    blue laptop         python tools/field_relay.py --alliance blue

PERIODS
    AUTONOMOUS           0:30  robots run runAutonomous(); joysticks dead.
                               When it ends the window moves on to DRIVER
                               CONTROL, ready for the next START.
    DRIVER CONTROL       1:30  joystick driving
    DRIVER SKILLS        1:00  joystick driving
    PROGRAMMING SKILLS   1:00  robots run runAutonomous()

CONTROLS
    START  (Space)       counts down "3, 2, 1, GO!" (with beeps, on every
                         laptop; robots stay disabled), then on GO enables
                         every alliance ticked "in this run" and starts the
                         clock. At 0:00 everyone is disabled.
    STOP   (Esc)         disables everyone right now and freezes the clock
                         (also cancels a countdown).
    RESET                STOP + clock back to the period's full time.
    ENABLE / DISABLE     per alliance, any time -- e.g. a referee disable
                         mid-match, or free driving without the clock.
    RELEASE ROBOTS       hands the robots back to normal practice (driver
                         menu stays closed until the program restarts).

Robots that have never heard from the gamemaster behave normally. Once a
robot has, it stays DISABLED unless this window enables it, and it also
disables itself if it hears nothing for 2 s (laptop closed, cable pulled,
radio dropout).

NEEDS
    Python 3 with tkinter (standard on Windows). Allow Python through the
    Windows firewall when asked (private networks), or the relays can't
    connect.
"""

import argparse
import json
import math
import os
import queue
import random
import socket
import sys
import threading
import time
import tkinter as tk

try:
    import winsound
except ImportError:   # not Windows: fall back to the Tk bell
    winsound = None

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))   # so field_web is found

WEB_PORT = 8080         # phone / tablet page (tools/field_web.py)
TCP_PORT = 5800        # relays connect here
DISCOVERY_PORT = 5801    # UDP broadcast "FCGM <tcp port>" so relays find us
SEND_INTERVAL_S = 0.2    # state pushed to the relays this often
STATUS_STALE_S = 2.0     # a robot / relay silent this long is shown as lost
COUNTDOWN_S = 3          # "3, 2, 1, GO!" before every START (robots stay disabled)
GO_SHOW_S = 1.0          # how long "GO!" stays up once the period starts

ALLIANCES = ("red", "blue")

# key, title, robot period (A = autonomous, D = driver), seconds,
# period selected automatically once this one runs out
PERIODS = [
    ("AUTO", "AUTONOMOUS", "A", 30, "DRIVER"),
    ("DRIVER", "DRIVER CONTROL", "D", 90, None),
    ("DSKILLS", "DRIVER SKILLS", "D", 60, None),
    ("PSKILLS", "PROGRAMMING SKILLS", "A", 60, None),
]
PERIOD_BY_KEY = {p[0]: p for p in PERIODS}

BG = "#14141a"
CARD = "#161B22"
CARD_SEL = "#22303c"
CYAN = "#00F0FF"
ORANGE = "#FF8C00"
GREEN = "#2ECC71"
RED = "#FF3131"
BLUE = "#3B8BFF"
GRAY = "#777777"
DIM = "#3a3a46"
WHITE = "#e6e6e6"
ALLIANCE_COLORS = {"red": RED, "blue": BLUE}


def describe_robot(fcs):
    """'A|1|0|3' (see field_control.cpp) -> short text for the robot list."""
    parts = (fcs or "").split("|")
    if len(parts) < 3:
        return "?"
    if parts[0] == "P":
        return "program running - waiting for field control"
    period = "AUTO" if parts[0] == "A" else "DRIVER"
    if parts[1] != "1":
        return f"{period} - disabled"
    return f"{period} - ENABLED" + (" (auton running)" if parts[2] == "1" else "")


# kind -> (Hz, ms): countdown tick, the start of a period, the end of one
BEEPS = {"count": (880, 150), "go": (1320, 600), "end": (440, 900)}


SOUNDS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sounds")


def custom_sound(kind):
    """tools/sounds/<kind>.wav or .mp3 (kind: count, go, end), or None."""
    for ext in (".wav", ".mp3"):
        path = os.path.join(SOUNDS_DIR, kind + ext)
        if os.path.isfile(path):
            return path
    return None


def play_file(path):
    """Plays a .wav / .mp3 through Windows' built-in MCI player, on its own thread."""
    def run():
        try:
            import ctypes
            mci = ctypes.windll.winmm.mciSendStringW
            alias = "fcsound%d" % (threading.get_ident() % 100000)
            if mci(f'open "{path}" type mpegvideo alias {alias}', None, 0, None) == 0:
                mci(f"play {alias} wait", None, 0, None)
                mci(f"close {alias}", None, 0, None)
        except (OSError, AttributeError):
            pass
    threading.Thread(target=run, daemon=True).start()


def beep(root, kind, custom=False):
    """Plays one of BEEPS without stalling the window (winsound.Beep blocks,
    so it gets its own thread). With custom=True, a file in tools/sounds/
    named after the kind (e.g. go.mp3) plays instead of the beep."""
    if custom:
        path = custom_sound(kind)
        if path:
            play_file(path)
            return
    if winsound is None:
        root.bell()
        return
    hz, ms = BEEPS[kind]
    threading.Thread(target=winsound.Beep, args=(hz, ms), daemon=True).start()


def countdown_text(state):
    """What the big clock shows instead of the time during the countdown /
    right after GO, or None. Shared with the relay window."""
    if state.get("countdown"):
        return str(state["countdown"]), ORANGE
    if state.get("go"):
        return "GO!", GREEN
    return None


def local_ips():
    ips = set()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ips.add(info[4][0])
    except OSError:
        pass
    try:  # the address the default route uses -- usually the useful one
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ips.add(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    return sorted(ip for ip in ips if not ip.startswith("127."))


class RelayServer:
    """TCP server the relays connect to. Pushes the latest state to every
    relay a few times a second and keeps each relay's last status report.
    All socket work happens on background threads; the GUI only calls
    set_state() and relays()."""

    def __init__(self, port):
        self.port = port
        self.lock = threading.Lock()
        self.state = None
        self.clients = {}   # socket -> {"addr", "alliance", "name", "robots", "seen"}
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("", port))
        self.sock.listen()
        for target in (self._acceptLoop, self._sendLoop, self._announceLoop):
            threading.Thread(target=target, daemon=True).start()

    def set_state(self, state):
        with self.lock:
            self.state = state

    def relays(self):
        with self.lock:
            return [dict(c) for c in self.clients.values()]

    def _acceptLoop(self):
        while True:
            conn, addr = self.sock.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self.lock:
                self.clients[conn] = {"addr": addr[0], "alliance": "?", "name": addr[0],
                                      "robots": [], "seen": time.time()}
            threading.Thread(target=self._readLoop, args=(conn,), daemon=True).start()

    def _readLoop(self, conn):
        try:
            for line in conn.makefile("r", encoding="utf-8", errors="ignore"):
                try:
                    msg = json.loads(line)
                except ValueError:
                    continue
                with self.lock:
                    c = self.clients.get(conn)
                    if c is None:
                        break
                    c["seen"] = time.time()
                    c["alliance"] = msg.get("alliance", c["alliance"])
                    c["name"] = msg.get("name", c["name"])
                    c["robots"] = msg.get("robots", c["robots"])
        except OSError:
            pass
        self._drop(conn)

    def _drop(self, conn):
        with self.lock:
            self.clients.pop(conn, None)
        try:
            conn.close()
        except OSError:
            pass

    def _sendLoop(self):
        while True:
            with self.lock:
                state = self.state
                conns = list(self.clients)
            if state is not None:
                data = (json.dumps(state) + "\n").encode()
                for conn in conns:
                    try:
                        conn.sendall(data)
                    except OSError:
                        self._drop(conn)
            time.sleep(SEND_INTERVAL_S)

    def _announceLoop(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        msg = f"FCGM {self.port}".encode()
        while True:
            try:
                s.sendto(msg, ("255.255.255.255", DISCOVERY_PORT))
            except OSError:
                pass
            time.sleep(1.0)


class GamemasterApp:
    def __init__(self, root, server):
        self.root = root
        self.server = server
        self.periodKey = "AUTO"
        self.secsLeft = float(PERIOD_BY_KEY["AUTO"][3])
        self.running = False
        self.countdownEnd = None   # time.time() the countdown reaches GO, while counting
        self.lastCount = None      # last number shown, to beep once per number
        self.goUntil = 0.0
        self.released = False
        self.enabled = {a: False for a in ALLIANCES}
        # Seeded from the clock so a restarted gamemaster never reuses an id
        # a robot already ran its autonomous for.
        self.runId = {a: int(time.time()) % 1000000 for a in ALLIANCES}
        self.inRun = {a: tk.BooleanVar(value=True) for a in ALLIANCES}
        self.lastTick = time.time()
        self.cmdQueue = queue.Queue()   # commands from the phone page, run on the Tk thread
        self.webState = {}              # latest snapshot the page polls

        root.title("Field Control - Gamemaster")
        root.configure(bg=BG)
        root.geometry("980x640")
        root.minsize(820, 560)

        # Period picker
        bar = tk.Frame(root, bg=BG)
        bar.pack(fill="x", padx=16, pady=(14, 4))
        self.periodButtons = {}
        for key, title, _, secs, _ in PERIODS:
            b = tk.Button(bar, text=f"{title}\n{secs // 60}:{secs % 60:02d}",
                          font=("Segoe UI", 11, "bold"), relief="flat", bd=0, padx=10, pady=8,
                          activebackground=CARD_SEL, activeforeground=WHITE, takefocus=0,
                          command=lambda k=key: self.selectPeriod(k))
            b.pack(side="left", expand=True, fill="x", padx=4)
            self.periodButtons[key] = b

        # Clock
        self.periodLabel = tk.Label(root, bg=BG, fg=GRAY, font=("Segoe UI", 18, "bold"))
        self.periodLabel.pack(pady=(10, 0))
        self.clockLabel = tk.Label(root, bg=BG, fg=WHITE, font=("Consolas", 110, "bold"))
        self.clockLabel.pack()

        # Start / stop / reset
        ctl = tk.Frame(root, bg=BG)
        ctl.pack(pady=(0, 10))
        self.startButton = self._button(ctl, "START  [Space]", GREEN, self.start)
        self._button(ctl, "STOP  [Esc]", RED, self.stop)
        self._button(ctl, "RESET", DIM, self.reset)

        # Alliance cards
        cards = tk.Frame(root, bg=BG)
        cards.pack(fill="both", expand=True, padx=16)
        self.allianceWidgets = {}
        for a in ALLIANCES:
            col = ALLIANCE_COLORS[a]
            card = tk.Frame(cards, bg=CARD, highlightthickness=3, highlightbackground=col)
            card.pack(side="left", fill="both", expand=True, padx=6, pady=4)
            head = tk.Frame(card, bg=CARD)
            head.pack(fill="x", padx=12, pady=(10, 4))
            tk.Label(head, text=f"{a.upper()} ALLIANCE", bg=CARD, fg=col,
                     font=("Segoe UI", 16, "bold")).pack(side="left")
            tk.Checkbutton(head, text="in this run", variable=self.inRun[a], bg=CARD, fg=WHITE,
                           selectcolor=BG, activebackground=CARD, activeforeground=WHITE,
                           takefocus=0, font=("Segoe UI", 10)).pack(side="right")
            toggle = tk.Button(card, font=("Segoe UI", 16, "bold"), relief="flat", bd=0, pady=10,
                               takefocus=0, command=lambda al=a: self.toggleAlliance(al))
            toggle.pack(fill="x", padx=12, pady=4)
            robots = tk.Label(card, bg=CARD, fg=WHITE, font=("Consolas", 11), justify="left",
                              anchor="nw")
            robots.pack(fill="both", expand=True, padx=12, pady=(4, 10))
            self.allianceWidgets[a] = (toggle, robots)

        # Footer
        foot = tk.Frame(root, bg=BG)
        foot.pack(fill="x", padx=16, pady=(4, 12))
        ips = ", ".join(local_ips()) or "?"
        tk.Label(foot, text=f"gamemaster at {ips}  (port {server.port})", bg=BG, fg=GRAY,
                 font=("Segoe UI", 10)).pack(side="left")
        self.releaseButton = tk.Button(foot, font=("Segoe UI", 10, "bold"), relief="flat", bd=0,
                                       padx=10, pady=4, takefocus=0, command=self.toggleRelease)
        self.releaseButton.pack(side="right")
        self.webLabel = tk.Label(root, bg=BG, fg=CYAN, font=("Segoe UI", 10, "bold"))
        self.webLabel.pack(fill="x", padx=16, pady=(0, 10))

        root.bind("<space>", lambda e: self.stop() if self.busy() else self.start())
        root.bind("<Escape>", lambda e: self.stop())

        self.refresh()
        self.tick()

    def _button(self, parent, text, color, cmd):
        fg = WHITE if color == DIM else "#0b0b0f"
        b = tk.Button(parent, text=text, bg=color, fg=fg, activebackground=color,
                      activeforeground=fg, font=("Segoe UI", 14, "bold"), relief="flat", bd=0,
                      padx=24, pady=8, takefocus=0, command=cmd)
        b.pack(side="left", padx=6)
        return b

    # ─── Actions ─────────────────────────────────────────────────────────
    def period(self):
        return PERIOD_BY_KEY[self.periodKey]

    def selectPeriod(self, key):
        self.stop()
        self.periodKey = key
        self.secsLeft = float(self.period()[3])
        self.refresh()

    def busy(self):
        """Counting down or running a period."""
        return self.running or self.countdownEnd is not None

    def start(self):
        """START: "3, 2, 1" with everyone disabled, then go() on GO."""
        if self.busy() or not any(self.inRun[a].get() for a in ALLIANCES):
            return
        if self.secsLeft <= 0:
            self.secsLeft = float(self.period()[3])
        self.released = False   # robots held DISABLED through the countdown
        for a in ALLIANCES:
            self.enabled[a] = False
        self.countdownEnd = time.time() + COUNTDOWN_S
        self.lastCount = None
        self.refresh()

    def go(self):
        self.countdownEnd = None
        starting = [a for a in ALLIANCES if self.inRun[a].get()]
        if not starting:   # unticked everyone during the countdown
            self.refresh()
            return
        for a in starting:
            self.enabled[a] = True
            self.runId[a] += 1
        self.running = True
        self.lastTick = time.time()
        self.goUntil = self.lastTick + GO_SHOW_S
        beep(self.root, "go", custom=True)   # tools/sounds/go.mp3 / go.wav if present
        self.refresh()

    def stop(self):
        self.running = False
        self.countdownEnd = None
        self.goUntil = 0.0
        for a in ALLIANCES:
            self.enabled[a] = False
        self.refresh()

    def reset(self):
        self.stop()
        self.secsLeft = float(self.period()[3])
        self.refresh()

    def toggleAlliance(self, a):
        self.enabled[a] = not self.enabled[a]
        if self.enabled[a]:
            self.released = False
            # With the clock stopped this is a fresh run (an autonomous
            # period would start the routine again); mid-period it's a
            # re-enable and the routine is not restarted.
            if not self.running:
                self.runId[a] += 1
        self.refresh()

    def toggleRelease(self):
        self.stop()
        self.released = not self.released
        self.refresh()

    # ─── Clock ───────────────────────────────────────────────────────────
    def runWebCommands(self):
        """Runs what the phone / tablet page asked for, same as pressing the buttons."""
        while True:
            try:
                c = self.cmdQueue.get_nowait()
            except queue.Empty:
                return
            cmd, a = c.get("cmd"), c.get("alliance")
            if cmd == "select" and c.get("key") in PERIOD_BY_KEY:
                self.selectPeriod(c["key"])
            elif cmd == "start":
                self.start()
            elif cmd == "stop":
                self.stop()
            elif cmd == "reset":
                self.reset()
            elif cmd == "toggle" and a in ALLIANCES:
                self.toggleAlliance(a)
            elif cmd == "inrun" and a in ALLIANCES:
                self.inRun[a].set(bool(c.get("value")))
            elif cmd == "release":
                self.toggleRelease()

    def tick(self):
        self.runWebCommands()
        now = time.time()
        if self.countdownEnd is not None:
            n = self.countdownNumber()
            if n == 0:
                self.go()
            elif n != self.lastCount:
                self.lastCount = n
                beep(self.root, "count")
        elif self.running:
            self.secsLeft -= now - self.lastTick
            if self.secsLeft <= 0:
                self.secsLeft = 0
                self.stop()
                beep(self.root, "end")
                nxt = self.period()[4]
                if nxt:
                    self.periodKey = nxt
                    self.secsLeft = float(self.period()[3])
        self.lastTick = now
        self.refresh()
        self.root.after(100, self.tick)

    def robotSecs(self):
        return max(0, int(math.ceil(self.secsLeft)))

    def countdownNumber(self):
        """3, 2, 1 while counting down; 0 otherwise (and at GO)."""
        if self.countdownEnd is None:
            return 0
        return max(0, int(math.ceil(self.countdownEnd - time.time())))

    # ─── Drawing + pushing state ─────────────────────────────────────────
    def refresh(self):
        key, title, robotPeriod, _, _ = self.period()
        state = {
            "release": self.released,
            "period": robotPeriod,
            "title": title,
            "secs": self.robotSecs(),
            "running": self.running,
            "countdown": self.countdownNumber(),   # 3 / 2 / 1, 0 = not counting
            "go": time.time() < self.goUntil,
            "enabled": dict(self.enabled),
            "run": dict(self.runId),
        }
        self.server.set_state(state)

        for k, b in self.periodButtons.items():
            sel = k == key
            b.configure(bg=CARD_SEL if sel else CARD, fg=CYAN if sel else GRAY)
        secs = self.robotSecs()
        big = countdown_text(state)
        if big:
            self.clockLabel.configure(text=big[0], fg=big[1])
        else:
            self.clockLabel.configure(text=f"{secs // 60}:{secs % 60:02d}",
                                      fg=GREEN if self.running else (ORANGE if secs == 0 else WHITE))
        if self.released:
            status = "RELEASED - robots in practice mode"
        elif state["countdown"]:
            status = f"{title}  -  GET READY"
        else:
            status = title + ("" if self.running else "  (stopped)")
        self.periodLabel.configure(text=status)

        relays = self.server.relays()
        webRobots = {}
        for a in ALLIANCES:
            toggle, robotsLabel = self.allianceWidgets[a]
            on = self.enabled[a]
            toggle.configure(text="ENABLED - click to disable" if on else "DISABLED - click to enable",
                             bg=GREEN if on else DIM, fg="#0b0b0f" if on else WHITE,
                             activebackground=GREEN if on else DIM)
            lines = []
            for r in relays:
                if r["alliance"] != a:
                    continue
                lines.append(f"laptop {r['name']}")
                if not r["robots"]:
                    lines.append("   no controllers plugged in")
                for rb in r["robots"]:
                    age = rb.get("age")
                    if rb.get("error"):
                        text = "ERROR: " + rb["error"]
                    elif age is None or age > STATUS_STALE_S:
                        text = "no reply from robot"
                    else:
                        text = describe_robot(rb.get("fcs"))
                    lines.append(f"   {rb.get('port', '?'):<6} {text}")
            if not lines:
                lines = ["no relay laptop connected"]
            robotsLabel.configure(text="\n".join(lines))
            webRobots[a] = lines
        self.webState = {
            "periods": [{"key": p[0], "title": p[1], "secs": p[3]} for p in PERIODS],
            "selected": key,
            "status": status,
            "clockText": big[0] if big else f"{secs // 60}:{secs % 60:02d}",
            "clockColor": big[1] if big else (GREEN if self.running else (ORANGE if secs == 0 else WHITE)),
            "running": self.running,
            "released": self.released,
            "enabled": dict(self.enabled),
            "inRun": {a: bool(self.inRun[a].get()) for a in ALLIANCES},
            "robots": webRobots,
        }
        self.releaseButton.configure(
            text="TAKE CONTROL" if self.released else "RELEASE ROBOTS",
            bg=ORANGE if self.released else CARD, fg="#0b0b0f" if self.released else GRAY)


def main():
    ap = argparse.ArgumentParser(description="Scrimmage field control - gamemaster window")
    ap.add_argument("--port", type=int, default=TCP_PORT, help=f"TCP port (default {TCP_PORT})")
    ap.add_argument("--web-port", type=int, default=WEB_PORT,
                    help=f"phone / tablet page port (default {WEB_PORT}, 0 = off)")
    ap.add_argument("--pin", help="PIN the phone page asks for (default: random 4 digits)")
    ap.add_argument("--no-pin", action="store_true", help="no PIN on the phone page")
    args = ap.parse_args()
    try:
        server = RelayServer(args.port)
    except OSError as e:
        print(f"Can't listen on port {args.port}: {e}. Is another gamemaster window open?")
        return
    root = tk.Tk()
    app = GamemasterApp(root, server)
    if args.web_port:
        from field_web import WebServer
        pin = "" if args.no_pin else (args.pin or f"{random.randint(0, 9999):04d}")
        try:
            WebServer(args.web_port, pin, lambda: app.webState, app.cmdQueue.put)
            urls = "  ".join(f"http://{ip}:{args.web_port}" for ip in local_ips()) or f"port {args.web_port}"
            base = f"http://{(local_ips() or ['<this laptop>'])[0]}:{args.web_port}"
            app.webLabel.configure(
                text=f"Control (phone / tablet): {urls}    PIN: {pin or 'none'}\n"
                     f"Team links (view only, no PIN):  {base}/red    {base}/blue")
            print(f"Control page: {urls}  PIN: {pin or 'none'}")
            print(f"Team links:   {base}/red   {base}/blue")
        except OSError as e:
            app.webLabel.configure(text=f"Phone page off: can't use port {args.web_port} ({e})", fg=RED)
    root.mainloop()


if __name__ == "__main__":
    main()
