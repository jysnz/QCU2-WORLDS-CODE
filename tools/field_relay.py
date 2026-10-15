"""
field_relay.py -- scrimmage field control, ALLIANCE laptop

Runs on the red and the blue laptop. Takes the match state from the
gamemaster window (tools/field_gamemaster.py) over the network and passes
it to every V5 controller plugged into this laptop by USB -- each one
forwards it over its radio to its robot (src/field_control.cpp). Also
shows the alliance the clock and whether each robot is listening.

USAGE
    python tools/field_relay.py --alliance red
    python tools/field_relay.py --alliance blue
    python tools/field_relay.py --alliance red --host 192.168.1.20   # if
                       the gamemaster isn't found automatically (its IP is
                       shown at the bottom of the gamemaster window)
    python tools/field_relay.py --alliance red --ports COM5 COM7     # only
                       these controllers (default: every one plugged in,
                       re-scanned every few seconds)

BEFORE YOU RUN THIS
    - pip install pyserial (same as tools/remote_touch.py)
    - Plug each driver's controller into this laptop by USB, paired to its
      robot as usual, with this repo's code running on the brain.
    - Close the PROS terminal / remote_touch.py -- one program per
      controller. This tool closes any other vexcom it finds when it starts.
    - Program I/O only crosses the radio on the "download" channel, so the
      controllers are switched to it while this runs (same as a wireless
      upload) and switched back on exit.

SAFETY
    If this laptop loses the gamemaster, it tells its robots DISABLED. If
    this window closes or a cable comes out, the robot stops by itself
    after 2 s without a message.
"""

import argparse
import json
import os
import socket
import subprocess
import sys
import threading
import time
import tkinter as tk

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
from remote_touch import (VEX_PID_CONTROLLER, PacketStream, closeOtherVexcom,  # noqa: E402
                          find_vexcom, vex_ports)
from field_gamemaster import (ALLIANCE_COLORS, BG, CARD, DIM, DISCOVERY_PORT,  # noqa: E402
                              GRAY, GREEN, ORANGE, RED, STATUS_STALE_S, TCP_PORT, WHITE, beep,
                              countdown_text, describe_robot)

ROBOT_SEND_INTERVAL_S = 0.25   # FC line to each robot this often (robot times out at 2 s)
GM_STALE_S = 1.5               # gamemaster silent this long -> tell robots DISABLED
STATUS_INTERVAL_S = 0.5        # status report to the gamemaster this often
RESCAN_INTERVAL_S = 3.0        # look for newly plugged-in controllers this often

NO_WINDOW = getattr(subprocess, "CREATE_NO_WINDOW", 0)


def vexcomProcesses():
    """[(pid, port, parentIsRelay)] for every running `vexcom --user <port>`.
    parentIsRelay is True when its parent is a field_relay.py (any relay, so a
    second relay window on this laptop doesn't touch the first one's links).
    An orphan whose relay was force-killed has no parent and counts as False."""
    if os.name != "nt":
        return []
    script = (
        "$all = Get-CimInstance Win32_Process; "
        "foreach ($p in ($all | Where-Object { $_.Name -eq 'vexcom.exe' -and $_.CommandLine -match '--user' })) { "
        "$par = $all | Where-Object { $_.ProcessId -eq $p.ParentProcessId }; "
        "$isRelay = [bool]($par -and $par.CommandLine -match 'field_relay'); "
        "$port = if ($p.CommandLine -match '--user\\s+(\\S+)') { $Matches[1] } else { '' }; "
        "\"$($p.ProcessId),$port,$isRelay\" }")
    try:
        out = subprocess.run(["powershell", "-NoProfile", "-Command", script], capture_output=True,
                             text=True, timeout=20, creationflags=NO_WINDOW).stdout
    except (OSError, subprocess.TimeoutExpired):
        return []
    rows = []
    for line in out.splitlines():
        parts = line.strip().split(",")
        if len(parts) == 3 and parts[0].isdigit():
            rows.append((int(parts[0]), parts[1], parts[2].lower() == "true"))
    return rows


def closeForeignVexcom():
    """Like remote_touch.closeOtherVexcom(), but leaves other relays' links
    alone: only closes vexcoms that aren't owned by a field_relay (the PROS
    extension's terminal, a stray remote_touch, or an orphan left by a relay
    that was killed). Returns a note, or None if nothing was closed."""
    killed = [pid for pid, _port, mine in vexcomProcesses() if not mine]
    for pid in killed:
        subprocess.run(["taskkill", "/PID", str(pid), "/F"], capture_output=True, timeout=10,
                       creationflags=NO_WINDOW)
    if killed:
        time.sleep(1.0)  # let Windows release the port
        return f"closed {len(killed)} stray vexcom process(es) holding a controller port"
    return None


class ControllerLink:
    """One controller: a `vexcom --user <port>` child process (the only
    thing that carries program I/O over the controller's radio, see
    VexcomLink in remote_touch.py). Sends the latest FC line on a fixed
    beat -- never a backlog, the robot only cares about the newest one --
    and remembers the robot's last FCS reply."""

    def __init__(self, port, vexcom):
        self.port = port
        self.vexcom = vexcom
        self.line = None          # FC line to keep sending; None = send nothing
        self.fcs = None           # robot's last "FCS|..." payload
        self.fcsAt = 0.0
        self.error = None
        self._stop = False
        self._setChannel(1)
        self.proc = subprocess.Popen([vexcom, "--user", port], stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     creationflags=NO_WINDOW)
        threading.Thread(target=self._readLoop, daemon=True).start()
        threading.Thread(target=self._sendLoop, daemon=True).start()

    def alive(self):
        return self.error is None

    def status(self):
        age = time.time() - self.fcsAt if self.fcs else None
        return {"port": self.port, "fcs": self.fcs, "age": age, "error": self.error}

    def _readLoop(self):
        stream = PacketStream()
        while not self._stop:
            raw = self.proc.stdout.read1(4096)
            if not raw:
                break
            for line in stream.feed(raw):
                idx = line.find("FCS|")
                if idx != -1:
                    self.fcs = line[idx + 4:].strip()
                    self.fcsAt = time.time()
        if not self._stop:
            self.error = f"vexcom quit (exit {self.proc.wait()}) - port busy or unplugged?"

    def _sendLoop(self):
        while not self._stop:
            line = self.line
            if line:
                try:
                    self.proc.stdin.write((line + "\n").encode("ascii"))
                    self.proc.stdin.flush()
                except (OSError, ValueError):
                    pass
            time.sleep(ROBOT_SEND_INTERVAL_S)

    def _setChannel(self, chan):
        try:
            subprocess.run([self.vexcom, "--chan", str(chan), self.port], capture_output=True,
                           timeout=15, creationflags=NO_WINDOW)
        except (OSError, subprocess.TimeoutExpired):
            pass

    def close(self):
        self._stop = True
        try:
            self.proc.kill()
            self.proc.wait(timeout=5)
        except Exception:
            pass
        self._setChannel(0)


class GamemasterClient:
    """Finds the gamemaster (UDP announcement, or --host), keeps a TCP
    connection to it, and holds its latest state. Reconnects forever."""

    def __init__(self, alliance, host, port, robotsFn):
        self.alliance = alliance
        self.host = host
        self.port = port
        self.robotsFn = robotsFn
        self.state = None
        self.stateAt = 0.0
        self.connectedTo = None
        self.note = "looking for the gamemaster..."
        self.sock = None
        threading.Thread(target=self._loop, daemon=True).start()
        threading.Thread(target=self._statusLoop, daemon=True).start()

    def fresh(self):
        return self.state is not None and time.time() - self.stateAt < GM_STALE_S

    def _discover(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("", DISCOVERY_PORT))
        s.settimeout(3.0)
        try:
            while True:
                try:
                    data, addr = s.recvfrom(64)
                except socket.timeout:
                    return None
                parts = data.decode(errors="ignore").split()
                if len(parts) == 2 and parts[0] == "FCGM" and parts[1].isdigit():
                    return addr[0], int(parts[1])
        finally:
            s.close()

    def _loop(self):
        while True:
            target = (self.host, self.port) if self.host else self._discover()
            if not target:
                self.note = ("looking for the gamemaster... (not found yet -- same network? "
                             "firewall? or pass --host <ip>)")
                continue
            try:
                sock = socket.create_connection(target, timeout=3)
                sock.settimeout(None)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except OSError as e:
                self.note = f"can't reach gamemaster at {target[0]}:{target[1]} ({e})"
                time.sleep(1.0)
                continue
            self.sock = sock
            self.connectedTo = target[0]
            self.note = f"connected to gamemaster at {target[0]}"
            try:
                for line in sock.makefile("r", encoding="utf-8", errors="ignore"):
                    try:
                        self.state = json.loads(line)
                        self.stateAt = time.time()
                    except ValueError:
                        pass
            except OSError:
                pass
            self.sock = None
            self.connectedTo = None
            self.note = "lost the gamemaster -- robots told DISABLED, reconnecting..."
            try:
                sock.close()
            except OSError:
                pass
            time.sleep(1.0)

    def _statusLoop(self):
        name = socket.gethostname()
        while True:
            sock = self.sock
            if sock:
                msg = {"alliance": self.alliance, "name": name, "robots": self.robotsFn()}
                try:
                    sock.sendall((json.dumps(msg) + "\n").encode())
                except OSError:
                    pass
            time.sleep(STATUS_INTERVAL_S)


class RelayApp:
    def __init__(self, root, args, vexcom):
        self.root = root
        self.args = args
        self.alliance = args.alliance
        self.vexcom = vexcom
        self.links = {}           # port -> ControllerLink
        self.lastState = None     # last state we had from the gamemaster
        self.lastScan = 0.0
        self.scanning = False
        self.lastSound = None     # (countdown, go, running) last time we beeped for
        self.lock = threading.Lock()
        self.gm = GamemasterClient(self.alliance, args.host, args.port, self.robotStatuses)

        col = ALLIANCE_COLORS[self.alliance]
        root.title(f"Field Control - {self.alliance.upper()} alliance")
        root.configure(bg=BG)
        root.geometry("620x520")
        tk.Label(root, text=f"{self.alliance.upper()} ALLIANCE", bg=BG, fg=col,
                 font=("Segoe UI", 22, "bold")).pack(pady=(14, 0))
        self.periodLabel = tk.Label(root, bg=BG, fg=GRAY, font=("Segoe UI", 16, "bold"))
        self.periodLabel.pack()
        self.clockLabel = tk.Label(root, bg=BG, fg=WHITE, font=("Consolas", 80, "bold"))
        self.clockLabel.pack()
        self.enableLabel = tk.Label(root, bg=DIM, fg=WHITE, font=("Segoe UI", 18, "bold"), pady=8)
        self.enableLabel.pack(fill="x", padx=16)
        self.robotsLabel = tk.Label(root, bg=CARD, fg=WHITE, font=("Consolas", 11),
                                    justify="left", anchor="nw", padx=10, pady=8)
        self.robotsLabel.pack(fill="both", expand=True, padx=16, pady=10)
        self.gmLabel = tk.Label(root, bg=BG, fg=GRAY, font=("Segoe UI", 10), wraplength=580,
                                justify="left")
        self.gmLabel.pack(fill="x", padx=16, pady=(0, 10))

        root.protocol("WM_DELETE_WINDOW", self.close)
        self.tick()

    def robotStatuses(self):
        with self.lock:
            return [l.status() for l in self.links.values()]

    def scanControllers(self):
        """Runs on its own thread (switching a controller's channel takes a
        few seconds), one scan at a time; the lock is only held for the
        dict updates so the window never waits on vexcom."""
        try:
            if self.args.ports:
                wanted = self.args.ports
            else:
                wanted = [p.device for p in vex_ports() if p.pid == VEX_PID_CONTROLLER]
            # A port another relay window already holds is theirs -- leave it,
            # so red and blue relays on one laptop each keep their own
            # controllers instead of fighting over all of them.
            ours = set()
            with self.lock:
                ours = {l.proc.pid for l in self.links.values()}
            taken = {port for pid, port, mine in vexcomProcesses() if mine and pid not in ours}
            wanted = [p for p in wanted if p not in taken]
            # a link whose vexcom died (unplugged) is dropped and retried
            with self.lock:
                dead = [self.links.pop(p) for p in [p for p, l in self.links.items() if not l.alive()]]
                new = [p for p in wanted if p not in self.links]
            for l in dead:
                l.close()
            for port in new:
                link = ControllerLink(port, self.vexcom)
                with self.lock:
                    self.links[port] = link
        finally:
            self.scanning = False

    def robotLine(self):
        """The FC line every robot on this alliance should hear right now
        (see field_control.hpp), or None before the gamemaster ever spoke
        -- robots then stay in normal practice mode."""
        if self.gm.fresh():
            self.lastState = self.gm.state
        st = self.lastState
        if st is None:
            return None
        if st.get("release"):
            return "FC OFF" if self.gm.fresh() else None
        on = self.gm.fresh() and st.get("enabled", {}).get(self.alliance, False)
        run = st.get("run", {}).get(self.alliance, 0)
        return f"FC {st.get('period', 'D')} {1 if on else 0} {int(st.get('secs', 0))} {run}"

    def tick(self):
        now = time.time()
        if not self.scanning and now - self.lastScan > RESCAN_INTERVAL_S:
            self.lastScan = now
            self.scanning = True
            threading.Thread(target=self.scanControllers, daemon=True).start()

        line = self.robotLine()
        with self.lock:
            links = list(self.links.values())
        for l in links:
            l.line = line

        st = self.lastState
        fresh = self.gm.fresh()
        if st is None:
            self.periodLabel.configure(text="waiting for the gamemaster")
            self.clockLabel.configure(text="-:--", fg=GRAY)
            self.enableLabel.configure(text="PRACTICE MODE (not under field control)", bg=DIM,
                                       fg=WHITE)
        else:
            secs = int(st.get("secs", 0))
            counting = fresh and st.get("countdown", 0) > 0
            self.periodLabel.configure(text=st.get("title", "") + ("  -  GET READY" if counting
                                                                   else ""))
            big = countdown_text(st) if fresh else None
            if big:
                self.clockLabel.configure(text=big[0], fg=big[1])
            else:
                self.clockLabel.configure(text=f"{secs // 60}:{secs % 60:02d}",
                                          fg=GREEN if st.get("running") and fresh else WHITE)
            self.playSounds(st if fresh else {})
            on = fresh and st.get("enabled", {}).get(self.alliance, False)
            if counting:
                self.enableLabel.configure(text="GET READY", bg=ORANGE, fg="#0b0b0f")
            elif st.get("release"):
                self.enableLabel.configure(text="RELEASED - practice mode", bg=ORANGE, fg="#0b0b0f")
            elif not fresh:
                self.enableLabel.configure(text="DISABLED - no gamemaster signal", bg=RED, fg=WHITE)
            elif on:
                self.enableLabel.configure(text="ENABLED", bg=GREEN, fg="#0b0b0f")
            else:
                self.enableLabel.configure(text="DISABLED", bg=DIM, fg=WHITE)

        rows = []
        for s in self.robotStatuses():
            if s["error"]:
                text = "ERROR: " + s["error"]
            elif s["age"] is None or s["age"] > STATUS_STALE_S:
                text = ("no reply yet (robot on? paired? running this repo's code?)"
                        if line else "waiting")
            else:
                text = describe_robot(s["fcs"])
            rows.append(f"{s['port']:<6} {text}")
        if not rows:
            rows = ["no V5 controller plugged in by USB"]
        self.robotsLabel.configure(text="\n".join(rows))
        self.gmLabel.configure(text=self.gm.note)

        self.root.after(100, self.tick)

    def playSounds(self, st):
        """Echo the gamemaster's countdown beeps / GO / end-of-period sound."""
        now = (st.get("countdown", 0), bool(st.get("go")), bool(st.get("running")))
        prev = self.lastSound
        self.lastSound = now
        if prev is None:
            return
        if now[0] and now[0] != prev[0]:
            beep(self.root, "count")
        elif now[1] and not prev[1]:
            beep(self.root, "go")
        elif prev[2] and not now[2]:   # period over (or STOP / signal lost)
            beep(self.root, "end")

    def close(self):
        with self.lock:
            links = list(self.links.values())
            self.links.clear()
        for l in links:
            l.close()
        self.root.destroy()


def main():
    ap = argparse.ArgumentParser(description="Scrimmage field control - alliance relay")
    ap.add_argument("--alliance", required=True, choices=("red", "blue"))
    ap.add_argument("--host", help="gamemaster laptop IP (default: find it automatically)")
    ap.add_argument("--port", type=int, default=TCP_PORT, help=f"gamemaster TCP port ({TCP_PORT})")
    ap.add_argument("--ports", nargs="+", help="controller COM ports (default: all plugged in)")
    ap.add_argument("--vexcom", help="path to vexcom.exe (default: the PROS extension's copy)")
    args = ap.parse_args()

    vexcom = args.vexcom or find_vexcom()
    if not vexcom:
        print("Couldn't find vexcom.exe (it ships with the PROS VS Code extension). "
              "Install that, or pass --vexcom PATH\\TO\\vexcom.exe")
        return
    note = closeForeignVexcom()
    if note:
        print(note)

    root = tk.Tk()
    RelayApp(root, args, vexcom)
    root.mainloop()


if __name__ == "__main__":
    main()
