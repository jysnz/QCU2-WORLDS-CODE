"""
Link through the V5 *controller* (the radio), for tools/remote_touch.py -- debi's way of reaching the brain.

When the laptop is plugged into the controller instead of the brain, the only serial port is "VEX V5 Controller
Port", and plain serial carries none of the program's text there. `vexcom --user <port>` (installed with the PROS
VS Code extension) is what relays the program's stdout/stdin over the radio, so this runs it as a child process.
Same interface as remote_touch.Link: lines (queue of "RUI|..." lines), send(list), dead, error, close().
"""
import glob
import os
import queue
import shutil
import subprocess
import threading
import time

import lift_tuner as lt

NO_WINDOW = getattr(subprocess, "CREATE_NO_WINDOW", 0)


def is_controller_port(device):
    """True if `device` is a V5 controller (radio link) rather than a brain."""
    try:
        import serial.tools.list_ports
        for p in serial.tools.list_ports.comports():
            if p.device == device:
                return p.pid == 0x0503 or "controller" in (p.description or "").lower()
    except Exception:
        pass
    return False


def find_vexcom():
    """vexcom.exe as installed by the PROS VS Code extension, or on PATH."""
    found = shutil.which("vexcom")
    if found:
        return found
    roots = [os.path.join(os.path.expandvars("%APPDATA%"), "Code", "User", "globalStorage", "sigbots.pros", "install"),
             os.path.expanduser("~/.config/Code/User/globalStorage/sigbots.pros/install")]
    for root in roots:
        hits = [h for h in glob.glob(os.path.join(root, "vex-vexcom-*", "vexcom*"))
                if os.path.isfile(h) and not h.endswith(".txt")]
        if hits:
            return hits[0]
    return None


def close_other_vexcom():
    """The PROS VS Code extension's terminal runs its own `vexcom --user`, which holds the only port. Close it."""
    if os.name != "nt":
        return
    try:
        out = subprocess.run(["tasklist", "/FI", "IMAGENAME eq vexcom.exe", "/FO", "CSV", "/NH"],
                             capture_output=True, text=True, timeout=10, creationflags=NO_WINDOW).stdout
    except (OSError, subprocess.TimeoutExpired):
        return
    pids = [line.split('","')[1] for line in out.splitlines() if line.startswith('"vexcom.exe"')]
    for pid in pids:
        subprocess.run(["taskkill", "/PID", pid, "/F"], capture_output=True, timeout=10, creationflags=NO_WINDOW)
    if pids:
        time.sleep(1.0)  # let Windows release the port


class VexcomLink:
    name = "brain"
    SEND_GAP_S = 0.06   # lines written back-to-back get lost on the radio, so they are spaced out

    def __init__(self, port, vexcom, debug=False, keep=("RUI|",)):
        self.port, self.vexcom, self.debug = port, vexcom, debug
        self.keep = tuple(keep)   # line prefixes that go to .lines (the lift tuner wants LTL| / LTC|)
        close_other_vexcom()
        # The brain only relays program I/O over the radio on the "download" channel (1); put it back on close().
        self._set_channel(1)
        self.proc = subprocess.Popen([vexcom, "--user", port], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, creationflags=NO_WINDOW)
        self.lines = queue.Queue()
        self.pid_lines = queue.Queue()   # the PID tuner's run stream (PID|... / CSV,...)
        self.out = queue.Queue()
        self.dead = False
        self.stop = False
        self.error = None
        self._tail = []
        threading.Thread(target=self._read, daemon=True).start()
        threading.Thread(target=self._write, daemon=True).start()
        if "RUI|" in self.keep:
            self.send(["RATE 250"])   # slower than over a cable

    def _set_channel(self, chan):
        try:
            subprocess.run([self.vexcom, "--chan", str(chan), self.port], capture_output=True, timeout=15,
                           creationflags=NO_WINDOW)
        except (OSError, subprocess.TimeoutExpired):
            pass

    def _read(self):
        stream = lt.PacketStream()
        while not self.stop:
            raw = self.proc.stdout.read1(4096) if hasattr(self.proc.stdout, "read1") else self.proc.stdout.read(1)
            if not raw:
                break
            for line in stream.feed(raw):
                self._tail = (self._tail + [line.strip()])[-3:]
                if line.startswith(self.keep):
                    if self.debug:
                        print("<", line)
                    self.lines.put(line)
                elif line.startswith(("PID|", "CSV,")):
                    self.pid_lines.put(line.rstrip("\r\n"))
        if not self.stop:
            self.dead = True
            code = self.proc.wait()
            self.error = (f"vexcom quit (exit {code}: {' / '.join(t for t in self._tail if t) or 'no message'}). "
                          f"Is the PROS terminal in VS Code (or another vexcom) still using {self.port}?")

    def _write(self):
        while not self.stop:
            try:
                line = self.out.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                self.proc.stdin.write((line + "\n").encode("ascii", errors="ignore"))
                self.proc.stdin.flush()
            except (OSError, ValueError):
                pass
            time.sleep(self.SEND_GAP_S)

    def send(self, lines):
        for line in lines:
            if self.debug:
                print(">", line)
            self.out.put(line)

    def close(self):
        t = time.time()
        while not self.out.empty() and time.time() - t < 2:   # let anything queued go out first
            time.sleep(0.05)
        self.stop = True
        try:
            self.proc.kill()
            self.proc.wait(timeout=5)
        except Exception:
            pass
        self._set_channel(0)
