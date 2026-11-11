#!/usr/bin/env python3
"""
Path Planner -- the laptop half of the brain's PATH PLANNER screen.

Opened automatically by tools/remote_touch.py when the brain enters that
screen (HOME -> PATH PLANNER). Not meant to be run on its own: it talks
to the robot through remote_touch's link and gets the robot's live pose
from the same RUI| lines the mirror uses.

WHAT'S ON IT
    Top       a big field map. Drop the VEX field render as
              tools/field.png (e.g. from field-rendering.jerryio.com --
              a top-down view, any size, square) and it's used as the
              background; without it you get a plain gridded field. The
              planned route is drawn over it, plus the robot's live pose
              and the path it actually drove.
    Middle    the block list -- one line per movement, in run order --
              with move up / down / duplicate / delete.
    Bottom    movement buttons that append a block (turnToHeading,
              turnToPoint, swingToHeading, swingToPoint, moveToPoint,
              moveToPose, wait, setPose, waitUntilDone) and, for the
              selected block, every lemlib parameter that movement takes.
              Adding a movement also adds a waitUntilDone after it, since
              lemlib motions are asynchronous -- without it the exported
              code would start the next movement immediately.

    Each block's movement is drawn on the map from wherever the previous
    block left the robot, by *simulating* lemlib's motion controllers on
    a model of this robot (track width, wheel size, rpm and the PID /
    exit settings are read from src/main.cpp): the curve a moveToPoint
    really takes as it turns while driving, moveToPose's boomerang
    approach, swings pivoting around the locked side (which moves the
    robot), backwards approaches, min/max speed and early-exit effects.
    So the stack of blocks reads as the route the robot will drive.

    Clicking the map sets the selected block's target point (for blocks
    that have one), and any block's point can be dragged around the map
    directly. "Pick start" then a click sets the robot's start pose
    instead (sent to the robot as chassis.setPose). "Reset robot" puts
    the robot's pose back to the start pose and clears the driven path /
    trail and the last run's status, on the map and on the brain.

DRAWING A PATH WITH THE MOUSE (path.jerryio style)
    "Draw path" (D) turns the map into a path editor, and which mouse
    button you use says what kind of point you're dropping:

        left click    a CURVE point -- the route bends smoothly through
                      it (a Catmull-Rom spline, the curve path.jerryio
                      draws through its control points), resampled every
                      "spacing" inches.
        Merge (M)     tap it, click the points you want joined (first to
                      last), tap it again: they all become curve points, so
                      those legs flow together as one smooth spline.
        right click   a STRAIGHT point -- the legs either side of it are
                      dead straight, and the robot squares up and stops
                      on it before carrying on.

    So a run of left-clicks is one flowing curve, and a right-click puts
    a hard corner in the middle of it. Right-clicking a point you've
    already dropped flips it between the two, so a curve can be pinned
    straight (or a corner rounded off) after the fact.

    Dragging a point moves it, shift-click or middle-click removes one,
    Ctrl+Z / Backspace undoes the last.

    Untick "smooth" for a turn-and-drive route instead: every leg is
    straight, and wherever the route turns left or right the robot is
    aimed down the next leg with a turnToHeading first, so each
    moveToPoint drives a straight line rather than steering itself onto
    the target as it goes. Legs that carry straight on don't get a turn.

    "Apply" turns the drawn route into ordinary blocks: the first point
    becomes the start pose (facing the second point) and every point
    after it becomes a moveToPoint. Points along a curve get a minSpeed
    and an earlyExitRange of half the spacing, so the robot flows
    through them instead of stopping at each one; straight points (and
    the end of the route) get neither, so it arrives on them properly.
    From there they're just blocks -- editable, runnable, exportable --
    and "Trace blocks" goes the other way, loading the existing blocks'
    points back in as control points to re-draw.

DRIVING IT FROM HERE
    "Drive (WASD)" hands the drivetrain to the keyboard: W / S drive
    forwards and back, A / D turn, held together they mix (arcade). The
    power slider sets how hard, and the robot only moves while a key is
    actually down.

    The robot is told to move by "DRIVE left right" lines, which expire on
    the brain after 300 ms. The planner repeats the current command as a
    keepalive while you hold a key and sends a stop the moment you let go,
    lose the window's focus, or switch the mode off -- so a dropped radio
    line, a window that loses focus mid-key, or this program being killed
    outright all end with the robot stopped rather than driving on.

    This needs the firmware that understands DRIVE (src/driver_menu.cpp);
    an older brain will simply ignore it. The brain has to be on the
    driver menu (any screen of it) for the command to be applied.

RUN / EXPORT
    F5 / Run all       sends the whole list to the brain and runs it there,
                       block after block, exactly like an autonomous would.
    F6 / Run selected  runs just the highlighted block.
    Esc / Stop         cancels (same as controller X).
    Export code        writes the C++ for the list to tools/exported_path.cpp,
                       copies it to the clipboard, and shows it in a window.
                       The export opens with the robot's own configuration
                       (drivetrain + both PID controllers, read from
                       src/main.cpp) as a record of what the route was
                       planned against, then chassis.calibrate(), then the
                       start pose, then the movements.
    Save / Load        the block list as JSON (tools/plans/*.json).

COORDINATES
    LemLib's: inches, origin at field centre, +x right, +y up (away from
    you), heading 0 = +y and increasing clockwise. Same as the brain.

CODE
    This file is the window. The planning logic (blocks, simulator, estimate, checks,
    export) is in planner_core.py, and is tested by tools/tests/test_planner.py.
"""

import copy
import json
import math
import os
import re
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import run_analysis
from planner_core import *  # noqa: F401,F403  (the logic; names, not _private ones)


# ─── The window ──────────────────────────────────────────────────────────────
class PlannerWindow(tk.Toplevel):
    def __init__(self, master, link):
        super().__init__(master)
        self.link = link
        self.planName = None           # file the plan was last saved to / loaded from
        self.editFunc = None           # autonomous.cpp function being edited (Edit auto...)
        self._retitle()
        self._applyTheme()
        self.blocks = []
        self.selected = None
        self.start = [0.0, 0.0, 0.0]   # start pose used to draw the route
        self.pickStart = False
        self.driveMode = False         # WASD driving on/off
        self.driveKeys = set()         # keys held right now
        self.driveLast = None          # last (l, r) sent, to avoid resending
        self.driveJob = None           # keepalive callback
        self.drawMode = False          # mouse path editor on/off
        self.waypoints = []            # control points: (x, y, CURVE/STRAIGHT[, heading])
        self.selWp = None              # the drawn point the heading buttons act on
        self.showFunctions = True      # robot function badges on the map (F)
        self.mergeMode = False         # Merge: click drawn points to pick them, then Merge again
        self.mergeSel = set()          # indices of the drawn points picked for merging
        self.drag = None               # ("wp", i) or ("block", i) while dragging
        self.robotPose = None
        self.robotPath = []
        self.planStatus = ("IDLE", -1, 0)
        self.pendingRun = None
        self._uploadList, self._uploadSpans = [], []   # what was last sent to the brain
        self.uploadStarted = 0.0
        self.uploadTries = 0
        self.statusHold = 0.0
        self.onBrainScreen = False
        self.fieldImage = None
        self._redrawPending = False    # see redrawMap(): draws are coalesced
        self._simCache = (None, None)  # (key, simulate() result)
        self._frameSim = None          # simulate() result pinned for the length of one paint
        self._lastState = None         # last robot state drawn, to skip no-ops
        self.startPlaced = False       # True once the robot has been put somewhere on the map
        self.mapPx, self.mapX0, self.mapY0 = MAP_PX, 0, 0   # the square map inside the canvas
        self._fieldSrc = None          # field.png at full size (Pillow only)
        self._relayoutJob = None
        self.sidebarShown = True
        self._livePending = False
        self.pathLegs = []             # per drawn leg: straight (True) or part of a curve
        self.aimPending = False        # a swing / turn just added in draw mode wants the next click
        self.pending = None            # (block, insert index): a motion being placed, not applied yet
        self.pinEdit = False           # Edit pins mode: drag / add / delete the pin locations
        self._pinPrev = None           # while dragging a pin: where it was a moment ago
        self.pendingPin = None         # a goToPin being placed: the pin tapped so far (PINS index)
        self.pendingGoal = None        # a shootAtGoal being placed: the goal tapped (GOALS index)
        self.hoverSwing = None         # block index of the swing the mouse is over
        self._chips = []               # clickable shape options drawn beside a selected swing / turn
        self.undoStack = []            # plan snapshots taken before each applied path
        self.redoStack = []            # what Ctrl+Z took away: ("wp", point) / ("plan", snapshot)
        self.replaceOnApply = False    # True while re-shaping a traced route
        # Hiding the window must not leave the robot driving on.
        self.protocol("WM_DELETE_WINDOW", lambda: (self.stopDrive(), self.withdraw()))

        self.clipboard = None          # the block copied with Ctrl+C
        self.hideGlueVar = tk.BooleanVar(value=False)    # View: leave the waitUntilDone rows out
        self.logReader = run_analysis.LogReader()        # the robot's recording of a run, as it arrives
        self.reportAutoVar = tk.BooleanVar(value=True)   # open the run report when a run ends
        self.lastRunText = ""
        self._reportWin = None
        self._awaitLog = None          # when a run ended with no recording received yet
        self._logRxAt = None           # when the last LOG| line arrived
        self._logAsks = 0              # times the robot was asked for the lines still missing
        self._logStuck = 0             # ...of those, how many in a row brought nothing new
        self._logGot = 0               # lines in hand at the last ask
        self._logWaitAsks = 0          # times it was asked for a recording that never started
        self._logDone = None           # (run id, until): the recording just reported on
        self._logAcks = 0              # LOG NEED requests the robot said it received
        self._rowMap = []              # list row -> block index (rows can be hidden)
        self._savedKey = self._dirtyKey()
        self._buildUi()
        self._bindKeys()
        self.redrawMap()
        self.refreshList()
        self._sizeWindow()
        self.after(AUTOSAVE_MS, self._autosaveTick)
        self.after(700, self._offerRestore)
        self.after(self.LOG_TICK_MS, self._logTick)

    def _sizeWindow(self):
        """Open at a size that fits the screen, taskbar and title bar included
        -- the side panel scrolls, so nothing needs the window to be tall --
        and let it shrink well below that."""
        sw, sh = self.winfo_screenwidth(), self.winfo_screenheight()
        w = min(1200, sw - 40)
        h = max(480, min(720, sh - 110))
        self.geometry(f"{w}x{h}+{max(0, (sw - w) // 2)}+{max(0, (sh - h) // 3)}")
        self.minsize(min(760, w), min(460, h))

    # -- keeping work safe: an autosave of anything not yet saved, offered back after a crash --
    def _dirtyKey(self):
        return repr((self.blocks, list(self.start)))

    def _autosaveTick(self):
        try:
            if self.blocks and self._dirtyKey() != self._savedKey:
                with open(AUTOSAVE_PATH, "w", encoding="utf-8") as f:
                    json.dump({"start": self.start, "blocks": self.blocks,
                               "saved": time.strftime("%Y-%m-%d %H:%M:%S")}, f)
            elif os.path.exists(AUTOSAVE_PATH) and self._dirtyKey() == self._savedKey:
                os.remove(AUTOSAVE_PATH)
        except (OSError, TypeError, ValueError):
            pass                        # a failed autosave must never get in the way
        try:
            if self.winfo_exists():
                self.after(AUTOSAVE_MS, self._autosaveTick)
        except tk.TclError:
            pass

    def _offerRestore(self):
        """If the last session ended with an unsaved plan, offer it back."""
        try:
            with open(AUTOSAVE_PATH, encoding="utf-8") as f:
                data = json.load(f)
        except (OSError, ValueError):
            return
        if not data.get("blocks") or self.blocks:
            return
        if messagebox.askyesno("Recover plan",
                               f"An unsaved plan from {data.get('saved', 'last time')} was "
                               f"found ({len(data['blocks'])} blocks).\n\nRestore it?",
                               parent=self):
            self._installPlan(data, "(recovered)")
            self._savedKey = None       # still unsaved: keep autosaving it
        else:
            try:
                os.remove(AUTOSAVE_PATH)
            except OSError:
                pass

    def _markSaved(self):
        self._savedKey = self._dirtyKey()
        try:
            os.remove(AUTOSAVE_PATH)
        except OSError:
            pass

    # -- a last look before a run --
    def planWarnings(self):
        self._syncCtx()
        try:
            return plan_warnings(self.blocks, self.start)
        except Exception as err:        # the check must never stop a run on its own
            return [f"The plan check itself failed ({err}); the plan may be invalid."]

    def flipPlan(self, mode, what=None):
        """Path menu: the plan as it would be driven from somewhere else on the field."""
        if not self.blocks:
            return
        if self.pending:
            self.cancelPending()
        self.pushUndo()
        self.blocks, start = flip_plan(self.blocks, self.start, mode)
        self.start = list(start)
        for v, val in zip(self.startVars, self.start):
            v.set(fmt(val))
        fx, fy = FLIP_AXES[mode]
        self.waypoints = [(0.0 - p[0] if fx else p[0], 0.0 - p[1] if fy else p[1]) + tuple(p[2:])
                          for p in self.waypoints]
        self.selected = self.selected if self.selected is not None and \
            self.selected < len(self.blocks) else None
        self.refreshList()
        self.showParams()
        gaps = flip_report(self.blocks)
        what = what or {"rotate": "Rotated half a way round the field.",
                        "mirror": "Mirrored left to right.",
                        "mirror_y": "Mirrored top to bottom."}[mode]
        messagebox.showinfo(
            "Move plan", what + "  Ctrl+Z undoes it."
            + ("\n\nCheck these:\n" + "\n".join("- " + g for g in gaps) if gaps else ""),
            parent=self)

    # -- the report after a run: what the robot recorded, turned into something to read --
    # The link (a controller radio) drops lines, so a recording rarely arrives whole the first
    # time. The robot keeps it and numbers its lines; whenever the lines stop coming with some
    # still missing, the planner asks for exactly those again -- and if the robot stops answering,
    # reports on what did arrive instead of showing nothing.
    LOG_TICK_MS = 300
    LOG_QUIET_S = 1.2          # no line for this long = the robot has finished sending
    LOG_MAX_ASKS = 40          # each ask is one "LOG NEED" (up to ~25 runs of missing lines)
    LOG_MAX_STUCK = 8          # asks in a row that brought nothing: give up, report what arrived

    def feedLogLine(self, line):
        """One LOG| line from the robot (see src/run_log.cpp); a finished recording becomes a
        report. Called from the serial pump, so it must never raise."""
        try:
            if self._logDone and time.time() < self._logDone[1] and \
                    line.startswith(f"LOG|#{self._logDone[0]}."):
                return                          # a late line of the recording just reported
            if line.startswith("LOG|ACK|"):     # the robot got a LOG NEED
                self._logAcks += 1
                return
            if not self.logReader.active:       # a new recording begins
                self._logAsks = self._logStuck = self._logGot = self._logAcks = 0
            self._awaitLog = None
            self._logRxAt = time.time()
            run = self.logReader.feed(line)
            if run is not None:
                self._logRxAt = None
                self._onRunLog(run)
        except Exception as err:        # a bad recording must not take the planner down
            self.statusHold = time.time() + 8
            self.brainStatus.config(text=f"run report failed: {err}", fg="#FF6B81")

    def _logTick(self):
        try:
            self._watchLog()
        except Exception as err:        # (tk would print it and the tick would stop for good)
            print("run log:", err)
        try:
            if self.winfo_exists():
                self.after(self.LOG_TICK_MS, self._logTick)
        except tk.TclError:
            pass

    def _watchLog(self):
        """Every 300 ms: chases a recording until it is whole. A run that ended with nothing coming
        back is asked for; a recording with lines missing has them asked for again; one that
        cannot be completed is reported as far as it arrived."""
        now = time.time()
        if self._awaitLog is not None:
            waited = now - self._awaitLog
            if waited > 9:
                self._awaitLog = None
                self._logWaitAsks = 0
                self.statusHold = now + 12
                self.brainStatus.config(
                    text="no run recording arrived from the robot, even after asking for it -- is "
                         "the firmware with src/run_log.cpp uploaded? "
                         "(python remote_touch.py --debug shows the LOG| lines)", fg="#FF8C00")
            elif waited > 2.5 * (self._logWaitAsks + 1) and self.link:
                self._logWaitAsks += 1
                self.link._send("LOG NEED 0 ALL")       # 0 = whichever recording is the last
                self.link._send("NOP")
        else:
            self._logWaitAsks = 0
        if self._logRxAt is None or not self.logReader.active:
            return
        if now - self._logRxAt < self.LOG_QUIET_S:
            return
        got = self.logReader.received()
        self._logStuck = self._logStuck + 1 if got <= self._logGot and self._logAsks else 0
        self._logGot = got
        cmd = self.logReader.need_command()
        if self.planStatus[0] == "RUNNING":     # a new run began: the robot dropped that recording
            cmd = None
        if cmd and self.link and self._logAsks < self.LOG_MAX_ASKS and \
                self._logStuck < self.LOG_MAX_STUCK:
            self._logAsks += 1
            self._logRxAt = now                 # wait for the answer before asking again
            self.link._send(cmd)
            self.link._send("NOP")              # chaser: nudges a stuck last line through the radio
            miss = self.logReader.missing() or []
            self.statusHold = now + 3
            self.brainStatus.config(text=f"run recording: asking the robot again for "
                                         f"{len(miss)} missing line(s)...", fg="#FF8C00")
            return
        self._logRxAt = None
        run = self.logReader.take()             # cannot be completed: report what did arrive
        if run is not None:
            run["asks"] = (self._logAsks, self._logAcks)
            self._onRunLog(run)
        else:
            self.statusHold = now + 12
            self.brainStatus.config(text="the run recording was lost on the link (nothing usable "
                                         "arrived). Run again, or use the USB cable.", fg="#FF8C00")

    def showSampleReport(self):
        """Robot > Sample run report: a made-up recording, to see what the report looks like."""
        reader = run_analysis.LogReader()
        run = None
        for ln in run_analysis.demo_lines(sway=3.0, start_heading=5.0, speed=26.0):
            run = reader.feed(ln) or run
        self.lastRunText = "SAMPLE (made up, not your robot)\n\n" + run_analysis.format_report(run)
        self.showRunReport()

    def _onRunLog(self, run):
        self._logDone = (run["id"], time.time() + 5)
        flat = start = None
        if run["name"] in ("plan", "step") and getattr(self, "_uploadList", None):
            flat, start = self._uploadList, tuple(self.start)
        try:
            self.lastRunText = run_analysis.format_report(run, flat, start)
        except Exception as err:        # e.g. the plan on screen is not the one that ran
            self.lastRunText = (f"(the comparison with the plan failed: {err})\n\n"
                                + run_analysis.format_report(run))
        try:
            path = run_analysis.save_report(run, self.lastRunText)
        except OSError:
            path = None
        whole = run.get("complete", True)
        self.statusHold = time.time() + 10
        self.brainStatus.config(
            text=f"run report ready ({len(run['samples'])} samples"
                 + ("" if whole else ", INCOMPLETE: the link dropped part of it")
                 + ") -- Robot > Last run report"
                 + (f"; saved to {os.path.relpath(path, HERE)}" if path else ""),
            fg="#2ECC71" if whole else "#FF8C00")
        if self.reportAutoVar.get():
            self.showRunReport()

    def showRunReport(self):
        """Robot > Last run report: the text to read, copy and send to whoever diagnoses it."""
        if not self.lastRunText:
            messagebox.showinfo("Run report", "No run has been recorded yet. Run the plan from "
                                "here (F5) and the report appears when it ends.", parent=self)
            return
        if self._reportWin is not None:
            try:
                self._reportWin.destroy()
            except tk.TclError:
                pass
        win = self._reportWin = tk.Toplevel(self)
        win.title("Run report")
        win.configure(bg=FIELD_BG)
        win.geometry("860x620")
        bar = tk.Frame(win, bg=FIELD_BG)
        bar.pack(fill="x", padx=8, pady=6)
        note = tk.Label(bar, text="", fg="#2ECC71", bg=FIELD_BG)

        def copy():
            self.clipboard_clear()
            self.clipboard_append(self.lastRunText)
            note.config(text="copied -- paste it to whoever is diagnosing the run")

        tk.Button(bar, text="Copy report", command=copy).pack(side="left")
        tk.Button(bar, text="Open reports folder",
                  command=lambda: os.path.isdir(run_analysis.REPORTS_DIR) and
                  os.startfile(run_analysis.REPORTS_DIR)).pack(side="left", padx=6)
        note.pack(side="left", padx=10)
        txt = tk.Text(win, wrap="none", bg=INPUT_BG, fg=TEXT, font=("Consolas", 10),
                      relief="flat", padx=10, pady=8)
        sy = tk.Scrollbar(win, command=txt.yview)
        txt.config(yscrollcommand=sy.set)
        sy.pack(side="right", fill="y")
        txt.pack(fill="both", expand=True, padx=(8, 0), pady=(0, 8))
        txt.insert("1.0", self.lastRunText)
        txt.config(state="disabled")

    def checkPlan(self):
        w = self.planWarnings()
        if w:
            messagebox.showwarning("Check plan", "\n".join("- " + t for t in w), parent=self)
        else:
            messagebox.showinfo("Check plan", "No problems found: the route stays on the field, "
                                "avoids the goals and fits in the autonomous period.", parent=self)

    # -- copy / paste a block --
    def copyBlock(self):
        if self.selected is not None and self.selected < len(self.blocks):
            self.clipboard = copy.deepcopy(self.blocks[self.selected])
            self.brainStatus.config(text=f"copied block {self.selected + 1}", fg="#2ECC71")

    def pasteBlock(self):
        if not self.clipboard:
            return
        self.pushUndo()
        at = self.selected + 1 if self.selected is not None else len(self.blocks)
        self.blocks.insert(at, copy.deepcopy(self.clipboard))
        self.selected = at
        self.refreshList()
        self.showParams()

    def _retitle(self):
        self.title("Path Planner -- V5" + (f"  --  {self.planName}" if self.planName else ""))

    def _applyTheme(self):
        """One modern dark look for every widget made after this: flat, rounded-feeling
        controls on deep blue-black, one accent colour. Anything that sets its own colours
        (the run buttons, the map icons) still wins."""
        for pattern, value in (
                ("*Entry.background", INPUT_BG), ("*Entry.foreground", TEXT),
                ("*Entry.insertBackground", TEXT), ("*Entry.relief", "flat"),
                ("*Entry.highlightThickness", 1), ("*Entry.highlightBackground", CARD_BORDER),
                ("*Entry.highlightColor", ACCENT),
                ("*Entry.selectBackground", ACCENT), ("*Entry.selectForeground", "#0b0d12"),
                ("*Button.background", BTN_BG), ("*Button.foreground", TEXT),
                ("*Button.activeBackground", BTN_HOVER), ("*Button.activeForeground", "#ffffff"),
                ("*Button.relief", "flat"), ("*Button.borderWidth", 0),
                ("*Button.padX", 10), ("*Button.padY", 4), ("*Button.cursor", "hand2"),
                ("*Button.font", ("Segoe UI", 9)),
                ("*Checkbutton.background", PANEL_BG), ("*Checkbutton.foreground", TEXT),
                ("*Checkbutton.activeBackground", PANEL_BG),
                ("*Checkbutton.activeForeground", "#ffffff"),
                ("*Checkbutton.selectColor", INPUT_BG), ("*Checkbutton.font", ("Segoe UI", 9)),
                ("*Listbox.background", INPUT_BG), ("*Listbox.foreground", TEXT),
                ("*Listbox.relief", "flat"), ("*Listbox.highlightThickness", 0),
                ("*Scale.background", PANEL_BG), ("*Scale.troughColor", INPUT_BG),
                ("*Scale.foreground", TEXT), ("*Scale.highlightThickness", 0),
                ("*Scrollbar.background", BTN_BG), ("*Scrollbar.troughColor", INPUT_BG),
                ("*Scrollbar.relief", "flat"), ("*Scrollbar.borderWidth", 0),
                ("*Label.background", PANEL_BG), ("*Label.foreground", "#c9cedb"),
                ("*Label.font", ("Segoe UI", 9)),
                ("*LabelFrame.background", PANEL_BG), ("*LabelFrame.foreground", ACCENT),
                ("*Frame.background", PANEL_BG)):
            # scoped to this window (Tk option patterns are "*<window name>*Class.option"; a
            # bare window path matches nothing), so the main remote window keeps its own look
            self.option_add(f"*{str(self)[1:]}{pattern}", value)
        st = ttk.Style(self)
        try:
            st.theme_use("clam")        # the one built-in theme whose colours can all be set
        except tk.TclError:
            pass
        st.configure("TNotebook", background=FIELD_BG, borderwidth=0, tabmargins=(0, 0, 0, 0))
        st.configure("TNotebook.Tab", background=FIELD_BG, foreground=MUTED, padding=(14, 7),
                     borderwidth=0, font=("Segoe UI", 10, "bold"))
        st.map("TNotebook.Tab",
               background=[("selected", PANEL_BG), ("active", BTN_BG)],
               foreground=[("selected", ACCENT), ("active", TEXT)])
        st.configure("TCombobox", fieldbackground=INPUT_BG, background=BTN_BG, foreground=TEXT,
                     arrowcolor=TEXT, bordercolor=CARD_BORDER, lightcolor=INPUT_BG,
                     darkcolor=INPUT_BG, selectbackground=INPUT_BG, selectforeground=TEXT,
                     padding=3)
        st.map("TCombobox", fieldbackground=[("readonly", INPUT_BG)],
               foreground=[("readonly", TEXT)], bordercolor=[("focus", ACCENT)])
        self.option_add(f"*{str(self)[1:]}*TCombobox*Listbox.background", INPUT_BG)
        self.option_add(f"*{str(self)[1:]}*TCombobox*Listbox.foreground", TEXT)
        self.option_add(f"*{str(self)[1:]}*TCombobox*Listbox.selectBackground", ACCENT)
        self.option_add(f"*{str(self)[1:]}*TCombobox*Listbox.selectForeground", "#0b0d12")

    # -- map icons --
    # Drawn as canvas items, not child widgets: a widget placed over a canvas
    # is repainted along with it, and on Windows that shows up as the buttons
    # flashing every time the map redraws. Clicks and hovers are matched by
    # position in onMapPress / onMapMotion.
    ICON = 40
    ICON_GAP = 6
    ICON_MARGIN = 56        # dark margin kept clear each side of the map, for the icons
    ICON_BG, ICON_HOVER, ICON_ON = "#232834", "#323948", "#3d4d86"
    SIDE_W = 500            # width of the side panel

    def _buildMapButtons(self):
        self.icons = {}
        self._hoverIcon = None

        def icon(name, glyph, cmd, tip):
            rect = self.map.create_rectangle(0, 0, 1, 1, fill=self.ICON_BG, outline="",
                                             tags=("ui",))
            text = self.map.create_text(0, 0, text=glyph, fill="#e6e6e6",
                                        font=("Segoe UI Symbol", 15), tags=("ui",))
            self.icons[name] = dict(rect=rect, text=text, cmd=cmd, tip=tip, box=None, on=False)
        icon("panel", "☰", self.toggleSidebar, "Hide / show the side panel  (H)")
        icon("fns", "ƒ", self.toggleFunctions, "Show / hide the robot function badges  (F)")
        icon("pins", "⊙", self.togglePinEdit,
             "Edit pin locations  (P)  -- drag a pin, tap empty field to add one, right-click to delete")
        icon("draw", "✎", self.toggleDraw, "Draw a path  (D)")
        icon("apply", "✓", self.applyDrawnPath, "Apply the drawn path to the blocks  (Enter)")
        icon("clear", "✕", self.clearWaypoints, "Clear the points being drawn")
        icon("merge", "⇌", self.toggleMerge,
             "Merge points into one smooth curve  (M)  -- pick them, then tap again")
        icon("redo", "↷", self.redoPath, "Redo  (Ctrl+Y)")
        icon("undo", "↶", self.undoPath, "Undo  (Ctrl+Z)")

    def _placeIcons(self):
        """Lay the icons out in the margins either side of the square map:
        panel toggle top-left, draw tools down the right, undo / redo at the
        bottom right -- the arrangement path.jerryio uses."""
        w = self.map.winfo_width() or 600
        h = self.map.winfo_height() or MAP_PX
        s, g, m = self.ICON, self.ICON_GAP, 8
        right = w - m - s
        spots = {"panel": (m, m), "fns": (m, m + s + g), "pins": (m, m + 2 * (s + g)), "draw": (right, m), "apply": (right, m + s + g),
                 "clear": (right, m + 2 * (s + g)), "merge": (right, m + 3 * (s + g)),
                 "redo": (right, h - m - 2 * s - g),
                 "undo": (right, h - m - s)}
        for name, (x, y) in spots.items():
            ic = self.icons[name]
            ic["box"] = (x, y, x + s, y + s)
            self.map.coords(ic["rect"], x, y, x + s, y + s)
            self.map.coords(ic["text"], x + s / 2, y + s / 2)
        self.map.tag_raise("ui")

    def _iconAt(self, ex, ey):
        for name, ic in self.icons.items():
            x0, y0, x1, y1 = ic["box"] or (0, 0, -1, -1)
            if x0 <= ex <= x1 and y0 <= ey <= y1:
                return name
        return None

    def _paintIcon(self, name):
        ic = self.icons[name]
        fill = self.ICON_ON if ic["on"] else (self.ICON_HOVER if name == self._hoverIcon
                                              else self.ICON_BG)
        self.map.itemconfigure(ic["rect"], fill=fill)

    def _setIconActive(self, name, on):
        self.icons[name]["on"] = on
        self._paintIcon(name)

    def _hoverOver(self, name):
        if name == self._hoverIcon:
            return
        old, self._hoverIcon = self._hoverIcon, name
        for n in (old, name):
            if n:
                self._paintIcon(n)

    # -- menu bar: every command in one place, whatever the panel is showing --
    def _buildMenu(self):
        bar = tk.Menu(self, tearoff=0)
        kw = dict(tearoff=0, bg=PANEL_BG, fg=TEXT, activebackground=ACCENT,
                  activeforeground="#0b0d12", bd=0)

        def menu(title, items):
            m = tk.Menu(bar, **kw)
            for it in items:
                if it is None:
                    m.add_separator()
                else:
                    label, cmd, accel, *var = it
                    if var:
                        m.add_checkbutton(label=label, command=cmd, accelerator=accel,
                                          variable=var[0])
                    else:
                        m.add_command(label=label, command=cmd, accelerator=accel)
            bar.add_cascade(label=title, menu=m)
            return m

        menu("File", [("Save → autonomous.cpp", self.savePlan, "Ctrl+S"),
                      ("Load plan…", self.loadPlan, "Ctrl+O"),
                      ("Edit autonomous.cpp function…", self.editAuto, ""),
                      ("Export code", self.exportCode, "Ctrl+E"),
                      None,
                      ("Close", self.destroy, "")])
        menu("Edit", [("Undo", self.undoPath, "Ctrl+Z"), ("Redo", self.redoPath, "Ctrl+Y"),
                      None,
                      ("Move block up", lambda: self.moveBlock(-1), ""),
                      ("Move block down", lambda: self.moveBlock(1), ""),
                      ("Duplicate block", self.duplicateBlock, "Ctrl+D"),
                      ("Copy block", self.copyBlock, "Ctrl+C"),
                      ("Paste block", self.pasteBlock, "Ctrl+V"),
                      ("Delete block", self.deleteBlock, "Del"),
                      ("Clear all blocks", self.clearBlocks, ""),
                      None,
                      ("Set timeouts for all blocks", self.autoTimeoutAll, "")])
        menu("Path", [("Draw a path", self.toggleDraw, "D"),
                      ("Apply drawn path", self.applyAction, "Enter"),
                      ("Clear drawn points", self.clearWaypoints, ""),
                      ("Merge points", self.toggleMerge, "M"),
                      None,
                      ("Trace blocks into points", self.tracePointsFromBlocks, ""),
                      ("Clear map", self.clearMap, ""),
                      None,
                      ("Check plan for problems", self.checkPlan, ""),
                      None,
                      ("Flip to the other alliance (rotate 180°)",
                       lambda: self.flipPlan("rotate"), ""),
                      ("Mirror left-right", lambda: self.flipPlan("mirror"), "")])
        menu("Robot", [("Run all", self.runAll, "F5"), ("Run selected", self.runSelected, "F6"),
                       ("Stop", self.stop, "Esc"),
                       ("Last run report", self.showRunReport, "F7"),
                       ("Sample run report", self.showSampleReport, ""),
                       ("Open the report after each run", lambda: None, "", self.reportAutoVar),
                       None,
                       ("Pick start pose", self.togglePickStart, ""),
                       ("Send start pose to robot", self.sendStartPose, ""),
                       ("Use robot's pose", self.useRobotPose, ""),
                       ("Reset robot", self.resetRobot, ""),
                       ("Drive with WASD", self.toggleDrive, "")])
        menu("Help", [("Quick start and shortcuts", self.showHelp, "F1")])
        menu("View", [("Side panel", self.toggleSidebar, "H"),
                      ("Function badges", self.toggleFunctions, "F"),
                      ("Edit pin locations", self.togglePinEdit, "P"),
                      ("Reset pins", self.resetPins, ""),
                      ("Hide waitUntilDone rows", self.refreshList, "", self.hideGlueVar),
                      None,
                      ("Plan tab", lambda: self._showTab(0), ""),
                      ("Add tab", lambda: self._showTab(1), ""),
                      ("Robot tab", lambda: self._showTab(2), "")])
        self.config(menu=bar)

    def showHelp(self):
        win = tk.Toplevel(self)
        win.title("Quick start and shortcuts")
        win.configure(bg=FIELD_BG)
        win.geometry("620x560")
        txt = tk.Text(win, wrap="word", bg=PANEL_BG, fg=TEXT, relief="flat", padx=16, pady=12,
                      font=("Segoe UI", 10), spacing1=2, spacing3=2)
        txt.pack(fill="both", expand=True)
        txt.tag_configure("h", foreground=ACCENT, font=("Segoe UI", 10, "bold"), spacing1=10)
        txt.tag_configure("k", foreground="#9db4ff", font=("Consolas", 10, "bold"))
        for head, rows in (
                ("Building a route", (
                    ("D", "draw a path: click to add a curved point, right-click for a straight one"),
                    ("Enter", "turn the drawn path (or the block being placed) into blocks"),
                    ("M", "merge drawn points into one smooth curve"),
                    ("Esc", "cancel whatever is being placed, or stop the robot"),
                    ("Delete", "remove the selected block (or drawn point)"),
                    ("Ctrl+Z / Ctrl+Y", "undo / redo"),
                    ("Ctrl+C / Ctrl+V", "copy a block / paste it after the selected one"),
                    ("Ctrl+D", "duplicate the selected block"))),
                ("Blocks", (
                    ("Add tab", "movement blocks, Lift, Claw, Home, Grab pin and Shoot at goal"),
                    ("Plan tab", "the list, its parameters, the motion graph, the time estimate"),
                    ("click a row", "selects the block and shows it on the map"),
                    ("drag", "a route point, the robot at the start, or a heading knob"))),
                ("After a run on the robot", (
                    ("[1.2 s, 0.8 in off]", "how long the block took, and how far from the plan "
                                            "the robot ended"),
                    ("Time estimate", "learns from your whole-plan runs, so it gets closer"),
                    ("F7 / Robot > Last run report", "what the robot recorded: how each step "
                                                      "went, and how each Grab pin approached. "
                                                      "Copy it to send for diagnosis"))),
                ("Other side of the field", (
                    ("Path > Flip", "rotates the plan 180 degrees (the red goals become the blue "
                                    "ones) and lists any pin or goal that no longer lines up"),
                    ("Path > Mirror", "swaps left and right instead"))),
                ("Map", (
                    ("H", "hide / show the side panel"),
                    ("F", "show / hide the function badges"),
                    ("P", "edit pin locations (the path is hidden while you do)"),
                    ("View > Hide waitUntilDone rows", "shorter list; block numbers keep "
                                                       "their place"))),
                ("Robot", (
                    ("F5", "run the whole plan"), ("F6", "run the selected block"),
                    ("Ctrl+S", "save the plan and write it into autonomous.cpp"),
                    ("Ctrl+O / Ctrl+E", "load a plan / export the code"),
                    ("Run all", "first checks the plan (off the field, through a goal, over "
                                "30 s) and asks before running if it finds something"),
                    ("Autosave", "an unsaved plan is kept every 30 s and offered back if the "
                                 "planner closes unexpectedly"))),
                ("Colours in the block list", (
                    ("white", "drives, turns, waits"), ("blue", "what the robot does where it stops"),
                    ("grey", "waitUntilDone (the glue between moves)")))):
            txt.insert("end", head + "\n", "h")
            for key, what in rows:
                txt.insert("end", f"  {key}", "k")
                txt.insert("end", f"   {what}\n")
        txt.config(state="disabled")

    def _showTab(self, i):
        if not self.sidebarShown:
            self.toggleSidebar()
        self.tabs.select(i)

    # -- layout --
    def _buildUi(self):
        self.configure(bg=FIELD_BG)
        top = tk.Frame(self, bg=FIELD_BG)
        top.pack(side="top", fill="both", expand=True)

        # The side panel is packed first, so when the window is tight it is the
        # map that gives way, never a button; the map then takes what's left.
        self.side = tk.Frame(top, bg=FIELD_BG, width=self.SIDE_W)
        self.side.pack(side="right", fill="y", padx=(0, 8), pady=8)
        self.side.pack_propagate(False)
        self.mapFrame = tk.Frame(top, bg=FIELD_BG)
        self.mapFrame.pack(side="left", fill="both", expand=True, padx=8, pady=8)

        self._buildMapArea(self.mapFrame)
        self._buildSidePanel(self.side)
        self._buildMenu()
        self._loadFieldImage()

    def _buildMapArea(self, mapFrame):
        # The canvas asks for little and takes whatever the window gives it;
        # the square map is fitted inside it (see _relayout).
        self.map = tk.Canvas(mapFrame, width=320, height=320, bg=FIELD_BG, highlightthickness=0)
        self.map.pack(fill="both", expand=True)
        self._buildMapButtons()
        self.map.bind("<Configure>", self._onMapConfigure)
        self.map.bind("<Button-1>", self.onMapPress)
        self.map.bind("<B1-Motion>", self.onMapDrag)
        self.map.bind("<ButtonRelease-1>", self.onMapRelease)
        self.map.bind("<Button-3>", self.onMapRightClick)
        self.map.bind("<Shift-Button-1>", lambda e: self.removeWaypointAt(e.x, e.y))
        self.map.bind("<Button-2>", lambda e: self.removeWaypointAt(e.x, e.y))
        self.map.bind("<Motion>", self.onMapMotion)
        self.map.bind("<Leave>", self._onMapLeave)
        self.mapStatus = tk.Label(mapFrame, text="", anchor="w", fg=MUTED, bg=FIELD_BG,
                                  font=("Consolas", 9))
        self.mapStatus.pack(fill="x", pady=(4, 0))

        # Mouse-path tools under the map: everything the icons do, with labels,
        # plus the options that don't have an icon.
        row = tk.Frame(mapFrame, bg=FIELD_BG)
        row.pack(fill="x", pady=(6, 0))
        self.drawBtnRow = tk.Button(row, text="Draw  (D)", command=self.toggleDraw)
        self.drawBtnRow.pack(side="left")
        for text, cmd, accent in (("Apply  (Enter)", self.applyAction, PATH_GREEN),
                                  ("↶ Undo", self.undoPath, None), ("↷ Redo", self.redoPath, None),
                                  ("Clear points", self.clearWaypoints, None)):
            b = tk.Button(row, text=text, command=cmd)
            if accent:
                b.config(bg=accent, fg="#0b0d12", activebackground="#a3e05a",
                         activeforeground="#0b0d12")
            b.pack(side="left", padx=(6, 0))
        self.mergeBtn = tk.Button(row, text="Merge  (M)", command=self.toggleMerge)
        self.mergeBtn.pack(side="left", padx=(6, 0))
        row2 = tk.Frame(mapFrame, bg=FIELD_BG)
        row2.pack(fill="x", pady=(6, 0))
        self.smoothVar = tk.BooleanVar(value=True)
        tk.Checkbutton(row2, text="smooth", variable=self.smoothVar, bg=FIELD_BG, fg="#ddd",
                       selectcolor=INPUT_BG, activebackground=FIELD_BG,
                       activeforeground="#ddd", command=self.redrawMap).pack(side="left")
        tk.Label(row2, text="spacing", fg="#888", bg=FIELD_BG).pack(side="left", padx=(8, 0))
        self.spacingVar = tk.StringVar(value="12")
        e = tk.Entry(row2, textvariable=self.spacingVar, width=5, font=("Consolas", 10))
        e.pack(side="left", padx=(2, 8))
        e.bind("<Return>", lambda _e: self.redrawMap())
        e.bind("<FocusOut>", lambda _e: self.redrawMap())
        self.reverseVar = tk.BooleanVar(value=False)
        tk.Checkbutton(row2, text="backwards", variable=self.reverseVar, bg=FIELD_BG, fg="#ddd",
                       selectcolor=INPUT_BG, activebackground=FIELD_BG,
                       activeforeground="#ddd").pack(side="left")
        tk.Button(row2, text="Trace blocks", command=self.tracePointsFromBlocks) \
            .pack(side="left", padx=(12, 0))
        tk.Button(row2, text="Clear map", command=self.clearMap).pack(side="left", padx=(6, 0))
        self.drawInfo = tk.Label(mapFrame, text="", fg="#777", bg=FIELD_BG, font=("Segoe UI", 8),
                                 anchor="w")
        self.drawInfo.pack(fill="x", pady=(4, 0))

    def _buildSidePanel(self, side):
        # header: brain status and the hide button -- always on screen
        hdr = tk.Frame(side, bg=FIELD_BG)
        hdr.pack(side="top", fill="x", pady=(0, 8))
        tk.Button(hdr, text="Hide panel  (H) ▸", command=self.toggleSidebar) \
            .pack(side="right")
        self.brainStatus = tk.Label(hdr, text="brain: waiting...", anchor="w", fg="#00F0FF",
                                    bg=FIELD_BG, font=("Segoe UI", 10, "bold"))
        from reconnect_button import add_reconnect_button
        add_reconnect_button(self, hdr, status=self.brainStatus, side="right", padx=(0, 6))
        self.brainStatus.pack(side="left", fill="x", expand=True)

        # footer: run / export / save -- packed before the tabs so it always keeps its space,
        # however short the window is
        foot = tk.Frame(side, bg=FIELD_BG)
        foot.pack(side="bottom", fill="x", pady=(8, 0))
        f1 = tk.Frame(foot, bg=FIELD_BG)
        f1.pack(fill="x")
        for text, cmd, color in (("▶  Run all  (F5)", self.runAll, "#2ECC71"),
                                 ("▶  Run selected  (F6)", self.runSelected, "#4fc3f7"),
                                 ("■  Stop  (Esc)", self.stop, "#FF6B81")):
            tk.Button(f1, text=text, command=cmd, fg="#0b0d12", bg=color, pady=7,
                      font=("Segoe UI", 9, "bold"), activebackground=color,
                      activeforeground="#0b0d12") \
                .pack(side="left", fill="x", expand=True, padx=(0, 4))
        f2 = tk.Frame(foot, bg=FIELD_BG)
        f2.pack(fill="x", pady=(6, 0))
        tk.Button(f2, text="Save → autonomous.cpp  (Ctrl+S)", command=self.savePlan, bg=ACCENT,
                  fg="#0b0d12", activebackground=ACCENT_HI, activeforeground="#0b0d12") \
            .pack(side="left", fill="x", expand=True, padx=(0, 4))
        tk.Button(f2, text="Load", command=self.loadPlan).pack(side="left", padx=(0, 4))
        tk.Button(f2, text="Edit auto…", command=self.editAuto).pack(side="left", padx=(0, 4))
        tk.Button(f2, text="Export code", command=self.exportCode).pack(side="left")
        f3 = tk.Frame(foot, bg=FIELD_BG)
        f3.pack(fill="x", pady=(6, 0))
        self.calibrateVar = tk.BooleanVar(value=True)
        tk.Checkbutton(f3, text="calibrate first (export)", variable=self.calibrateVar,
                       bg=FIELD_BG, activebackground=FIELD_BG).pack(side="left")
        self.setStartVar = tk.BooleanVar(value=True)
        tk.Checkbutton(f3, text="Run all starts from the start pose", variable=self.setStartVar,
                       bg=FIELD_BG, activebackground=FIELD_BG).pack(side="left", padx=(10, 0))

        # three tabs, each scrolling on its own: what is in the plan, what can be added to it,
        # and the robot itself
        nb = ttk.Notebook(side)
        nb.pack(side="top", fill="both", expand=True)
        self.tabs = nb
        self._tabCanvases = []

        def tab(title):
            page = tk.Frame(nb, bg=PANEL_BG)
            nb.add(page, text=title)
            sc = tk.Canvas(page, bg=PANEL_BG, highlightthickness=0)
            vsb = tk.Scrollbar(page, command=sc.yview, width=10)
            sc.configure(yscrollcommand=vsb.set)
            vsb.pack(side="right", fill="y")
            sc.pack(side="left", fill="both", expand=True)
            inner = tk.Frame(sc, bg=PANEL_BG)
            win = sc.create_window(0, 0, window=inner, anchor="nw")
            inner.bind("<Configure>", lambda e, c=sc: c.configure(scrollregion=c.bbox("all")))
            sc.bind("<Configure>", lambda e, c=sc, w=win: c.itemconfigure(w, width=e.width))
            self._tabCanvases.append(sc)
            return inner
        self._sideCanvas = None

        def wheel(e):
            i = nb.index(nb.select())
            self._tabCanvases[i].yview_scroll(-1 if e.delta > 0 else 1, "units")
        saved = {}

        def enter(_e):
            saved["old"] = self.bind_all("<MouseWheel>")
            self.bind_all("<MouseWheel>", wheel)

        def leave(_e):
            self.unbind_all("<MouseWheel>")
            if saved.get("old"):
                self.bind_all("<MouseWheel>", saved["old"])
        side.bind("<Enter>", enter)
        side.bind("<Leave>", leave)

        def section(parent, title, pad=(0, 10)):
            """A card: hairline border, small accent title."""
            f = tk.Frame(parent, bg=PANEL_BG, highlightthickness=1,
                         highlightbackground=CARD_BORDER)
            f.pack(fill="x", padx=8, pady=pad)
            tk.Label(f, text=title.upper(), anchor="w", fg=ACCENT, bg=PANEL_BG,
                     font=("Segoe UI", 8, "bold")).pack(fill="x", padx=10, pady=(8, 0))
            return f

        def row(parent, pady=(4, 4)):
            r = tk.Frame(parent, bg=PANEL_BG)
            r.pack(fill="x", padx=10, pady=pady)
            return r

        def labelled(parent, label, var, width=7, first=False):
            tk.Label(parent, text=label, fg=MUTED, bg=PANEL_BG) \
                .pack(side="left", padx=(0 if first else 8, 3))
            e = tk.Entry(parent, textvariable=var, width=width, font=("Consolas", 10))
            e.pack(side="left", ipady=2)
            return e

        def hint(parent, text):
            tk.Label(parent, text=text, anchor="w", justify="left", wraplength=440, fg=MUTED,
                     bg=PANEL_BG, font=("Segoe UI", 8)).pack(fill="x", padx=10, pady=(0, 8))

        # ═════════ tab 1: PLAN -- the blocks, their parameters and their motion graph ═════════
        plan = tab("Plan")
        est = tk.Frame(plan, bg=PANEL_BG, highlightthickness=1, highlightbackground=CARD_BORDER)
        est.pack(fill="x", padx=8, pady=(8, 0))
        tk.Label(est, text="ESTIMATED TIME", anchor="w", fg=ACCENT, bg=PANEL_BG,
                 font=("Segoe UI", 8, "bold")).pack(fill="x", padx=10, pady=(8, 0))
        self.timeLabel = tk.Label(est, text="--", anchor="w", fg=TEXT, bg=PANEL_BG,
                                  font=("Segoe UI", 16, "bold"))
        self.timeLabel.pack(fill="x", padx=10)
        self.timeDetail = tk.Label(est, text="", anchor="w", justify="left", fg=MUTED,
                                   bg=PANEL_BG, font=("Segoe UI", 8), wraplength=450)
        self.timeDetail.pack(fill="x", padx=10, pady=(0, 4))
        tk.Frame(est, bg=CARD_BORDER, height=1).pack(fill="x", padx=10)
        tk.Label(est, text="LAST RUN ON THE ROBOT", anchor="w", fg=ACCENT, bg=PANEL_BG,
                 font=("Segoe UI", 8, "bold")).pack(fill="x", padx=10, pady=(6, 0))
        self.actualLabel = tk.Label(est, text="--", anchor="w", fg=TEXT, bg=PANEL_BG,
                                    font=("Segoe UI", 16, "bold"))
        self.actualLabel.pack(fill="x", padx=10)
        self.actualDetail = tk.Label(est, text="run the plan from here and its real time shows "
                                     "up when it finishes", anchor="w", justify="left", fg=MUTED,
                                     bg=PANEL_BG, font=("Segoe UI", 8), wraplength=450)
        self.actualDetail.pack(fill="x", padx=10, pady=(0, 8))
        self.guide = tk.Frame(plan, bg=PANEL_BG, highlightthickness=1, highlightbackground=ACCENT)
        tk.Label(self.guide, text="GETTING STARTED", anchor="w", fg=ACCENT, bg=PANEL_BG,
                 font=("Segoe UI", 8, "bold")).pack(fill="x", padx=10, pady=(8, 2))
        for n, (title, how) in enumerate((
                ("Place the robot", "Robot tab: type the start pose, or Pick start and tap the map."),
                ("Build the route", "Press D and click points on the map, then Enter. Or use the "
                                    "Add tab for turns, lifts, claws, Grab pin and Shoot at goal."),
                ("Check it", "The estimated time is above; click a block to see it on the map."),
                ("Run it", "Run all (F5) sends it to the robot. Save writes it into autonomous.cpp."))):
            r = tk.Frame(self.guide, bg=PANEL_BG)
            r.pack(fill="x", padx=10, pady=2)
            tk.Label(r, text=str(n + 1), fg="#0b0d12", bg=ACCENT, width=2,
                     font=("Segoe UI", 9, "bold")).pack(side="left", anchor="n")
            tk.Label(r, text=f"{title}  --  {how}", anchor="w", justify="left", wraplength=410,
                     fg="#c9cedb", bg=PANEL_BG, font=("Segoe UI", 9)).pack(side="left", padx=8)
        tk.Label(self.guide, text="Press F1 for every shortcut.", anchor="w", fg=MUTED,
                 bg=PANEL_BG, font=("Segoe UI", 8)).pack(fill="x", padx=10, pady=(2, 8))
        self._timeKey = None
        self._runT0 = None             # when the robot's plan went RUNNING (laptop clock)
        self._runWhich = None          # what was started from here: None = not from here, "ALL", i
        self.runHistory = []           # real times of the runs that completed, newest last
        self._idleSince = None         # when the plan status left RUNNING for IDLE (see applyState)
        self._stepT0 = None            # (robot step, when it began): times each block of a run
        self._stepTimes = {}           # robot step -> seconds it took, this run
        self._stepPoses = {}           # robot step -> the robot's pose when it ended
        self._blockDriftData = {}      # block -> inches the robot ended from the plan, last run
        self._blockTimesData, self._blockTimesFor = {}, None
        self.blocksCard = bl = section(plan, "blocks", pad=(8, 10))
        lf = tk.Frame(bl, bg=PANEL_BG)
        lf.pack(fill="x", padx=10, pady=(4, 6))
        self.listbox = tk.Listbox(lf, font=("Consolas", 10), bg=INPUT_BG, fg=TEXT,
                                  selectbackground="#2f3f78", selectforeground="#ffffff",
                                  activestyle="none", height=9, exportselection=False)
        self.listbox.pack(side="left", fill="both", expand=True)
        self.listbox.bind("<<ListboxSelect>>", self.onSelect)

        def listWheel(e):
            # over the blocks list the wheel scrolls only the list -- "break"
            # keeps it from also reaching the panel's own wheel binding
            self.listbox.yview_scroll(-1 if e.delta > 0 else 1, "units")
            return "break"
        self.listbox.bind("<MouseWheel>", listWheel)
        sb = tk.Scrollbar(lf, command=self.listbox.yview, width=10)
        sb.pack(side="left", fill="y")
        self.listbox.config(yscrollcommand=sb.set)
        ops = tk.Frame(lf, bg=PANEL_BG)
        ops.pack(side="left", fill="y", padx=(8, 0))
        for text, cmd in (("▲  Up", lambda: self.moveBlock(-1)),
                          ("▼  Down", lambda: self.moveBlock(1)),
                          ("⧉  Duplicate", self.duplicateBlock),
                          ("✕  Delete", self.deleteBlock),
                          ("Clear all", self.clearBlocks)):
            tk.Button(ops, text=text, width=11, anchor="w", command=cmd).pack(pady=2)

        # parameters of the selected block
        self.paramFrame = tk.LabelFrame(plan, text=" parameters ", fg=ACCENT, bg=PANEL_BG,
                                        font=("Segoe UI", 9, "bold"), bd=0, relief="flat",
                                        highlightthickness=1, highlightbackground=CARD_BORDER,
                                        labelanchor="nw")
        self.paramFrame.pack(fill="x", padx=8, pady=(0, 4))
        self.paramWidgets = {}
        self.codeLabel = tk.Label(plan, text="", anchor="w", justify="left", fg="#2ECC71",
                                  bg=PANEL_BG, font=("Consolas", 9), wraplength=450)
        self.codeLabel.pack(fill="x", padx=10, pady=(0, 10))

        # motion graph: the simulated power / speed / error of the selected motion
        gr = section(plan, "motion graph", pad=(0, 8))
        self.graphCanvas = tk.Canvas(gr, height=200, bg=INPUT_BG, highlightthickness=0)
        self.graphCanvas.pack(fill="x", padx=10, pady=(6, 2))
        self.graphCanvas.bind("<Configure>", lambda e: self.updateMotionGraph(force=True))
        self.graphInfo = tk.Label(gr, text="", anchor="w", justify="left", fg=MUTED,
                                  bg=PANEL_BG, font=("Consolas", 9), wraplength=450)
        self.graphInfo.pack(fill="x", padx=10, pady=(0, 8))
        self._graphKey = None

        # ═════════ tab 2: ADD -- every block the robot understands ═════════
        add = tab("Add")
        mv = section(add, "movement", pad=(8, 10))
        grid = tk.Frame(mv, bg=PANEL_BG)
        grid.pack(fill="x", padx=10, pady=(6, 10))
        for i, kind in enumerate(k for k in KINDS if k not in FUNCTIONS and k != "goToPin"):
            tk.Button(grid, text=kind,
                      command=lambda k=kind: (self.pickMotion(k), self.tabs.select(0))) \
                .grid(row=i // 3, column=i % 3, padx=2, pady=2, sticky="ew")
        for c in range(3):
            grid.columnconfigure(c, weight=1, uniform="kinds")

        # robot functions: what the robot does at the spot it has reached
        fn = section(add, "robot functions")
        fgrid = tk.Frame(fn, bg=PANEL_BG)
        fgrid.pack(fill="x", padx=10, pady=(6, 2))
        # goToPin is placed by tapping the map (pickMotion), so it is a card here but not a badge
        # (the plain Shoot and Auto-grab cards are gone: Shoot at goal does the shooting and Grab pin the
        # IR grabbing; their blocks still load from old plans)
        for i, kind in enumerate(["liftToLevel", "claw", "homeLift", "goToPin", "shootAtGoal"]):
            self._functionButton(fgrid, kind).grid(row=i // 3, column=i % 3, padx=3, pady=3,
                                                   sticky="nsew")
        for c in range(3):
            fgrid.columnconfigure(c, weight=1, uniform="funcs")
        hint(fn, "Added after the selected block and shown on the map as a badge where the "
                 "robot is when it runs. liftToLevel uses the angles from LIFT CAL. Grab pin "
                 "is placed by tapping a pin on the map, Shoot at goal by tapping a goal.")

        pn = section(add, "pins")
        prow = row(pn, pady=(6, 4))
        self.pinEditBtn = tk.Button(prow, text="Edit pin locations  (P)",
                                    command=self.togglePinEdit)
        self.pinEditBtn.pack(side="left")
        tk.Button(prow, text="Reset pins", command=self.resetPins).pack(side="left", padx=6)
        hint(pn, "Drag a pin to move it, tap empty field to add one, right-click to delete. "
                 "Grab-pin blocks on a moved pin follow it. Saved in tools/pins.json.")

        sm = section(add, "smart planning")
        self.avoidVar = tk.BooleanVar(value=True)
        tk.Checkbutton(sm, text="Avoid goals  (Grab pin and Shoot at goal route round them)",
                       variable=self.avoidVar, command=self.redrawMap, anchor="w") \
            .pack(fill="x", padx=10, pady=(6, 0))
        self.autoTimeoutVar = tk.BooleanVar(value=True)
        tk.Checkbutton(sm, text="Pick timeouts automatically  (simulated time + margin)",
                       variable=self.autoTimeoutVar, anchor="w").pack(fill="x", padx=10)
        arow = row(sm, pady=(4, 4))
        tk.Button(arow, text="Set timeouts for all blocks", command=self.autoTimeoutAll) \
            .pack(side="left")
        hint(sm, "Grab pin and the movement blocks get a timeout from how long the simulated robot "
                 "takes to finish them, plus 35% and 300 ms to spare. Editing a block's "
                 "x / y / heading / speed refreshes it; typing a timeout yourself keeps yours.")

        # ═════════ tab 3: ROBOT -- where it starts, its size, the selected point, driving ═════════
        rob = tab("Robot")
        rb = section(rob, "start pose", pad=(8, 10))
        r1 = row(rb, pady=(6, 4))
        self.startVars = []
        for i, lab in enumerate(("x", "y", "hdg")):
            v = tk.StringVar(value="0")
            e = labelled(r1, lab, v, 6, first=(i == 0))
            e.bind("<Return>", lambda _e: self.applyStartPose())
            self.startVars.append(v)
        tk.Button(r1, text="Send to robot", command=self.sendStartPose).pack(side="right")
        r2 = row(rb, pady=(2, 8))
        self.pickBtn = tk.Button(r2, text="Pick start", command=self.togglePickStart)
        self.pickBtn.pack(side="left")
        tk.Button(r2, text="Use robot's pose", command=self.useRobotPose) \
            .pack(side="left", padx=6)
        tk.Button(r2, text="Reset robot", command=self.resetRobot, fg="#FF6B81") \
            .pack(side="left")

        sz = section(rob, "robot size")
        # Footprint: drawn as a bounding box at the start, the end of the route, the selected
        # block and the live robot. length runs along the heading (front to back), width across.
        r3 = row(sz, pady=(6, 8))
        self.robotLenVar = tk.StringVar(value=fmt(DEFAULT_ROBOT_IN))
        self.robotWidVar = tk.StringVar(value=fmt(DEFAULT_ROBOT_IN))
        for i, (lab, var) in enumerate((("length (in)", self.robotLenVar),
                                        ("width (in)", self.robotWidVar))):
            e = labelled(r3, lab, var, 6, first=(i == 0))
            e.bind("<Return>", lambda _e: self.redrawMap())
            e.bind("<FocusOut>", lambda _e: self.redrawMap())

        r3b = row(sz, pady=(0, 4))
        self.frontEdgeVar, self.backEdgeVar = tk.StringVar(value=""), tk.StringVar(value="")
        for i, (lab, var) in enumerate((("front edge (in)", self.frontEdgeVar),
                                        ("back edge (in)", self.backEdgeVar))):
            e = labelled(r3b, lab, var, 6, first=(i == 0))
            e.bind("<Return>", lambda _e: self.redrawMap())
            e.bind("<FocusOut>", lambda _e: self.redrawMap())
        hint(sz, "Front / back edge: measure from the robot's odometry centre (the tracking "
                 "point, not necessarily the middle) to the edge that touches a wall or goal. "
                 "Blank = half the length. Shoot at goal (how far it drives in) and Grab pin "
                 "(how far its claw reaches) use these.")

        # selected route point, like path.jerryio's "Control" panel
        pt = section(rob, "selected point")
        r4 = row(pt, pady=(6, 8))
        self.ptVars = []
        for i, lab in enumerate(("x", "y", "heading")):
            v = tk.StringVar()
            e = labelled(r4, lab, v, 7 if lab == "heading" else 6, first=(i == 0))
            e.bind("<Return>", lambda _e: self.applyPointPanel())
            e.bind("<FocusOut>", lambda _e: self.applyPointPanel())
            self.ptVars.append(v)
        for text, delta in (("↻", self.HEADING_STEP), ("↺", -self.HEADING_STEP)):
            tk.Button(r4, text=text, width=3, font=("Segoe UI Symbol", 11),
                      command=lambda d=delta: self.rotatePoint(d)).pack(side="right", padx=(4, 0))

        dr = section(rob, "drive")
        drow = row(dr, pady=(6, 8))
        self.driveBtn = tk.Button(drow, text="Drive (WASD)", command=self.toggleDrive)
        self.driveBtn.pack(side="left")
        tk.Label(drow, text="power", fg=MUTED, bg=PANEL_BG).pack(side="left", padx=(10, 2))
        self.powerVar = tk.IntVar(value=70)
        tk.Scale(drow, from_=10, to=127, orient="horizontal", variable=self.powerVar,
                 length=110, bg=PANEL_BG, fg=TEXT, highlightthickness=0, showvalue=True,
                 troughcolor=INPUT_BG, activebackground=ACCENT, bd=0).pack(side="left")
        self.driveStatus = tk.Label(drow, text="off", fg=MUTED, bg=PANEL_BG,
                                    font=("Consolas", 9))
        self.driveStatus.pack(side="left", padx=8)

        self.hint = tk.Label(rob, text="Drag the robot to place it, any route point to move "
                             "it, and the knob on a point's line to turn it.  D = draw a path "
                             "(click to add points, Enter to apply).  H = hide this panel.  "
                             "Ctrl+Z / Ctrl+Y undo / redo.  Delete = remove block.  Ctrl+S saves "
                             "the plan and writes it into autonomous.cpp.  Ctrl+O load.  "
                             "Ctrl+E export.", anchor="w", justify="left", wraplength=450,
                             fg=MUTED, bg=PANEL_BG, font=("Segoe UI", 8))
        self.hint.pack(fill="x", padx=10, pady=(0, 8))

    FUNCTION_LABELS = {"liftToLevel": ("Lift", "to level"), "claw": ("Claw", "open / close"),
                       "homeLift": ("Home", "lift"), "shootToGoal": ("Shoot", "to goal"),
                       "autoGrabPin": ("Auto-grab", "IR closes claws"),
                       "shootAtGoal": ("Shoot at goal", "tap goal + side"),
                       "goToPin": ("Grab pin", "go + IR grab")}

    def _functionButton(self, parent, kind):
        """A robot-function button: a dark card with a title and a caption that
        lights up with the accent colour on hover and press."""
        card_bg, hover_bg, press_bg = BTN_BG, BTN_HOVER, "#3d4d86"
        card = tk.Frame(parent, bg=card_bg, cursor="hand2", highlightthickness=1,
                        highlightbackground=CARD_BORDER)
        title, sub = self.FUNCTION_LABELS[kind]
        name = tk.Label(card, text=title, bg=card_bg, fg=TEXT, cursor="hand2",
                        font=("Segoe UI", 10, "bold"))
        name.pack(fill="x", padx=8, pady=(7, 0))
        small = tk.Label(card, text=sub, bg=card_bg, fg=MUTED, cursor="hand2",
                         font=("Segoe UI", 8))
        small.pack(fill="x", padx=8, pady=(0, 7))
        parts = (card, name, small)

        def paint(bg, edge=CARD_BORDER):
            for w in parts:
                w.config(bg=bg)
            card.config(highlightbackground=edge)
        for w in parts:
            w.bind("<Enter>", lambda _e: paint(hover_bg, ACCENT))
            w.bind("<Leave>", lambda _e: paint(card_bg))
            w.bind("<ButtonPress-1>", lambda _e: paint(press_bg, ACCENT))
            w.bind("<ButtonRelease-1>",
                   lambda _e, k=kind: (paint(hover_bg, ACCENT), self.pickMotion(k),
                                       self.tabs.select(0)))   # its parameters are on the Plan tab
        return card

    def toggleSidebar(self):
        """Hide the whole right-hand panel so only the map shows (and bring it
        back). The map grows to fill the window when the panel goes."""
        self.sidebarShown = not self.sidebarShown
        if self.sidebarShown:
            self.side.pack(side="right", fill="y", padx=(0, 8), pady=8, before=self.mapFrame)
        else:
            self.side.pack_forget()
        self._setIconActive("panel", not self.sidebarShown)

    def _bindKeys(self):
        self.bind("<F1>", lambda e: self.showHelp())
        self.bind("<F5>", lambda e: self.runAll())
        self.bind("<F6>", lambda e: self.runSelected())
        self.bind("<F7>", lambda e: self.showRunReport())
        self.bind("<Escape>", lambda e: self.cancelPending() if self.pending
                  else self.cancelMerge() if self.mergeMode
                  else (self.stopDrive(), self.stop()))
        self.bind("<m>", lambda e: None if self._typing() or self.driveMode else self.toggleMerge())
        self.bind("<M>", lambda e: None if self._typing() or self.driveMode else self.toggleMerge())
        self.bind("<Control-c>", lambda e: None if self._typing() else self.copyBlock())
        self.bind("<Control-v>", lambda e: None if self._typing() else self.pasteBlock())
        self.bind("<Control-d>", lambda e: None if self._typing() else self.duplicateBlock())
        self.bind("<Control-s>", lambda e: self.savePlan())
        self.bind("<Control-o>", lambda e: self.loadPlan())
        self.bind("<Control-e>", lambda e: self.exportCode())
        self.bind("<Control-r>", lambda e: getattr(self, "onReconnect", None) and self.onReconnect())
        self.listbox.bind("<Delete>", lambda e: self.deleteBlock())
        self.bind("<Delete>", lambda e: None if self._typing() else
                  self.deleteSelectedWaypoint() if self.drawMode else self.deleteBlock())
        # ...and the draw-path keys, which must not fire while a number is
        # being typed into one of the entries (Tk sends the key to the
        # widget first, then here).
        # W/A/S/D drive the robot while drive mode is on; "d" only toggles
        # the path editor when it isn't (it would otherwise do both).
        self.bind("<KeyPress>", self.onDriveKeyPress, add="+")
        self.bind("<KeyRelease>", self.onDriveKeyRelease, add="+")
        self.bind("<FocusOut>", lambda e: self.stopDrive() if self.driveMode else None)
        self.bind("<d>", lambda e: None if self._typing() or self.driveMode else self.toggleDraw())
        self.bind("<D>", lambda e: None if self._typing() or self.driveMode else self.toggleDraw())
        self.bind("<f>", lambda e: None if self._typing() or self.driveMode else self.toggleFunctions())
        self.bind("<F>", lambda e: None if self._typing() or self.driveMode else self.toggleFunctions())
        self.bind("<p>", lambda e: None if self._typing() or self.driveMode else self.togglePinEdit())
        self.bind("<P>", lambda e: None if self._typing() or self.driveMode else self.togglePinEdit())
        self.bind("<h>", lambda e: None if self._typing() else self.toggleSidebar())
        self.bind("<H>", lambda e: None if self._typing() else self.toggleSidebar())
        self.bind("<Control-z>", lambda e: None if self._typing() else self.undoPath())
        self.bind("<Control-y>", lambda e: None if self._typing() else self.redoPath())
        self.bind("<Control-Z>", lambda e: None if self._typing() else self.redoPath())
        self.bind("<BackSpace>", lambda e: None if self._typing() else self.undoWaypoint())
        self.bind("<Return>", lambda e: self.applyAction()
                  if (self.drawMode or self.pending) and not self._typing() else None)

    def _typing(self):
        """True when the keyboard focus is in a text entry / combobox."""
        w = self.focus_get()
        return isinstance(w, (tk.Entry, tk.Text)) or isinstance(w, ttk.Combobox) or \
            isinstance(w, ttk.Entry)

    def _loadFieldImage(self):
        """tools/field.png, scaled to the map. With Pillow it is resized to
        whatever size the map has (see _relayout); without it Tk can only
        subsample it, so the map stays at its fixed size."""
        if not os.path.exists(FIELD_PNG):
            return
        try:
            from PIL import Image
            self._fieldSrc = Image.open(FIELD_PNG).convert("RGB")
            self._scaleField()
        except ImportError:
            try:
                img = tk.PhotoImage(file=FIELD_PNG)
                f = max(1, round(img.width() / MAP_PX))
                self.fieldImage = img.subsample(f, f)
            except tk.TclError:
                self.fieldImage = None
        except Exception:
            self._fieldSrc = None
            self.fieldImage = None

    def _scaleField(self):
        if self._fieldSrc is None:
            return
        from PIL import Image, ImageTk
        self.fieldImage = ImageTk.PhotoImage(
            self._fieldSrc.resize((self.mapPx, self.mapPx), Image.LANCZOS))

    def _onMapConfigure(self, _e):
        """The canvas changed size (window resized, panel hidden / shown):
        refit the map once things settle, not on every step of a drag."""
        if self._relayoutJob:
            self.after_cancel(self._relayoutJob)
        self._relayoutJob = self.after(60, self._relayout)

    def _relayout(self):
        """Fit the square map in the middle of the canvas, as large as the
        window allows while keeping a margin each side for the icons."""
        self._relayoutJob = None
        w, h = self.map.winfo_width(), self.map.winfo_height()
        if w < 60 or h < 60:
            return
        side = MAP_PX if self._fieldSrc is None and self.fieldImage else \
            int(max(220, min(h, w - 2 * self.ICON_MARGIN)))
        x0, y0 = (w - side) // 2, (h - side) // 2
        changed = (side, x0, y0) != (self.mapPx, self.mapX0, self.mapY0)
        self.mapPx, self.mapX0, self.mapY0 = side, x0, y0
        self._placeIcons()
        if changed:
            self._scaleField()
            self.map.delete("field")
            self.redrawMap()

    # -- coordinates --
    @property
    def ppi(self):
        """Map pixels per field inch."""
        return self.mapPx / FIELD_IN

    def toMap(self, x, y):
        return (self.mapX0 + self.mapPx / 2 + x * self.ppi,
                self.mapY0 + self.mapPx / 2 - y * self.ppi)

    def toField(self, px, py):
        return ((px - self.mapX0 - self.mapPx / 2) / self.ppi,
                (self.mapY0 + self.mapPx / 2 - py) / self.ppi)

    # -- map drawing --
    def simulation(self):
        """simulate() for the current blocks, cached. The map is redrawn
        on every state the robot sends (20 a second), and re-running the
        whole motion simulation each time is what made it crawl and
        flicker -- the route only changes when the blocks or the start
        pose do."""
        if self._frameSim is not None:      # mid-paint: nothing changes until it ends
            return self._frameSim
        self._syncCtx()
        key = (repr(self.blocks), tuple(self.start), repr(PLAN_CTX))
        if self._simCache[0] != key:
            self._simCache = (key, simulate(self.blocks, tuple(self.start)))
        return self._simCache[1]

    def _edge(self, var):
        """Inches from the robot's odometry centre to its front / back edge. Blank = half the
        length. It is a measured number: the tracking centre is rarely the middle of the robot."""
        try:
            v = float(var.get())
            if v > 0:
                return v
        except (ValueError, tk.TclError):
            pass
        return self.robotSize()[0] / 2.0

    def frontEdge(self):
        return self._edge(self.frontEdgeVar)

    def backEdge(self):
        return self._edge(self.backEdgeVar)

    def _syncCtx(self):
        """What expanding a Shoot at goal needs from this window (see PLAN_CTX)."""
        PLAN_CTX.update(size=tuple(self.robotSize()), avoid=bool(self.avoidVar.get()),
                        exit=round(self.spacing() / 2.0, 2), auto=bool(self.autoTimeoutVar.get()),
                        front=self.frontEdge(), back=self.backEdge())

    def flatPlan(self):
        """(blocks, spans) as the robot runs them: Shoot at goal blocks expanded."""
        self._syncCtx()
        return expand_plan(self.blocks, tuple(self.start))

    def redrawMap(self):
        """Ask for a redraw. Several calls in one go (editing a block
        refreshes the list, the params and the map) collapse into a single
        draw at idle, so the map doesn't flicker or fall behind."""
        if self._redrawPending:
            return
        self._redrawPending = True
        self.after_idle(self._drawAndGraph)

    def updateTimeEstimate(self):
        """The estimated run time of the plan, on the Plan tab."""
        self._syncCtx()
        key = (repr(self.blocks), tuple(self.start), repr(PLAN_CTX))
        if key == self._timeKey:
            return
        self._timeKey = key
        if not self.blocks:
            self.timeLabel.config(text="--", fg=TEXT)
            self.timeDetail.config(text="add blocks to get an estimate")
            return
        total, driving, other = estimate_seconds(self.flatPlan()[0], self.start)
        left = AUTO_PERIOD_S - total
        self.timeLabel.config(text=f"~ {total:.1f} s",
                              fg="#2ECC71" if left >= 0 else "#FF6B81")
        self.timeDetail.config(
            text=f"driving and turning {driving:.1f} s  +  lift / claws / waits {other:.1f} s.  "
                 + (f"{left:.1f} s to spare in a {AUTO_PERIOD_S:.0f} s autonomous."
                    if left >= 0 else f"{-left:.1f} s over a {AUTO_PERIOD_S:.0f} s autonomous."))

    def _drawAndGraph(self):
        self._drawMap()
        try:
            self.updateTimeEstimate()
        except Exception as e:      # the estimate must never break the map
            self.timeDetail.config(text=f"estimate error: {e}")
        try:
            self.updateMotionGraph()
        except Exception as e:      # the graph must never break the map
            self.graphInfo.config(text=f"graph error: {e}")

    # -- motion graph --
    GRAPH_KINDS = ("moveToPoint", "goToPin")

    def updateMotionGraph(self, force=False):
        """The speed profile of the move being edited (the selected block, or the one still being
        placed), drawn like the DECEL TUNE window: speed against inches driven, with the point
        where the slow-down starts. moveToPoint and goToPin only. Tune maxSpeed, decelRange,
        decelSpeed and decelSteps in the parameters above and it redraws."""
        c = self.graphCanvas
        if self.pending:
            b, at = self.pending
        elif self.selected is not None and self.selected < len(self.blocks):
            b, at = self.blocks[self.selected], self.selected
        else:
            b = at = None
        if b is None or b["kind"] not in self.GRAPH_KINDS:
            key = ("none", c.winfo_width())
            if force or key != self._graphKey:
                self._graphKey = key
                c.delete("all")
                c.create_text(10, 12, anchor="w", fill="#777", font=("Segoe UI", 9),
                              text="select a moveToPoint or goToPin block")
                self.graphInfo.config(text="")
            return
        x0, y0, _th = self._poseAt(at)
        key = (repr(b), round(x0, 2), round(y0, 2), c.winfo_width(), c.winfo_height())
        if not force and key == self._graphKey:
            return
        self._graphKey = key
        dist = math.hypot(b["x"] - x0, b["y"] - y0)
        if b["kind"] == "goToPin" and b.get("forwards", True) and b.get("reach", 0) > 0:
            dist = max(dist - b["reach"], 0.0)      # it stops `reach` short: the claw tip lands on the pin
        self._paintGraph(b, dist)

    @staticmethod
    def speedProfile(dist, speed, rng, end, steps):
        """[(inches driven, speed)]: full speed, then the slow-down over the last `rng` inches
        (one step to `end`, or a ramp of `steps` stages). Same shape as tools/decel_tune.py."""
        if rng <= 0 or end >= speed or dist <= rng * 1.25:
            return [(0, speed), (dist, speed)]
        start = dist - rng
        pts = [(0, speed), (start, speed)]
        if steps >= 2:
            for i in range(1, steps + 1):
                a = start + rng * (i - 1) / steps
                b = start + rng * i / steps
                v = speed - (speed - end) * i / steps
                pts += [(a, v), (b, v)]
        else:
            pts += [(start, end), (dist, end)]
        return pts

    def _paintGraph(self, b, dist):
        c = self.graphCanvas
        c.delete("all")
        W = max(c.winfo_width(), 200)
        H = max(c.winfo_height(), 120)
        l, r, t, bt = 40, W - 14, 14, H - 26
        speed = float(b.get("maxSpeed", 127))
        rng = float(b.get("decelRange", 0))
        end = float(b.get("decelSpeed", 40))
        steps = int(b.get("decelSteps", 1)) if b["kind"] == "moveToPoint" else 1
        c.create_rectangle(l, t, r, bt, outline="#3a3a3a")

        def X(inches):
            return l + (r - l) * inches / max(dist, 1)

        def Y(spd):
            return bt - (bt - t) * spd / 127.0
        for sp in (0, 40, 80, 127):
            c.create_line(l - 3, Y(sp), l, Y(sp), fill="#555")
            c.create_text(l - 6, Y(sp), text=str(sp), anchor="e", fill="#777", font=("Segoe UI", 7))
        c.create_text(l, bt + 12, text="0 in", anchor="w", fill="#777", font=("Segoe UI", 7))
        c.create_text(r, bt + 12, text=f"{dist:.0f} in", anchor="e", fill="#777",
                      font=("Segoe UI", 7))
        flat = []
        for xi, sp in self.speedProfile(dist, speed, rng, end, steps):
            flat += [X(xi), Y(sp)]
        c.create_line(*flat, fill="#2ECC71", width=2)
        slowing = rng > 0 and end < speed and dist > rng * 1.25
        if slowing:
            xs = X(dist - rng)
            c.create_line(xs, t, xs, bt, fill="#FF9F43", dash=(3, 3))
            c.create_text(xs + 4, t + 8, text="slow-down starts", anchor="w", fill="#FF9F43",
                          font=("Segoe UI", 7))
        if slowing:
            info = (f"{dist:.1f} in   {speed:.0f} -> {end:.0f} over the last {rng:.0f} in"
                    + (f"   ({steps}-step ramp)" if steps >= 2 else ""))
        else:
            info = f"{dist:.1f} in   {speed:.0f} the whole way   (set a decel range to slow down)"
        self.graphInfo.config(text=info)

    def _drawMap(self):
        """Repaint the route layer. The field render is created once and left
        alone -- only the items drawn on top of it ("dyn") are replaced, all
        inside this one callback, so the map never shows a half-drawn frame
        and the big image isn't rebuilt every time something moves."""
        self._redrawPending = False
        c = self.map
        if not c.find_withtag("field"):
            # Clean like path.jerryio: just the field on a flat dark ground --
            # no grid, no wash over the render.
            if self.fieldImage:
                c.create_image(self.mapX0 + FIELD_IMG_SHIFT_IN[0] * self.ppi,
                               self.mapY0 + FIELD_IMG_SHIFT_IN[1] * self.ppi, anchor="nw",
                               image=self.fieldImage, tags="field")
            else:
                c.create_rectangle(self.mapX0, self.mapY0, self.mapX0 + self.mapPx,
                                   self.mapY0 + self.mapPx, fill="#222222", outline="",
                                   tags="field")
            c.tag_lower("field")
        c.delete("dyn")
        c.delete("live")
        if self.pinEdit:
            # editing the pins: the route is put away, only the pins are shown; it comes back
            # (with the pins where they were moved to) when editing is done
            try:
                self.drawPinHints()
                self._paintModePill()
            finally:
                self._tagNew("dyn")
            c.tag_raise("ui")
            return
        # The simulation is looked up once per paint: pointHeading() asks for
        # it for every point, and the cache key is a repr of every block.
        self._frameSim = self.simulation()
        try:
            self._paintMap()
            self._paintModePill()
        finally:
            self._frameSim = None
            self._tagNew("dyn")
        self._paintLive()
        self._tagNew("live")
        c.tag_raise("live")
        c.tag_raise("ui")

    def modeHint(self):
        """(mode, what to do) -- what a click on the map does right now."""
        if self.pinEdit:
            return ("EDITING PINS", "drag a pin  ·  tap empty field to add  ·  right-click to delete"
                                    "  ·  P when done")
        if self.pickStart:
            return ("PICK START", "tap the map where the robot starts")
        if self.pending:
            k = self.pending[0]["kind"]
            if k == "goToPin":
                return ("GRAB PIN", "tap a pin  ·  Enter to add  ·  Esc to cancel")
            if k == "shootAtGoal":
                return ("SHOOT AT GOAL", "tap a goal, then tap the side to shoot from  ·  Enter to "
                                         "add  ·  Esc to cancel")
            return (k.upper(), "tap the map to place it  ·  Enter to add  ·  Esc to cancel")
        if self.drawMode and self.mergeMode:
            return ("MERGE", "tap the points to merge  ·  M to merge  ·  Esc to cancel")
        if self.drawMode:
            return ("DRAWING", "click: curve point  ·  right-click: straight point  ·  Enter: apply "
                               " ·  D: stop drawing")
        if self.driveMode:
            return ("DRIVING", "W A S D drive the robot  ·  Esc to stop")
        return ("", "")

    def _paintModePill(self):
        """A pill at the top of the map saying what the next click does, so a mode is never a
        mystery. Drawn only while a mode is on."""
        mode, hint = self.modeHint()
        if not mode:
            return
        c = self.map
        w = c.winfo_width()
        x0 = max(self.ICON_MARGIN, self.mapX0) + 8
        y0 = 8
        t1 = c.create_text(x0 + 12, y0 + 11, text=mode, anchor="w", fill=ACCENT,
                           font=("Segoe UI", 9, "bold"))
        b1 = c.bbox(t1)
        t2 = c.create_text(b1[2] + 10, y0 + 11, text=hint, anchor="w", fill="#e8eaf0",
                           font=("Segoe UI", 9))
        b2 = c.bbox(t2)
        right = min(b2[2] + 12, w - self.ICON_MARGIN - 6)
        r = c.create_rectangle(x0, y0, right, y0 + 22, fill="#0f1116", outline=ACCENT)
        c.tag_lower(r, t1)

    def _tagNew(self, tag):
        """Tag the items just drawn: the ones not yet in any layer."""
        c = self.map
        for item in c.find_withtag("!field&&!ui&&!dyn&&!live"):
            c.addtag_withtag(tag, item)

    def redrawLive(self):
        """Repaint only the live robot and its trail. The robot reports 20
        times a second; redoing the whole route for each would be most of
        the flicker, so those updates touch just this small layer, at most
        about 25 times a second."""
        if self._livePending or self._redrawPending:
            return          # a repaint is already on its way, and includes this
        self._livePending = True
        self.after(40, self._drawLive)

    def _drawLive(self):
        self._livePending = False
        if self._redrawPending:
            return
        c = self.map
        c.delete("live")
        self._paintLive()
        self._tagNew("live")
        c.tag_raise("live")
        c.tag_raise("ui")

    # A step this long between two trail samples is not driving: the robot's
    # position was reset (setPose, "Send to robot") or the odometry glitched.
    # Joining the two would draw a straight line clean across the field.
    TRAIL_JUMP_IN = 18.0

    def trailRuns(self):
        """The robot's trail as separate polylines, split wherever it jumps or
        has a point that can't be on the field, and ending at the live pose
        when that is just a short step on."""
        limit = FIELD_IN / 2 + 12
        runs, run = [], []
        for pt in self.robotPath:
            x, y = pt
            if not (math.isfinite(x) and math.isfinite(y)) or abs(x) > limit or abs(y) > limit:
                if len(run) > 1:
                    runs.append(run)
                run = []
                continue
            if run and math.hypot(x - run[-1][0], y - run[-1][1]) > self.TRAIL_JUMP_IN:
                if len(run) > 1:
                    runs.append(run)
                run = []
            run.append((x, y))
        if run and self.robotPose and math.hypot(self.robotPose[0] - run[-1][0],
                                                 self.robotPose[1] - run[-1][1])                 <= self.TRAIL_JUMP_IN:
            run.append((self.robotPose[0], self.robotPose[1]))   # the thinned trail stops short
        if len(run) > 1:
            runs.append(run)
        return runs

    def _paintLive(self):
        # robot: driven path then live pose
        for run in self.trailRuns():
            flat = [v for x, y in run for v in self.toMap(x, y)]
            self.drawPolyline(flat, "#2EFF8C", 2)
        if self.robotPose:
            self.drawRobotMarker(self.robotPose, "#2EFF8C", "", robot=True)

    def _paintMap(self):
        c = self.map
        # planned route: every block's movement, drawn from where the
        # previous block left the robot, so the whole stack reads as one
        # route. All in the path green, the selected block's leg brighter.
        sim = self.simulation()
        self.drawStartRobot()
        # footprint boxes only at the ends of the route and the selected
        # block -- one per block would bury a drawn path
        lastMove = max((i for i, b in enumerate(self.blocks)
                        if b["kind"] in PLACED_ON_MAP or b["kind"] == "setPose"), default=None)
        for i, (b, (pose, shape)) in enumerate(zip(self.blocks, sim)):
            sel = (i == self.selected)
            col = PATH_SEL if sel else PATH_GREEN
            w = 4 if sel else 3
            dash = (6, 3) if not b.get("forwards", True) else None
            if shape and shape[0] == "trace":
                self.drawTrace(shape[1], col, w, dash, arrow=False)
            elif shape and shape[0] == "swing":     # just the line the robot's centre takes
                self.drawTrace(shape[5], col, w, None, arrow=False)
            elif shape and shape[0] == "turn":
                # no arc for an applied turn: they pile up along a route. Only the
                # selected turnToPoint shows where it aims.
                _, x, y, a0, a1 = shape[:5]
                if sel and b["kind"] == "turnToPoint":
                    x0, y0 = self.toMap(x, y)
                    x1, y1 = self.toMap(b["x"], b["y"])
                    c.create_line(x0, y0, x1, y1, fill=col, width=1, dash=(2, 4))
                    c.create_oval(x1 - 4, y1 - 4, x1 + 4, y1 + 4, outline=col)
            # a marker only where it means something: the selected block and
            # the end of the route (the flag), not one per block
            swing = shape and shape[0] == "swing"
            if swing and (sel or i == self.hoverSwing) and i != lastMove:
                self.drawSwingEnd(shape, PATH_SEL)      # where the swing leaves the robot
            elif sel and not swing and (b["kind"] in PLACED_ON_MAP or b["kind"] == "setPose"):
                self.drawRobotBox(pose, PURPLE, width=1, dash=(4, 3))
            if i == lastMove:
                self.drawRobotBox(pose, END_BOX, width=2)

        self.drawFunctionBadges(sim)

        # every pin always has a faint ring; a pin you have chosen gets a solid one (below)
        self.drawPinHints()
        self.drawGoalHints()

        # The route's points, always visible like path.jerryio's: a disc on
        # every block's target, drawn after all the lines so none hides one.
        # The selected block's is larger; the last one carries the flag.
        for i, b in enumerate(self.blocks):
            if b["kind"] in ("moveToPoint", "moveToPose"):
                r = 11 if i == self.selected else 8
                self.drawPoint(b["x"], b["y"], PURPLE, r=r)
                self.drawHeadingLine(i, r)
            elif b["kind"] == "goToPin":            # a pin: orange, with its lift level beside it
                r = 11 if i == self.selected else 8
                self.drawPinRing(b["x"], b["y"], PIN_COLOR)
                self.drawPoint(b["x"], b["y"], PIN_COLOR, r=r)
                px, py = self.toMap(b["x"], b["y"])
                self.map.create_text(px + PIN_RING_PX + 4, py, text=f"{i + 1} PIN {b['level']}",
                                     anchor="w",
                                     fill=PIN_COLOR, font=("Consolas", 9, "bold"))
            elif b["kind"].startswith("swing"):     # the point the swing ends on
                r = 11 if i == self.selected else 8
                self.drawPoint(*self.pointPos(i), PURPLE, r=r)
                self.drawHeadingLine(i, r)
        ends = [b for b in self.blocks if b["kind"] in ("moveToPoint", "moveToPose")]
        if ends:
            self.drawFlag(ends[-1]["x"], ends[-1]["y"])
        self._chips = []
        if self.pending:
            self._paintPending()
        else:
            sel = self.selected
            if sel is not None and sel < len(self.blocks):
                self.drawExitRing(self.blocks[sel], sim[sel][1])
            if sel is not None and sel < len(self.blocks) \
                    and self.blocks[sel]["kind"] == "moveToPose":
                self.drawLeadHandle(self.blocks[sel], self.poseBefore(sel), PATH_SEL)
        if not self.pending and self.aimVisible():
            i = self.selected
            b = self.blocks[i]
            if b["kind"] in self.AIM_KINDS:
                self.drawAimLine(i, self.poseBefore(i))
            else:                       # turn / swing to a point: show its target
                self.drawPoint(b["x"], b["y"], PATH_SEL, r=8)
            self.drawShapeChips(b, self.poseBefore(i))

        self.drawWaypoints()

    def toggleFunctions(self):
        """Hide / show the function badges (lift, claws...) so they don't
        cover the route while it's being drawn. The blocks are untouched."""
        self.showFunctions = not self.showFunctions
        self._setIconActive("fns", not self.showFunctions)      # lit while hidden
        self.mapStatus.config(text="function badges " + ("shown" if self.showFunctions
                                                         else "hidden"))
        self.redrawMap()

    def drawFunctionBadges(self, sim):
        """A badge on the map for every robot function block, at the pose the
        robot is in when it runs (the end of the block before it). Several at
        one spot stack upwards so none hides another."""
        if not self.showFunctions:
            return
        c = self.map
        stacked = []        # (map x, map y) of badges already placed
        for i, b in enumerate(self.blocks):
            if b["kind"] not in FUNCTIONS:
                continue
            pose = sim[i][0]
            px, py = self.toMap(pose[0], pose[1])
            n = sum(1 for sx, sy in stacked if abs(sx - px) < 40 and abs(sy - py) < 40)
            stacked.append((px, py))
            col = FUNCTION_COLORS[b["kind"]]
            sel = i == self.selected
            label = f"{i + 1} " + block_summary(b).replace("liftToLevel", "LIFT") \
                .replace("(wait)", "").replace("(+ open claws)", "").upper().strip()
            bx, by = px + 16, py - 16 - n * 20
            c.create_line(px, py, bx, by, fill=col, width=2)
            c.create_oval(px - 4, py - 4, px + 4, py + 4, fill=col, outline="")
            w = 7 * len(label) + 12
            c.create_rectangle(bx, by - 9, bx + w, by + 9, fill="#d8d8d8",
                               outline=PATH_GREEN if sel else "#7a7a7a", width=2 if sel else 1)
            c.create_text(bx + 6, by, text=label, anchor="w", fill="#000000",
                          font=("Consolas", 9, "bold"))

    # -- mouse path editor --
    def spacing(self):
        try:
            return max(2.0, float(self.spacingVar.get()))
        except (ValueError, tk.TclError):
            return 12.0

    def pathPoints(self):
        """The drawn route as (x, y, stop) points -- see path_points()."""
        self.pathLegs = []
        return path_points(self.effectiveWaypoints(), self.spacing(), self.smoothVar.get(),
                           legs=self.pathLegs)

    def drawWaypoints(self):
        """Control points and the route through them, live while drawing."""
        if not self.waypoints:
            if self.drawMode:
                self.drawInfo.config(text="click the map to drop points" + (
                    "  (continues from the end of the route)" if self.anchor() else ""))
            return
        c = self.map
        pts = self.pathPoints()
        if len(pts) > 1:
            flat = [v for x, y, _ in pts for v in self.toMap(x, y)]
            self.drawPolyline(flat, PATH_GREEN, 3,
                              dash=(7, 4) if self.reverseVar.get() else None)
        if len(self.mergeSel) >= 2:     # preview: the stretch that Merge will join
            lo, hi = min(self.mergeSel), max(self.mergeSel)
            span = [v for w in self.waypoints[lo:hi + 1] for v in self.toMap(w[0], w[1])]
            c.create_line(*span, fill=MERGE_GOLD, width=2, dash=(4, 4))
        for i, w in enumerate(self.waypoints):
            px, py = self.toMap(w[0], w[1])
            straight = len(w) > 2 and w[2] == STRAIGHT
            r = 11
            # translucent purple points like path.jerryio; a hard (straight)
            # corner is a square instead of a disc
            if straight:
                c.create_rectangle(px - r, py - r, px + r, py + r, fill=POINT_FILL,
                                   outline=PURPLE, width=1)
            else:
                self.drawPoint(w[0], w[1], PURPLE, r)
            if i in self.mergeSel:      # picked for merging
                c.create_oval(px - r - 4, py - r - 4, px + r + 4, py + r + 4,
                              outline=MERGE_GOLD, width=2)
            c.create_text(px, py, text=str(i + 1), fill="#f2f2f2", font=("Segoe UI", 7, "bold"))
        self.drawWaypointHeadings()
        lx, ly = self.waypoints[-1][0], self.waypoints[-1][1]
        if len(pts) > 1:
            # where the robot would end up, facing along the last leg
            hdg = heading_to(pts[-2][0], pts[-2][1], pts[-1][0], pts[-1][1],
                             not self.reverseVar.get())
            self.drawRobotBox((pts[-1][0], pts[-1][1], hdg), END_BOX, width=2)
        self.drawFlag(lx, ly)
        curves = sum(1 for w in self.waypoints if len(w) < 3 or w[2] == CURVE)
        dist = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                   for a, b in zip(pts, pts[1:]))
        shape = ("turnToHeading + moveToPoint per leg" if not self.smoothVar.get()
                 else f"{len(pts) - 1} moveToPoint")
        self.drawInfo.config(
            text=f"{len(self.waypoints)} points ({curves} curve, "
                 f"{len(self.waypoints) - curves} straight)  ->  {shape}  ({dist:.0f} in)")

    def wpTravelHeading(self, i):
        """The way the robot faces leaving drawn point i along the route
        (the last point: arriving at it), when no heading was chosen."""
        fwd = not self.reverseVar.get()
        wp = self.waypoints
        if i + 1 < len(wp):
            return heading_to(wp[i][0], wp[i][1], wp[i + 1][0], wp[i + 1][1], fwd)
        if i > 0:
            return heading_to(wp[i - 1][0], wp[i - 1][1], wp[i][0], wp[i][1], fwd)
        end = self.anchor()
        return end[2] if end is not None else 0.0

    def wpHeading(self, i):
        """(heading, set) of drawn point i: the one chosen with the heading
        buttons, else the direction of travel."""
        w = self.waypoints[i]
        if len(w) > 3 and w[3] is not None:
            return w[3], True
        return self.wpTravelHeading(i), False

    def setWpHeading(self, i, theta):
        w = self.waypoints[i]
        self.waypoints[i] = (w[0], w[1], w[2] if len(w) > 2 else CURVE,
                             round(theta % 360.0, 1))

    def curWp(self):
        """Index of the drawn point the heading controls act on, or None."""
        i = self.selWp
        return i if self.drawMode and i is not None and i < len(self.waypoints) else None

    def drawWaypointHeadings(self):
        """A heading line out of every drawn point, as path.jerryio does. One
        chosen with the heading buttons is solid with an arrowhead; the rest
        are dashed and only show the direction of travel."""
        for i, w in enumerate(self.waypoints):
            h, isSet = self.wpHeading(i)
            px, py = self.toMap(w[0], w[1])
            rad = math.radians(h)
            kx, ky = px + math.sin(rad) * self.HEAD_LEN, py - math.cos(rad) * self.HEAD_LEN
            sel = i == self.selWp
            self.map.create_line(px, py, kx, ky, fill="#d6c8ff" if isSet or sel else "#b3a6e6",
                                 width=2, arrow="last")
            if sel:
                self.map.create_oval(px - 13, py - 13, px + 13, py + 13, outline="#d6c8ff",
                                     width=2)

    def routeEnd(self):
        """(x, y, heading) the existing blocks leave the robot at, or None
        with no blocks. The position is the last block's *target* point,
        not where the simulated robot happened to stop, so a path drawn from
        here joins the line already on the map with no gap."""
        if not self.blocks:
            return None
        sim = self.simulation()
        if not sim:
            return None
        x, y, th = sim[-1][0]
        for b in reversed(self.blocks):
            if b["kind"].startswith("swing"):
                break           # a swing moves the robot: the route ends where it does
            if b["kind"] == "goToPin":
                # grabbing a pin ends with the claw on the pin and the robot's centre `reach`
                # short of it, facing it. The next path continues from the robot's TIP -- the
                # claw, `reach` inches ahead of that centre, i.e. on the pin -- at the heading
                # the robot ends with.
                # (taken straight from the pin itself, so it can never land on where the
                # simulated robot happened to be when its deceleration started)
                return (b["x"], b["y"], th)
            if b["kind"] in ("moveToPoint", "moveToPose", "setPose"):
                return (b["x"], b["y"], th)
        return (x, y, th)

    def anchor(self):
        """The pose a drawn path continues from: where the blocks already in
        the plan leave the robot. None when there are no blocks (the first
        point drawn is then the start pose) or while re-shaping a traced
        route. It is looked up fresh every time, never stored, so editing
        the blocks by hand moves it and the drawing can't drift off the end
        of the route."""
        if self.replaceOnApply:
            return None
        end = self.routeEnd()
        if end is None and self.startPlaced:
            end = tuple(self.start)         # the robot as placed on the map
        elif end is None and self.robotPose:
            # nothing planned yet: the first path starts from the robot
            # itself, wherever it currently is
            end = tuple(self.robotPose)
        return end

    def effectiveWaypoints(self):
        """Control points of the drawing, led by the anchor if there is one."""
        end = self.anchor()
        # The leg out of the anchor is whatever the first point drawn is: a
        # left click curves out of the route's end, a right click runs
        # straight to it. (A straight anchor would make the first leg
        # straight whatever you clicked.)
        first = self.waypoints[0] if self.waypoints else None
        mode = first[2] if first is not None and len(first) > 2 else STRAIGHT
        lead = [(end[0], end[1], mode)] if end else []
        return lead + list(self.waypoints)

    def toggleDraw(self):
        if self.pinEdit:
            self.togglePinEdit()
        if self.pending:
            self.cancelPending()
        self.drawMode = not self.drawMode
        self.aimPending = False
        self.drawBtnRow.config(bg=PURPLE if self.drawMode else "#3a3a3a",
                               fg="#ffffff" if self.drawMode else "#e6e6e6",
                               text="Drawing  (D)" if self.drawMode else "Draw  (D)")
        if not self.drawMode:
            self.cancelMerge()
        self._setIconActive("draw", self.drawMode)       # lit while drawing
        if self.drawMode and self.pickStart:
            self.togglePickStart()
        self.redrawMap()

    def undoWaypoint(self):
        if self.waypoints:
            self.redoStack.append(("wp", self.waypoints.pop()))
            self.mergeSel = set()
            if self.selWp is not None and self.selWp >= len(self.waypoints):
                self.selWp = None
            self.syncPointPanel()
            self.redrawMap()

    def pushUndo(self):
        """Snapshot the plan before a path is applied, for undoPath()."""
        self.undoStack.append(self.snapshot())
        del self.undoStack[:-50]
        self.redoStack = []             # a new edit forks history

    def undoPath(self):
        """Ctrl+Z: drop the control point being drawn, or, with none in
        progress, take back the path that was applied last."""
        if self.waypoints:
            self.undoWaypoint()
            return
        if not self.undoStack:
            return
        self.redoStack.append(("plan", self.snapshot()))
        self.restore(self.undoStack.pop())

    def snapshot(self):
        return (copy.deepcopy(self.blocks), list(self.start), self.selected)

    def restore(self, snap):
        self.blocks, start, self.selected = copy.deepcopy(snap[0]), snap[1], snap[2]
        self.start = list(start)
        for v, val in zip(self.startVars, self.start):
            v.set(fmt(val))
        self.replaceOnApply = False
        self.refreshList()
        self.showParams()

    def redoPath(self):
        """Ctrl+Y / Ctrl+Shift+Z: put back what Ctrl+Z last took away."""
        if not self.redoStack:
            return
        kind, item = self.redoStack.pop()
        if kind == "wp":
            self.waypoints.append(item)
            self.redrawMap()
        else:
            self.undoStack.append(self.snapshot())
            self.restore(item)

    def clearWaypoints(self):
        self.waypoints = []
        self.mergeSel = set()
        self.replaceOnApply = False
        self.drawInfo.config(text="")
        self.redrawMap()

    def clearMap(self):
        """Wipe the lines off the map: the drawn control points and the
        path the robot actually drove. The movement blocks are the plan
        rather than scribble, so those are only cleared if asked for."""
        if self.pending:
            self.cancelPending()
        self.waypoints = []
        self.mergeSel = set()
        self.replaceOnApply = False
        self.robotPath = []
        self.drawInfo.config(text="")
        if self.blocks and messagebox.askyesno(
                "Clear map", f"Also remove the {len(self.blocks)} movement "
                             "block{} and the route they draw?".format(
                                 "" if len(self.blocks) == 1 else "s"),
                default="no", parent=self):
            self.pushUndo()
            self.blocks = []
            self.selected = None
            self.showParams()
        self.refreshList()

    def cancelMerge(self):
        if self.mergeMode:
            self.mergeMode = False
            self.mergeSel = set()
            self._setMergeUi(False)
            self.drawInfo.config(text="")
            self.redrawMap()

    def deleteSelectedWaypoint(self):
        i = self.selWp
        if i is not None and i < len(self.waypoints):
            del self.waypoints[i]
            self.mergeSel = set()
            self.selWp = None
            self.syncPointPanel()
            self.redrawMap()

    def _setMergeUi(self, on):
        self.mergeBtn.config(bg=MERGE_GOLD if on else "#3a3a3a",
                             fg="#1b1b1b" if on else "#e6e6e6",
                             text="Merge: tap to finish" if on else "Merge  (M)")
        self._setIconActive("merge", on)

    def toggleMerge(self):
        """Merge button. First tap: pick drawn points on the map (click to
        add / remove). Second tap: every point from the first to the last
        one picked becomes a curve point, so those legs join into one smooth
        spline the robot flows through. With nothing picked it just leaves
        merge mode."""
        if not self.mergeMode:
            if len(self.waypoints) < 2:
                self.drawInfo.config(text="draw at least two points to merge")
                return
            self.mergeMode = True
            self.mergeSel = set()
            if not self.drawMode:
                self.toggleDraw()
            self._setMergeUi(True)
            self.drawInfo.config(text="Merge: click the points to join (first to last), "
                                      "then tap Merge / Enter.  Esc cancels.")
            self.redrawMap()
            return
        picked = sorted(self.mergeSel)
        if len(picked) >= 2 or (picked and len(self.waypoints) > 1):
            lo, hi = picked[0], picked[-1]
            if lo == hi:        # a lone point: join it to both neighbours
                lo, hi = max(0, lo - 1), min(len(self.waypoints) - 1, hi + 1)
            for k in range(lo, hi + 1):
                w = self.waypoints[k]
                self.waypoints[k] = (w[0], w[1], CURVE) + tuple(w[3:])
        self.mergeMode = False
        self.mergeSel = set()
        self._setMergeUi(False)
        self.drawInfo.config(text="")
        self.redrawMap()

    def toggleWaypointMode(self, i):
        """Flip a dropped point between curve and straight."""
        x, y = self.waypoints[i][0], self.waypoints[i][1]
        mode = self.waypoints[i][2] if len(self.waypoints[i]) > 2 else CURVE
        self.waypoints[i] = (x, y, STRAIGHT if mode == CURVE else CURVE) + tuple(
            self.waypoints[i][3:])
        self.redrawMap()

    def removeWaypointAt(self, ex, ey):
        i = self._hitWaypoint(ex, ey)
        if i is not None:
            del self.waypoints[i]
            self.mergeSel = set()
            self.selWp = None
            self.syncPointPanel()
            self.redrawMap()

    def tracePointsFromBlocks(self):
        """Load the route the blocks already describe back in as control
        points, so an existing plan can be re-shaped with the mouse."""
        # traced back as straight points, so the route is reproduced exactly
        # as the blocks describe it -- right-click any of them to round it off.
        pts = [(round(self.start[0], 1), round(self.start[1], 1), STRAIGHT)]
        for b in self.blocks:
            if b["kind"] in ("moveToPoint", "moveToPose", "setPose"):
                pt = (round(b["x"], 1), round(b["y"], 1), STRAIGHT)
                if math.hypot(pt[0] - pts[-1][0], pt[1] - pts[-1][1]) > 0.05:
                    pts.append(pt)
        self.waypoints = pts
        self.replaceOnApply = True      # the traced route replaces the blocks
        if not self.drawMode:
            self.toggleDraw()
        else:
            self.redrawMap()

    # A leg is treated as carrying straight on (no turn block) below this
    # much heading change -- a turnToHeading for a degree or two would cost
    # its whole timeout for nothing.
    TURN_DEG = 3.0

    def applyDrawnPath(self):
        """Turn the drawn route into blocks: first point = start pose
        (facing the next point), every point after it a moveToPoint.

        With "smooth" on, points along a curve keep moving through the
        corner (minSpeed + earlyExitRange) instead of stopping dead on
        each. With it off the route is turn-and-drive: wherever it turns
        left or right the robot is aimed down the next leg with a
        turnToHeading first, so the moveToPoint after it drives a
        straight line instead of steering onto the target as it goes."""
        pts = self.pathPoints()
        end = self.anchor()
        # With blocks already in the plan the drawing is led by their end
        # point (see anchor()), so the new path is appended and joins on
        # exactly; with none, the first point drawn becomes the start pose.
        replace = end is None
        fromRobot = end is not None and not self.blocks   # first path: starts at the robot
        if len(pts) < 2:
            messagebox.showinfo("Draw path", "Drop at least one point on the map first."
                                if end else "Drop at least two points on the map first.",
                                parent=self)
            return
        self.pushUndo()
        fwd = not self.reverseVar.get()
        exitRange = round(self.spacing() / 2.0, 2)
        # With "smooth" off the route is a turn-and-drive one: aim down each
        # leg first, so the moveToPoint that follows is a straight line.
        turnFirst = not self.smoothVar.get()
        blocks = []
        if replace:
            x0, y0 = pts[0][0], pts[0][1]
            hdg = heading_to(x0, y0, pts[1][0], pts[1][1], fwd)
            if len(self.waypoints[0]) > 3 and self.waypoints[0][3] is not None:
                hdg = self.waypoints[0][3]         # the start heading was chosen
            self.start = [round(x0, 1), round(y0, 1), round(hdg, 1)]
            for v, val in zip(self.startVars, self.start):
                v.set(fmt(val))
            rest = pts[1:]
            cx, cy, ch = x0, y0, hdg
        else:
            if fromRobot:       # the start pose is where the robot is now, heading included
                self.start = [round(v, 1) for v in end]
                for v, val in zip(self.startVars, self.start):
                    v.set(fmt(val))
            cx, cy, ch = end
            sim = self.simulation()
            if not fromRobot and sim:
                # The drawing is led from the anchor (after a goToPin: the pin itself), but the
                # robot really is where the simulation left it -- its centre, `reach` short of
                # the pin, facing it. Aim and turn from there, so there is no jump to the pin
                # and no turn around to face the drawn line.
                cx, cy, ch = sim[-1][0]
            rest = pts[1:]      # pts[0] is the anchor: the robot is already there
        legs = list(self.pathLegs)      # filled by pathPoints() above, one per point after the first
        legs = legs[len(legs) - len(rest):] if len(legs) >= len(rest) else [True] * len(rest)
        if self.avoidVar.get():
            # every leg that would clip a goal gets waypoints round it (straight legs, so each
            # is aimed down first), then carries on to the point that was drawn
            clr = self.robotSize()
            newRest, newLegs, prev = [], [], (cx, cy)
            for (x, y, stop), leg in zip(rest, legs):
                for w in avoid_route(prev, (x, y), clr):
                    newRest.append((w[0], w[1], False))
                    newLegs.append(True)
                newRest.append((x, y, stop))
                newLegs.append(leg)
                prev = (x, y)
            rest, legs = newRest, newLegs
        # headings chosen on drawn points: each lands on the nearest point of
        # the route, which then turns to it (and so stops there)
        chosen = {}
        for k, w in enumerate(self.waypoints):
            if len(w) > 3 and w[3] is not None and not (replace and k == 0):
                n = min(range(len(rest)),
                        key=lambda m: math.hypot(rest[m][0] - w[0], rest[m][1] - w[1]),
                        default=None)
                if n is not None:
                    chosen[n] = w[3]
        for i, (x, y, stop) in enumerate(rest):
            want = heading_to(cx, cy, x, y, fwd)
            if turnFirst or legs[i]:
                # A straight leg (right click) is aimed down first, so the
                # moveToPoint after it drives a straight line. Without that
                # the robot steers onto the point as it goes -- a curve --
                # whenever it isn't already facing it. Only where the route
                # actually turns: a leg carrying straight on needs no turn.
                if abs((want - ch + 180) % 360 - 180) > self.TURN_DEG:
                    blocks.append(new_block("turnToHeading", theta=round(want, 2)))
                    blocks.append(new_block("waitUntilDone"))
            ch = want           # arriving along this leg, the robot faces its way
            # points along a curve are flowed through (minSpeed + early exit);
            # straight points and the last point are arrived on properly.
            stop = stop or i == len(rest) - 1 or i in chosen
            blocks.append(new_block("moveToPoint", x=round(x, 2), y=round(y, 2), forwards=fwd,
                                    minSpeed=0 if stop else 40,
                                    earlyExitRange=0.0 if stop else exitRange))
            blocks.append(new_block("waitUntilDone"))
            if i in chosen:
                if abs((chosen[i] - ch + 180) % 360 - 180) > self.TURN_DEG:
                    blocks.append(new_block("turnToHeading", theta=round(chosen[i] % 360.0, 2)))
                    blocks.append(new_block("waitUntilDone"))
                ch = chosen[i]
            cx, cy = x, y
        firstNew = 0 if replace else len(self.blocks)
        self.blocks = blocks if replace else self.blocks + blocks
        self.autoTimeoutRange(firstNew, len(self.blocks))
        # Select the last movement of the path just added: a block added next
        # goes in after it (past its waitUntilDone), i.e. on the end of this
        # path -- not after the path's first block, which is where the swing
        # used to end up.
        newEnd = max((k for k in range(len(self.blocks) - len(blocks), len(self.blocks))
                      if self.blocks[k]["kind"] in ("moveToPoint", "moveToPose")),
                     default=len(self.blocks) - 1)
        self.selected = newEnd
        # The drawing is spent; the next one continues from the new end of
        # the route (and Ctrl+Z takes this path back).
        self.waypoints = []
        self.mergeSel = set()
        self.selWp = None
        self.replaceOnApply = False
        self.refreshList()
        self.showParams()

    # -- map hit-testing --
    HIT_PX = 9

    def _hitWaypoint(self, ex, ey):
        for i, w in enumerate(self.waypoints):
            px, py = self.toMap(w[0], w[1])
            if math.hypot(px - ex, py - ey) <= self.HIT_PX:
                return i
        return None

    def _hitBlockPoint(self, ex, ey):
        """A block's target point under the cursor -- the selected block
        wins, so overlapping points stay draggable."""
        order = list(range(len(self.blocks)))
        if self.selected is not None and self.selected < len(self.blocks):
            order.remove(self.selected)
            order.insert(0, self.selected)
        for i in order:
            b = self.blocks[i]
            if b["kind"] not in HAS_POINT:
                continue
            px, py = self.toMap(b["x"], b["y"])
            if math.hypot(px - ex, py - ey) <= self.HIT_PX:
                return i
        return None

    # Everything on the map is drawn as a dark halo with the bright line
    # on top of it, so a route stays readable wherever it crosses the field
    # render, the grid, or another route.
    HALO = "#0B0B12"

    def drawPolyline(self, flat, color, width, dash=None, arrow=None):
        # one clean line, no dark halo -- the field render is plain enough
        if len(flat) < 4:
            return
        self.map.create_line(*flat, fill=color, width=width, dash=dash, arrow=arrow,
                             capstyle="round", joinstyle="round")

    GOAL_PICK_PX = 20

    def goalAt(self, ex, ey):
        """Index into GOALS of the goal under the tap at map px (ex, ey), or None."""
        best, bestD = None, self.GOAL_PICK_PX
        for i, (x, y, _c) in enumerate(GOALS):
            px, py = self.toMap(x, y)
            d = math.hypot(px - ex, py - ey)
            if d <= bestD:
                best, bestD = i, d
        return best

    def clearance(self):
        ln, wd = self.robotSize()
        return goal_clearance(ln, wd)

    def drawGoalHints(self):
        """While Shoot at goal is being placed: a ring round every goal, the size of the goal
        itself, to tap. Nothing is drawn otherwise."""
        if not (self.pending and self.pending[0]["kind"] == "shootAtGoal"):
            return
        r = GOAL_R * (self.toMap(1, 0)[0] - self.toMap(0, 0)[0])    # the goal's radius in map px
        for x, y, color in GOALS:
            px, py = self.toMap(x, y)
            self.map.create_oval(px - r, py - r, px + r, py + r, outline=GOAL_COLORS[color],
                                 width=2)

    def pinAt(self, ex, ey):
        """Index into PINS of the pin under the tap at map px (ex, ey), or None.
        The nearest one wins when two are within reach."""
        best, bestD = None, PIN_PICK_PX
        for i, (x, y) in enumerate(PINS):
            px, py = self.toMap(x, y)
            d = math.hypot(px - ex, py - ey)
            if d <= bestD:
                best, bestD = i, d
        return best

    def drawPinHints(self):
        """A faint ring on every pin, so you can see what a tap will pick -- only while Grab pin
        is being placed or the pins are being edited."""
        if not (self.pinEdit or (self.pending and self.pending[0]["kind"] == "goToPin")):
            return
        for x, y in PINS:
            px, py = self.toMap(x, y)
            if self.pinEdit:        # editing: solid, with the coordinates beside each pin
                self.map.create_oval(px - 9, py - 9, px + 9, py + 9, outline=PIN_COLOR, width=2,
                                     fill="#3a2a10")
                self.map.create_text(px + 12, py - 12, text=f"{x:g}, {y:g}", anchor="w",
                                     fill=PIN_COLOR, font=("Consolas", 8, "bold"))
            else:
                self.map.create_oval(px - 9, py - 9, px + 9, py + 9, outline=PIN_HINT, width=1,
                                     dash=(3, 3))

    def togglePinEdit(self):
        """Edit pins mode: drag a pin to move it, tap empty field to add one, right-click one to
        delete it. goToPin blocks sitting on a pin move with it."""
        if self.pending:
            self.cancelPending()
        if not self.pinEdit:        # entering: put the other map modes away first
            if self.drawMode:
                self.toggleDraw()
            if self.pickStart:
                self.togglePickStart()
        self.pinEdit = not self.pinEdit
        self._setIconActive("pins", self.pinEdit)
        self.pinEditBtn.config(relief="sunken" if self.pinEdit else "raised",
                               text="Done editing pins  (P)" if self.pinEdit
                               else "Edit pin locations  (P)")
        if not self.pinEdit:        # done: Grab pin blocks have followed their pins, so the time
            for i, blk in enumerate(self.blocks):   # each takes to get there is worked out again
                if blk["kind"] == "goToPin" and self.autoTimeoutVar.get():
                    self._autoTimeoutAt(i)
            self.refreshList()
            self.showParams()
        self.mapStatus.config(text="edit pins: the path is hidden -- drag a pin to move it, tap "
                                   "empty field to add one, right-click a pin to delete it"
                              if self.pinEdit else "pins saved -- the path is back")
        self.redrawMap()

    def resetPins(self):
        if not messagebox.askyesno("Reset pins", "Put every pin back where it started?",
                                   parent=self):
            return
        old = list(PINS)
        PINS[:] = DEFAULT_PINS
        for b in self.blocks:       # a grab-pin block follows its pin to the pin's default spot
            if b["kind"] == "goToPin" and (b["x"], b["y"]) in old:
                i = old.index((b["x"], b["y"]))
                if i < len(PINS):
                    b["x"], b["y"] = PINS[i]
        save_pins()
        self.showParams()
        self.refreshList()
        self.redrawMap()

    def _pinPress(self, e):
        """A press in Edit pins mode: pick a pin up, or put a new one down and drag that."""
        i = self.pinAt(e.x, e.y)
        if i is None:
            x, y = self.toField(e.x, e.y)
            PINS.append((round(x, 1), round(y, 1)))
            i = len(PINS) - 1
        self._pinPrev = tuple(PINS[i])
        self.drag = ("pin", i)
        self.redrawMap()

    def _pinDrag(self, i, x, y):
        if i >= len(PINS):
            return
        prev = self._pinPrev
        PINS[i] = (x, y)
        for b in self.blocks:       # grab-pin blocks on this pin go where it goes
            if b["kind"] == "goToPin" and (b["x"], b["y"]) == prev:
                b["x"], b["y"] = x, y
        self._pinPrev = (x, y)
        self.mapStatus.config(text=f"pin {i + 1}: ({x:g}, {y:g}) in")
        self.refreshList()
        self.redrawMap()

    def _pinDelete(self, e):
        i = self.pinAt(e.x, e.y)
        if i is not None:
            del PINS[i]
            save_pins()
            self.redrawMap()

    def drawPinRing(self, x, y, color):
        """The solid ring round a chosen pin."""
        px, py = self.toMap(x, y)
        r = PIN_RING_PX
        self.map.create_oval(px - r, py - r, px + r, py + r, outline=color, width=3)

    def drawPoint(self, x, y, color, r=11):
        """A path point as path.jerryio draws it: a purple disc with a thin
        rim. Solid rather than see-through -- stippled fills are slow to
        paint and were part of the flicker."""
        px, py = self.toMap(x, y)
        self.map.create_oval(px - r, py - r, px + r, py + r, fill=POINT_FILL, outline=color,
                             width=1)

    def drawExitRing(self, b, shape=None):
        """The early-exit range of a moveToPoint / moveToPose: a ring of
        earlyExitRange inches around its target. The motion ends the moment
        the robot is inside it, still travelling, so the next block takes
        over mid-glide. Orange when it will take effect; grey when minSpeed
        is 0, because lemlib ignores earlyExitRange then. A dot marks where
        the simulated robot actually leaves the motion."""
        if b["kind"] not in ("moveToPoint", "moveToPose"):
            return
        rng = b.get("earlyExitRange", 0)
        if rng <= 0:
            return
        c = self.map
        active = b.get("minSpeed", 0) != 0
        col = EXIT_COL if active else EXIT_OFF
        px, py = self.toMap(b["x"], b["y"])
        r = rng * self.ppi
        c.create_oval(px - r, py - r, px + r, py + r, outline=col,
                      width=2 if active else 1, dash=(5, 3))
        c.create_text(px, py + r + 9, fill=col, font=("Segoe UI", 8, "bold"),
                      text=f"exit {fmt(rng)} in" if active
                      else f"exit {fmt(rng)} in -- ignored while minSpeed is 0")
        if active and shape and shape[0] == "trace" and shape[1]:
            ex, ey = self.toMap(*shape[1][-1])       # where the robot leaves the motion
            c.create_oval(ex - 4, ey - 4, ex + 4, ey + 4, fill=col, outline="#1b1b1b")

    HEAD_LEN = 26      # px from a point's centre to the knob on its heading line

    def headingKnob(self, i):
        """Map position (px, py) of block i's heading knob, and the point."""
        px, py = self.toMap(*self.pointPos(i))
        rad = math.radians(self.pointHeading(i)[0])
        return (px + math.sin(rad) * self.HEAD_LEN, py - math.cos(rad) * self.HEAD_LEN), (px, py)

    def pointPos(self, i):
        """Field (x, y) of block i's point. A swing has no target of its own
        (or aims at something else), so its point is where it leaves the robot."""
        b = self.blocks[i]
        if b["kind"].startswith("swing"):
            sim = self.simulation()
            if i < len(sim):
                return sim[i][0][0], sim[i][0][1]
        return b["x"], b["y"]

    def drawHeadingLine(self, i, r):
        """The line out of a point showing the way the robot faces there. A
        heading that has been set is solid; one that is only the direction
        of travel is dashed and dim. The selected point gets a knob to drag."""
        (kx, ky), (px, py) = self.headingKnob(i)
        isSet = self.pointHeading(i)[1]
        col = "#d6c8ff" if (isSet or i == self.selected) else "#b3a6e6"
        self.map.create_line(px, py, kx, ky, fill=col, width=2, arrow="last")
        if i == self.selected:
            self.map.create_oval(kx - 5, ky - 5, kx + 5, ky + 5, fill="#d6c8ff", outline=PURPLE,
                                 width=2)

    def _hitHeadingKnob(self, ex, ey):
        i = self.selected
        if i is None or i >= len(self.blocks) or self.blocks[i]["kind"] not in (
                "moveToPoint", "moveToPose", "swingToHeading", "swingToPoint"):
            return None
        (kx, ky), _ = self.headingKnob(i)
        return i if math.hypot(kx - ex, ky - ey) <= self.HIT_PX else None

    def startKnob(self):
        """Map position of the knob on the start robot's heading line."""
        px, py = self.toMap(self.start[0], self.start[1])
        rad = math.radians(self.start[2])
        return px + math.sin(rad) * self.HEAD_LEN, py - math.cos(rad) * self.HEAD_LEN

    def drawStartRobot(self):
        """The robot where the plan starts: its footprint box, a disc on its
        centre and a heading line with a knob. Drag the robot to put it
        somewhere else, drag the knob to turn it."""
        px, py = self.toMap(self.start[0], self.start[1])
        kx, ky = self.startKnob()
        self.drawRobotBox(tuple(self.start), PURPLE, width=2)
        self.drawPoint(self.start[0], self.start[1], PURPLE, r=9)
        self.map.create_line(px, py, kx, ky, fill="#d6c8ff", width=2)
        self.map.create_oval(kx - 5, ky - 5, kx + 5, ky + 5, fill="#d6c8ff", outline=PURPLE,
                             width=2)

    def _hitStart(self, ex, ey):
        """True on the start robot's body (inside its footprint)."""
        px, py = self.toMap(self.start[0], self.start[1])
        length, wid = self.robotSize()
        reach = max(self.HIT_PX, min(length, wid) / 2 * self.ppi)
        return math.hypot(px - ex, py - ey) <= reach

    def _hitStartKnob(self, ex, ey):
        kx, ky = self.startKnob()
        return math.hypot(kx - ex, ky - ey) <= self.HIT_PX

    def finishStartDrag(self):
        """The robot has been put down: that is now where the first path
        starts, and (idle brain only) where the robot itself is told it is."""
        self.startPlaced = True
        if self.link and self.planStatus[0] != "RUNNING":
            self.link._send(f"POSE {self.start[0]:.2f} {self.start[1]:.2f} {self.start[2]:.2f}")

    def drawFlag(self, x, y):
        """Chequered finish flag standing on the last point of the route."""
        px, py = self.toMap(x, y)
        top = py - 24
        self.map.create_line(px, py - 4, px, top, fill="#e8e8e8", width=2)
        s = 5
        for row in range(2):
            for col in range(3):
                dark = (row + col) % 2 == 0
                x0, y0 = px + 1 + col * s, top + row * s
                self.map.create_rectangle(x0, y0, x0 + s, y0 + s, outline="",
                                          fill="#111111" if dark else "#f4f4f4")

    def drawDirectionArrows(self, pts, color, every_px=70):
        """Arrowheads spaced along a route, so which way round it goes is
        readable without following it end to end."""
        px = [self.toMap(x, y) for x, y in pts]
        run = 0.0
        for (x0, y0), (x1, y1) in zip(px, px[1:]):
            seg = math.hypot(x1 - x0, y1 - y0)
            if seg < 1e-6:
                continue
            run += seg
            if run < every_px:
                continue
            run = 0.0
            ux, uy = (x1 - x0) / seg, (y1 - y0) / seg
            self.map.create_line(x1 - ux * 9, y1 - uy * 9, x1, y1, fill=self.HALO, width=6,
                                 arrow="last", arrowshape=(10, 12, 5))
            self.map.create_line(x1 - ux * 9, y1 - uy * 9, x1, y1, fill=color, width=2,
                                 arrow="last", arrowshape=(9, 11, 4))

    def drawTrace(self, trace, color, width, dash, arrow=True):
        """The simulated path of the robot centre, thinned so Tk isn't
        handed thousands of segments."""
        step = max(1, len(trace) // 120)
        pts = trace[::step]
        if pts[-1] != trace[-1]:
            pts.append(trace[-1])
        if len(pts) < 2 or all(math.hypot(px - pts[0][0], py - pts[0][1]) < 0.05 for px, py in pts):
            return
        flat = [v for x, y in pts for v in self.toMap(x, y)]
        self.drawPolyline(flat, color, width, dash, "last" if arrow else None)

    def drawTurnArc(self, x, y, a0, a1, color, width, swing=False):
        """Arc from heading a0 to a1 around (x, y) -- the short way round,
        like lemlib's AUTO direction -- with an arrowhead at the end. Swings
        pivot about one side of the drivetrain, so their arc is offset and
        drawn dashed to tell them apart."""
        px, py = self.toMap(x, y)
        r = 22
        delta = (a1 - a0 + 180) % 360 - 180   # signed, shortest
        if abs(delta) < 0.5:
            return
        # Tk arcs: start angle CCW from +x; compass heading h -> 90 - h
        start = 90 - a0
        extent = -delta
        if swing:
            off = 9
            side = 1 if delta > 0 else -1
            rad = math.radians(a0 + 90 * side)
            px, py = px + math.sin(rad) * off, py - math.cos(rad) * off
        self.map.create_arc(px - r, py - r, px + r, py + r, start=start, extent=extent,
                            style="arc", outline=self.HALO, width=width + 4)
        self.map.create_arc(px - r, py - r, px + r, py + r, start=start, extent=extent,
                            style="arc", outline=color, width=width,
                            dash=(3, 3) if swing else None)
        end = math.radians(a1)
        ex, ey = px + math.sin(end) * r, py - math.cos(end) * r
        tang = math.radians(a1 + (90 if delta > 0 else -90))
        self.map.create_line(ex - math.sin(tang) * 6, ey + math.cos(tang) * 6, ex, ey,
                             fill=color, width=width, arrow="last")

    def robotSize(self):
        """(length, width) in inches from the window, falling back to the
        default for anything that isn't a positive number."""
        def num(var):
            try:
                v = float(var.get())
                return v if v > 0 else DEFAULT_ROBOT_IN
            except (ValueError, tk.TclError):
                return DEFAULT_ROBOT_IN
        return num(self.robotLenVar), num(self.robotWidVar)

    def drawRobotBox(self, pose, color, width=2, dash=None):
        """The robot's footprint (length along the heading, width across it)
        centred on `pose`, rotated to its heading, with the front edge
        marked so it's clear which way it faces."""
        x, y, th = pose
        px, py = self.toMap(x, y)
        ppi = self.ppi
        length, wid = self.robotSize()
        rad = math.radians(th)
        fwd = (math.sin(rad), -math.cos(rad))     # screen direction of the heading
        right = (math.cos(rad), math.sin(rad))
        corners = []
        for a, f in ((-1, 1), (1, 1), (1, -1), (-1, -1)):     # front-left first
            ca, cf = a * wid / 2 * ppi, f * length / 2 * ppi
            corners += [px + right[0] * ca + fwd[0] * cf, py + right[1] * ca + fwd[1] * cf]
        self.map.create_polygon(*corners, outline=color, fill="", width=width, dash=dash)
        self.map.create_line(*corners[0:4], fill=color, width=width + 2)   # front edge

    def drawSwingEnd(self, shape, color):
        """One footprint box where a swing leaves the robot, turned to the
        heading it ends on."""
        trace, heads = shape[5], shape[6]
        if len(trace) >= 2:
            x, y = trace[-1]
            self.drawRobotBox((x, y, heads[-1]), color, width=2)

    def _swingAt(self, ex, ey):
        """Index of the swing whose path (or end footprint) is under the mouse."""
        best, bestD = None, 14.0
        for i, (b, (pose, shape)) in enumerate(zip(self.blocks, self.simulation())):
            if not shape or shape[0] != "swing":
                continue
            for x, y in shape[5][::2] + [shape[5][-1]]:
                px, py = self.toMap(x, y)
                d = math.hypot(px - ex, py - ey)
                if d < bestD:
                    best, bestD = i, d
        return best

    def drawRobotMarker(self, pose, color, label, small=False, robot=False, box=False):
        x, y, th = pose
        px, py = self.toMap(x, y)
        r = 9 if not small else 6
        rad = math.radians(th)
        tx, ty = px + math.sin(rad) * (r + 10), py - math.cos(rad) * (r + 10)
        if robot or box:
            self.drawRobotBox(pose, color, width=2 if robot else 1, dash=None if robot else (4, 3))
        if robot:
            self.map.create_oval(px - r, py - r, px + r, py + r, outline=color, width=2)
        else:
            self.drawPoint(x, y, color, r=r + 2)
        self.map.create_line(px, py, tx, ty, fill=color, width=2, arrow="last")
        if label:
            self.map.create_text(px, py, text=label, fill=color, font=("Segoe UI", 8, "bold"))

    def _onMapLeave(self, _e):
        self._hoverOver(None)
        if self.hoverSwing is not None:
            self.hoverSwing = None
            self.redrawMap()

    def onMapMotion(self, e):
        icon = self._iconAt(e.x, e.y)
        self._hoverOver(icon)
        if icon:
            self.mapStatus.config(text=self.icons[icon]["tip"])
            if str(self.map.cget("cursor")) != "hand2":
                self.map.config(cursor="hand2")
            return
        x, y = self.toField(e.x, e.y)
        hs = None if (self.pending or self.drag) else self._swingAt(e.x, e.y)
        if hs != self.hoverSwing:           # hovering a swing shows the body's path through it
            self.hoverSwing = hs
            self.redrawMap()
        if self.pickStart:
            hint = "   -- click to set START pose"
        elif self.pending:
            hint = "   -- tap to place, drag to adjust, Enter = Apply, Esc = cancel"
        elif self.mergeMode:
            hint = "   -- click points to pick them, tap Merge / Enter to join, Esc = cancel"
        elif self.drawMode:
            i = self._hitWaypoint(e.x, e.y)
            hint = ("   -- drag to move, right-click to flip curve/straight, "
                    "shift-click to remove") if i is not None else \
                   "   -- left-click = curve point, right-click = straight point"
        elif self._hitAimKnob(e.x, e.y) is not None:
            hint = "   -- drag to aim this turn"
        elif self._hitHeadingKnob(e.x, e.y) is not None:
            hint = "   -- drag to turn this point's heading"
        elif self._hitBlockPoint(e.x, e.y) is not None:
            hint = "   -- drag to move this block's point"
        elif self._hitStartKnob(e.x, e.y):
            hint = "   -- drag to turn the robot"
        elif self._hitStart(e.x, e.y):
            hint = "   -- drag to put the robot where it is"
        elif self.aimable() and (not self.drawMode or self.aimPending):
            kind = self.blocks[self.selected]["kind"]
            what = "swing" if kind.startswith("swing") else "turn"
            hint = (f"   -- click or drag to aim this {what}" if kind in self.AIM_KINDS
                    else f"   -- click to set where this {what} aims; drag to adjust")
        else:
            hint = ""
        # the cursor says what a click will do: a crosshair when it drops a
        # point, a hand over anything that can be picked up
        grab = (self._chipAt(e.x, e.y) is not None or self._hitHeadingKnob(e.x, e.y) is not None
                or self._hitStartKnob(e.x, e.y)
                or self._hitAimKnob(e.x, e.y) is not None
                or self._hitBlockPoint(e.x, e.y) is not None or self._hitStart(e.x, e.y)
                or (self.drawMode and self._hitWaypoint(e.x, e.y) is not None))
        cursor = "hand2" if grab else ("crosshair" if self.drawMode or self.pickStart or self.pending
                                       or (self.aimable() and self.aimPending) else "arrow")
        if str(self.map.cget("cursor")) != cursor:
            self.map.config(cursor=cursor)
        # inches (what the code uses and Jerry.io exports); the cm are what
        # path.jerryio's own cursor readout and point panel show on screen
        self.mapStatus.config(text=f"({x:6.1f}, {y:6.1f}) in  = ({x * 2.54:7.1f}, "
                                   f"{y * 2.54:7.1f}) cm" + hint)

    def onMapPress(self, e):
        icon = self._iconAt(e.x, e.y)
        if icon:                                # a click on one of the map's icons
            self.icons[icon]["cmd"]()
            return
        chip = self._chipAt(e.x, e.y)
        if chip:                                # a shape option beside the selected swing
            self._applyChip(chip)
            return
        if self.pinEdit:
            self._pinPress(e)
            return
        x, y = self.toField(e.x, e.y)
        x, y = round(x, 1), round(y, 1)
        if self.pickStart:
            self.start[0], self.start[1] = x, y
            self.startVars[0].set(fmt(x))
            self.startVars[1].set(fmt(y))
            self.startPlaced = True
            self.togglePickStart()
            self.redrawMap()
            return
        if self.pending and self.pending[0]["kind"] in ("shootAtGoal", "goToPin") \
                and not self.drawMode:
            # these are placed by tapping a goal / pin only, so the robot at the start can still be
            # picked up while one is being placed: drag it, or turn it by its knob
            tapped = self.goalAt(e.x, e.y) is not None if self.pending[0]["kind"] == "shootAtGoal" \
                else self.pinAt(e.x, e.y) is not None
            if not tapped and self._hitStartKnob(e.x, e.y):
                self.pushUndo()
                self.drag = ("starthdg", 0)
                return
            if not tapped and self._hitStart(e.x, e.y):
                self.pushUndo()
                self.drag = ("start", 0)
                return
        if self.pending:                        # placing a motion: the tap sets where it goes
            knob = self._hitPendingKnob(e.x, e.y)
            if knob == "lead":
                self.drag = ("pendlead", 0)
            elif knob == "hdg":
                self.drag = ("pend", 0)         # a drag turns the finish heading
            else:
                self._placePending(e.x, e.y, True)
                self.drag = ("pend", 0)
            return
        sel = self.selected
        if sel is not None and sel < len(self.blocks) and not self.drawMode \
                and self.blocks[sel]["kind"] == "goToPin":
            # a selected goToPin only ever changes by tapping another pin; it never follows a
            # tap on empty field (those fall through, and the checks below leave it where it is)
            p = self.pinAt(e.x, e.y)
            b = self.blocks[sel]
            if p is not None:
                if (b["x"], b["y"]) != tuple(PINS[p]):
                    self.pushUndo()
                    b["x"], b["y"] = PINS[p]
                    self.showParams()
                    self.refreshList()
                    self.redrawMap()
                return
        k = self._hitLeadKnob(e.x, e.y)         # the arc's lead point on the selected moveToPose
        if k is not None:
            self.pushUndo()
            self.drag = ("lead", k)
            return
        k = self._hitHeadingKnob(e.x, e.y)     # the knob on the selected point's heading line
        if k is not None:
            self.pushUndo()
            self.drag = ("hdg", k)
            return
        k = self._hitAimKnob(e.x, e.y)          # the knob on a selected turn / swing's aim line
        if k is not None:
            self.pushUndo()
            self.drag = ("aim", k)
            return
        if self._hitStartKnob(e.x, e.y):       # the knob on the start robot's heading line
            self.pushUndo()
            self.drag = ("starthdg", 0)
            return
        if self.drawMode and self.aimPending and self.aimable():
            self.pushUndo()
            kind = self.blocks[self.selected]["kind"]
            if kind in self.AIM_KINDS:
                self._aimAt(self.selected, e.x, e.y)
                self.drag = ("aim", self.selected)
            else:
                self.blocks[self.selected]["x"], self.blocks[self.selected]["y"] = x, y
                self.autoSide(self.selected)
                self.drag = ("block", self.selected)
            self.showParams()
            self.refreshList()
            return
        if self.drawMode and self.mergeMode:    # picking points to merge
            i = self._hitWaypoint(e.x, e.y)
            if i is not None:
                self.mergeSel ^= {i}
                self.redrawMap()
            return
        if self.drawMode:
            i = self._hitWaypoint(e.x, e.y)
            j = self._hitBlockPoint(e.x, e.y) if i is None else None
            if j is not None:                  # a point of the route: drag it, as in jerry
                self.selWp = None
                self.pushUndo()
                self.selected = j
                self.refreshList()
                self.showParams()
                self.drag = ("block", j)
                return
            if i is None and self._hitStart(e.x, e.y) and not self.waypoints:
                self.pushUndo()                # pick the robot up (nothing drawn yet to click on)
                self.drag = ("start", 0)
                return
            if i is None:                      # new curve point at the end
                self.redoStack = []
                self.waypoints.append((x, y, CURVE))
                i = len(self.waypoints) - 1
            self.selWp = i                     # its heading is set with the heading buttons
            self.drag = ("wp", i)
            self.syncPointPanel()
            self.redrawMap()
            return
        i = self._hitBlockPoint(e.x, e.y)      # grab a block's point and drag it
        if i is not None:
            self.pushUndo()                    # so Ctrl+Z puts a dragged point back
            if i != self.selected:
                self.selected = i
                self.refreshList()
                self.showParams()
            self.drag = ("block", i)
            return
        if self._hitStart(e.x, e.y):           # point at the robot and drag it where it is
            self.pushUndo()
            self.drag = ("start", 0)
            return
        if self.aimable() and self.blocks[self.selected]["kind"] in self.AIM_KINDS:
            # a turn / swing to a heading: the click (and drag) aims it
            self.pushUndo()
            self._aimAt(self.selected, e.x, e.y)
            self.drag = ("aim", self.selected)
            self.showParams()
            self.refreshList()
            return
        if self.selected is None or self.blocks[self.selected]["kind"] not in HAS_POINT \
                or self.blocks[self.selected]["kind"] == "goToPin":   # a pin is picked, not dropped
            return
        self.pushUndo()
        b = self.blocks[self.selected]
        b["x"], b["y"] = x, y
        self.autoSide(self.selected)
        self.drag = ("block", self.selected)
        self.showParams()
        self.refreshList()

    def onMapDrag(self, e):
        if not self.drag:
            return
        x, y = self.toField(e.x, e.y)
        x, y = round(x, 1), round(y, 1)
        what, i = self.drag
        if what == "pin":
            self._pinDrag(i, x, y)
        elif what == "pend":
            if self.pending:
                self._placePending(e.x, e.y, False)
        elif what == "pendlead":
            if self.pending:
                self.setLead(self.pending[0], self._poseAt(self.pending[1]), e.x, e.y)
                self.syncPendingParams()
                self.redrawMap()
        elif what == "lead" and i < len(self.blocks):
            self.setLead(self.blocks[i], self.poseBefore(i), e.x, e.y)
            self.codeLabel.config(text=block_code(self.blocks[i]))
            self.syncList()
        elif what == "wp":
            if i < len(self.waypoints):
                w = self.waypoints[i]
                self.waypoints[i] = (x, y, w[2] if len(w) > 2 else CURVE) + tuple(w[3:])
                self.syncPointPanel()
                self.redrawMap()
        elif what == "start":
            self.start[0], self.start[1] = x, y
            for v, val in zip(self.startVars, self.start):
                v.set(fmt(val))
            self.redrawMap()
        elif what == "starthdg":
            px, py = self.toMap(self.start[0], self.start[1])
            if math.hypot(e.x - px, e.y - py) > 3:
                self.start[2] = round(math.degrees(math.atan2(e.x - px, py - e.y)) % 360.0, 1)
                self.startVars[2].set(fmt(self.start[2]))
                self.redrawMap()
        elif what == "aim" and i < len(self.blocks):
            self._aimAt(i, e.x, e.y)
            self.codeLabel.config(text=block_code(self.blocks[i]))
            self.syncList()
        elif what == "hdg" and i < len(self.blocks):
            # turn the heading to point at the mouse
            _, (px, py) = self.headingKnob(i)
            if math.hypot(e.x - px, e.y - py) > 3:
                self.setPointHeading(i, math.degrees(math.atan2(e.x - px, py - e.y)))
                self.codeLabel.config(text=block_code(self.blocks[i]))
                self.syncPointPanel()
                self.syncList()
        elif i < len(self.blocks):
            b = self.blocks[i]
            if b["kind"] == "goToPin":      # a pin does not follow the mouse
                return
            b["x"], b["y"] = x, y
            self.autoSide(i)
            self.codeLabel.config(text=block_code(b))
            self.syncPointPanel()
            self.syncList()             # redraws the map too

    def onMapRelease(self, _e):
        if self.drag and self.drag[0] == "pin":
            save_pins()
            self.showParams()
        if self.drag and self.drag[0] in ("start", "starthdg"):
            self.finishStartDrag()
        if self.drag and self.drag[0] in ("block", "hdg", "aim", "lead"):
            self.showParams()
        if self.drag and self.drag[0] in ("aim", "block"):
            self.aimPending = False         # aimed: the next click drops a point again
        self.drag = None

    def onMapRightClick(self, e):
        """Right button: drop a STRAIGHT point -- the legs either side of
        it stay dead straight and the robot stops on it. On a point that's
        already there, flip it between straight and curve instead."""
        if self.pinEdit:                    # Edit pins: right-click deletes the pin under the mouse
            self._pinDelete(e)
            return
        if not self.drawMode:
            return
        if self.mergeMode:
            return
        i = self._hitWaypoint(e.x, e.y)
        if i is not None:
            self.selWp = i
            self.syncPointPanel()
            self.toggleWaypointMode(i)
            return
        if self.mergeMode:
            return
        x, y = self.toField(e.x, e.y)
        self.redoStack = []
        self.waypoints.append((round(x, 1), round(y, 1), STRAIGHT))
        self.selWp = len(self.waypoints) - 1
        self.syncPointPanel()
        self.redrawMap()

    # -- driving (WASD) --
    # The brain drops a DRIVE command it hasn't heard from in 300 ms, so
    # the current one is repeated this often while a key is held. Well
    # inside that, and slow enough not to crowd the radio link (the vexcom
    # link paces its sends 60 ms apart).
    DRIVE_KEEPALIVE_MS = 120

    def toggleDrive(self):
        self.driveMode = not self.driveMode
        self.driveBtn.config(relief="sunken" if self.driveMode else "raised")
        if self.driveMode:
            if self.drawMode:      # W/A/S/D would fight the drawing keys
                self.toggleDraw()
            self.focus_set()
            self.driveStatus.config(text="ready -- hold W A S D", fg="#2ECC71")
            self._driveTick()
        else:
            self.stopDrive()

    def onDriveKeyPress(self, e):
        if not self.driveMode or self._typing():
            return
        k = e.keysym.lower()
        if k in ("w", "a", "s", "d"):
            self.driveKeys.add(k)
            self._sendDrive()

    def onDriveKeyRelease(self, e):
        if not self.driveMode:
            return
        self.driveKeys.discard(e.keysym.lower())
        self._sendDrive()

    def driveMix(self):
        """W/S forwards and back, A/D turn, mixed arcade style when held
        together. Turning is given a bit less than full power -- at full
        it spins faster than anyone can steer by keyboard."""
        p = self.powerVar.get()
        fwd = ("w" in self.driveKeys) - ("s" in self.driveKeys)
        turn = ("d" in self.driveKeys) - ("a" in self.driveKeys)
        l = fwd * p + turn * p * 0.7
        r = fwd * p - turn * p * 0.7
        cap = max(1.0, abs(l), abs(r), float(p))
        if cap > 127:               # keep the mix's shape when it clips
            l, r = l * 127 / cap, r * 127 / cap
        return int(round(max(-127, min(127, l)))), int(round(max(-127, min(127, r))))

    def _sendDrive(self, force=False):
        l, r = self.driveMix() if self.driveMode else (0, 0)
        if (l, r) != self.driveLast or force:
            self.link._send(f"DRIVE {l} {r}")
            self.driveLast = (l, r)
        if not self.driveMode:
            return
        if (l, r) == (0, 0):
            self.driveStatus.config(text="ready -- hold W A S D", fg="#2ECC71")
        else:
            self.driveStatus.config(text=f"L {l:+4d}  R {r:+4d}", fg="#00F0FF")

    def _driveTick(self):
        """Keepalive: resends the held command so the brain's watchdog
        doesn't time it out mid-drive."""
        if not self.driveMode:
            return
        if self.driveKeys:
            self._sendDrive(force=True)
        self.driveJob = self.after(self.DRIVE_KEEPALIVE_MS, self._driveTick)

    def stopDrive(self, _e=None):
        """Stop, and mean it: the keys are dropped, the keepalive is
        cancelled and a zero command goes out. If it doesn't arrive, the
        brain's 300 ms watchdog stops the robot anyway."""
        if self.driveJob is not None:
            self.after_cancel(self.driveJob)
            self.driveJob = None
        self.driveKeys.clear()
        if self.driveMode or self.driveLast not in (None, (0, 0)):
            self.link._send("DRIVE 0 0")
            self.driveLast = (0, 0)
        self.driveMode = False
        self.driveBtn.config(relief="raised")
        self.driveStatus.config(text="off", fg="#777")

    # -- start pose --
    def togglePickStart(self):
        if self.pending:
            self.cancelPending()        # picking the start and placing a block can't both be on
        if self.pinEdit:
            self.togglePinEdit()
        self.pickStart = not self.pickStart
        self.pickBtn.config(relief="sunken" if self.pickStart else "raised")

    def applyStartPose(self):
        if self.pending:
            self.cancelPending()
        try:
            self.start = [float(v.get()) for v in self.startVars]
        except ValueError:
            return
        self.startPlaced = True
        self.redrawMap()

    def sendStartPose(self):
        self.applyStartPose()
        self.link._send(f"POSE {self.start[0]:.2f} {self.start[1]:.2f} {self.start[2]:.2f}")

    def resetRobot(self):
        """Forget everything the robot thinks it knows about where it is:
        its pose goes back to the start pose (chassis.setPose), the trail
        it drew on the brain and the driven path on this map are cleared,
        and the last run's status is dropped. The claws go back on
        (extended) and the lift homes, so the robot is in its starting
        configuration, not only its starting place."""
        self.applyStartPose()
        self.link._send(f"POSE {self.start[0]:.2f} {self.start[1]:.2f} {self.start[2]:.2f}")
        self.link._send("PLAN CLEAR")   # also resets the brain's step/status display
        self.link._send("ROBOT HOME")   # claws on, lift homed: the robot's starting configuration
        self.robotPath = []
        self.robotPose = tuple(self.start)
        self.pendingRun = None
        self.redrawMap()

    def useRobotPose(self):
        if self.robotPose:
            self.start = [round(v, 1) for v in self.robotPose]
            self.startPlaced = True
            for v, val in zip(self.startVars, self.start):
                v.set(fmt(val))
            self.redrawMap()

    # -- block list --
    def rowText(self, i, b, times=None):
        """A row of the block list: its number and summary, and -- after a whole run on the
        robot, while the plan is unchanged -- how long the robot really spent on it."""
        t = (times or {}).get(i)
        d = self._blockDriftData.get(i) if times is not None else None
        tag = ""
        if t:
            tag = f"{t:.1f} s"
        if d is not None and d >= 0.5:
            tag += (", " if tag else "") + f"{d:.1f} in off"
        return f"{i + 1:2d}  {KIND_GLYPH.get(b['kind'], ' ')}  {block_summary(b)}" \
            + (f"   [{tag}]" if tag else "")

    def _styleRows(self):
        """Colour each row by what the block is: driving plain, robot functions in the accent
        colour, the waitUntilDone glue dimmed."""
        for row, i in enumerate(self._rowMap):
            if row < self.listbox.size():
                self.listbox.itemconfig(row, fg=row_color(self.blocks[i]["kind"]))
        self._updateGuide()

    def _visibleRows(self):
        """Block indices shown in the list: all of them, or without the waitUntilDone glue (the
        selected block always stays, so a selection never vanishes)."""
        if self.hideGlueVar.get():
            return [i for i, b in enumerate(self.blocks)
                    if b["kind"] != "waitUntilDone" or i == self.selected]
        return list(range(len(self.blocks)))

    def _updateGuide(self):
        """The getting-started card is there only while the plan is empty."""
        if not hasattr(self, "guide"):
            return
        if self.blocks:
            self.guide.pack_forget()
        elif not self.guide.winfo_ismapped():
            self.guide.pack(fill="x", padx=8, pady=(8, 0), before=self.blocksCard)

    def _blockTimes(self):
        return self._blockTimesData if self._blockTimesFor == repr(self.blocks) else None

    def refreshList(self):
        self.listbox.delete(0, "end")
        times = self._blockTimes()
        self._rowMap = self._visibleRows()
        for i in self._rowMap:
            self.listbox.insert("end", self.rowText(i, self.blocks[i], times))
        self._styleRows()
        if self.selected in self._rowMap:
            row = self._rowMap.index(self.selected)
            self.listbox.selection_set(row)
            self.listbox.see(row)
        self.redrawMap()

    def syncList(self):
        """refreshList() for the middle of a drag: the block list has the same
        number of rows, so only the rows whose text changed are rewritten.
        Rebuilding the whole listbox on every mouse move made it flash."""
        if self.listbox.size() != len(self._rowMap) or self._rowMap != self._visibleRows():
            return self.refreshList()
        times = self._blockTimes()
        for row, k in enumerate(self._rowMap):
            b = self.blocks[k]
            text = self.rowText(k, b, times)
            if self.listbox.get(row) != text:
                self.listbox.delete(row)
                self.listbox.insert(row, text)
                self.listbox.itemconfig(row, fg=row_color(b["kind"]))
        want = (self._rowMap.index(self.selected),) if self.selected in self._rowMap else ()
        if tuple(self.listbox.curselection()) != want:
            self.listbox.selection_clear(0, "end")
            for k in want:
                self.listbox.selection_set(k)
        self.redrawMap()

    def onSelect(self, _e=None):
        self.pending = None
        self.aimPending = False
        sel = self.listbox.curselection()
        self.selected = self._rowMap[sel[0]] if sel and sel[0] < len(self._rowMap) else None
        self.showParams()
        self.redrawMap()

    # -- placing a motion: pick it, tap the map, see its path, then Apply --
    def pickMotion(self, kind):
        """A movement button. A motion is previewed before it becomes a
        block: tap the map and the path the robot would take shows as one
        dashed line; Enter / Apply adds it, Esc drops it. Everything else
        (wait, setPose, ...) is added straight away, as is anything while
        the path editor is open."""
        if self.pinEdit and kind == "goToPin":
            self.togglePinEdit()        # picking a pin needs the normal map mode
        self.pending = None
        if kind not in PLACED_ON_MAP or self.drawMode:
            self.addBlock(kind)
            return
        at = len(self.blocks) if self.selected is None else self.selected + 1
        if at < len(self.blocks) and self.blocks[at]["kind"] == "waitUntilDone":
            at += 1                 # never between a movement and its waitUntilDone
        b = new_block(kind)
        if kind == "goToPin":       # the claw is the robot's front: its tip's measured distance out
            b["reach"] = round(self.frontEdge(), 2)
        x, y, th = self._poseAt(at)
        if kind in HAS_POINT:
            b["x"], b["y"] = round(x, 1), round(y, 1)
        if kind in HAS_HEADING:
            b["theta"] = round(th, 1)
        self.pending = (b, at)
        self.pendingPin = None      # goToPin: the pin tapped so far (index into PINS)
        self.pendingGoal = None     # shootAtGoal: the goal tapped so far (index into GOALS)
        self.showParams()           # its parameters are editable straight away
        if kind == "goToPin":
            self.mapStatus.config(text="goToPin: tap a pin (the rings) to pick it -- taps "
                                       "anywhere else do nothing  --  Enter = Apply, Esc = cancel")
        elif kind == "shootAtGoal":
            self.mapStatus.config(text="shoot at goal: tap a goal (the rings), then tap the side of "
                                       "it to shoot from  --  Enter = Apply, Esc = cancel")
        else:
            self.mapStatus.config(text=f"{kind}: tap the map to place it"
                                       + (" (drag to turn it)" if kind == "moveToPose" else "")
                                       + "  --  Enter = Apply, Esc = cancel")
        self.redrawMap()

    def _poseAt(self, at):
        """Where the robot is as the block at index `at` would start."""
        sim = self.simulation()
        return sim[at - 1][0] if 0 < at <= len(sim) else tuple(self.start)

    def _placePending(self, ex, ey, first):
        """A tap (first) or drag on the map while a motion is being placed."""
        b, at = self.pending
        k = b["kind"]
        x0, y0, th0 = self._poseAt(at)
        if k == "goToPin":                          # only a pin can be picked: other taps do nothing
            p = self.pinAt(ex, ey)
            if p is None or p == self.pendingPin:
                return
            self.pendingPin = p
            b["x"], b["y"] = PINS[p]
            self.syncPendingParams()
            self.mapStatus.config(text=f"pin at ({b['x']}, {b['y']}) picked -- Enter = Apply, "
                                       "tap another pin to change it, Esc = cancel")
            self.redrawMap()
            return
        if k == "shootAtGoal":                      # tap a goal, then tap a side of it
            g = self.goalAt(ex, ey)
            if g is not None and g != self.pendingGoal:
                self.pendingGoal = g
                b["x"], b["y"] = GOALS[g][0], GOALS[g][1]
                b["goalSide"] = self._facingSide(b, (x0, y0))      # the side the robot is nearest
            elif self.pendingGoal is not None:          # the same goal: where on it was the tap?
                gx, gy = self.toMap(b["x"], b["y"])
                ox, oy = ex - gx, gy - ey                # screen y points down, field y up
                if math.hypot(ox, oy) < 8:
                    return
                b["goalSide"] = ("RIGHT" if ox > 0 else "LEFT") if abs(ox) > abs(oy) \
                    else ("TOP" if oy > 0 else "BOTTOM")
            else:
                return
            self.syncPendingParams()
            self.mapStatus.config(text=f"goal at ({b['x']:g}, {b['y']:g}), shooting from its "
                                       f"{b['goalSide']} -- tap a side of it to change, "
                                       "Enter = Apply, Esc = cancel")
            self.redrawMap()
            return
        if k in self.AIM_KINDS:                     # aimed by direction
            px, py = self.toMap(x0, y0)
            if math.hypot(ex - px, ey - py) < 4:
                return
            b["theta"] = round(math.degrees(math.atan2(ex - px, py - ey)) % 360.0, 1)
        elif first or k != "moveToPose":            # a target point
            fx, fy = self.toField(ex, ey)
            b["x"], b["y"] = round(fx, 1), round(fy, 1)
            if k == "moveToPose":
                b["theta"] = round(heading_to(x0, y0, b["x"], b["y"]), 1)
        else:                                       # moveToPose: drag turns the finish heading
            px, py = self.toMap(b["x"], b["y"])
            if math.hypot(ex - px, ey - py) >= 4:
                b["theta"] = round(math.degrees(math.atan2(ex - px, py - ey)) % 360.0, 1)
        self._pendingSide()
        self.syncPendingParams()
        self.redrawMap()

    def _facingSide(self, b, pose):
        """The side of the goal the robot is on, as the default side to shoot from."""
        dx, dy = pose[0] - b["x"], pose[1] - b["y"]
        if abs(dx) > abs(dy):
            return "RIGHT" if dx > 0 else "LEFT"
        return "TOP" if dy > 0 else "BOTTOM"

    def _pinStop(self, b, pose):
        """Where the robot's centre ends a goToPin: `reach` short of the pin when it drives
        forwards at it, otherwise on the pin."""
        dx, dy = b["x"] - pose[0], b["y"] - pose[1]
        dist = math.hypot(dx, dy)
        reach = b.get("reach", 0.0)
        if b.get("forwards", True) and reach > 0 and dist > reach:
            return (b["x"] - dx / dist * reach, b["y"] - dy / dist * reach)
        return (b["x"], b["y"])

    DETOUR_KINDS = ("goToPin", "moveToPoint", "moveToPose")     # the blocks that drive somewhere

    def _pinDetour(self, b, pose):
        """The goal-avoiding waypoints in front of a drive ([] = straight is clear): Grab pin,
        moveToPoint and moveToPose (it is the straight way there that is checked)."""
        if not self.avoidVar.get() or b["kind"] not in self.DETOUR_KINDS:
            return []
        end = self._pinStop(b, pose) if b["kind"] == "goToPin" else (b["x"], b["y"])
        return avoid_route((pose[0], pose[1]), end, self.robotSize())

    def _shootPlan(self, b, pose):
        """(waypoints, pre, stand) of a shootAtGoal from `pose`."""
        clr = self.clearance()
        pre, stand = shoot_points((b["x"], b["y"]), b["goalSide"],
                                  shoot_standoff(b, self.robotSize(), self.frontEdge()), clr)
        wps = avoid_route((pose[0], pose[1]), pre, self.robotSize()) if self.avoidVar.get() else []
        return wps, pre, stand

    def _paintPendingShoot(self, b, pose):
        if self.pendingGoal is None:
            return
        c = self.map
        gx, gy = self.toMap(b["x"], b["y"])
        c.create_oval(gx - 14, gy - 14, gx + 14, gy + 14, outline=PREVIEW, width=3)
        for side, (dx, dy) in SIDE_DIR.items():         # the four sides, the chosen one lit
            on = side == b["goalSide"]
            hx, hy = gx + dx * 26, gy - dy * 26
            c.create_oval(hx - 6, hy - 6, hx + 6, hy + 6, outline=PREVIEW, width=2,
                          fill=PREVIEW if on else "")
        wps, pre, stand = self._shootPlan(b, pose)
        self.drawTrace([(pose[0], pose[1])] + wps + [pre, stand], PREVIEW, 3, (6, 3), arrow=False)
        sx, sy = self.toMap(*stand)
        c.create_oval(sx - 7, sy - 7, sx + 7, sy + 7, outline=PREVIEW, width=2)
        c.create_line(sx, sy, gx, gy, fill=PREVIEW, width=2, arrow="last")      # it faces the goal

    def _pendingSide(self):
        """Lock the side a pending swing turns towards (the other one if it
        goes backwards)."""
        b, at = self.pending
        if not b["kind"].startswith("swing"):
            return
        x0, y0, th0 = self._poseAt(at)
        target = b["theta"] if b["kind"] == "swingToHeading" else \
            heading_to(x0, y0, b["x"], b["y"], b.get("forwards", True))
        err = angle_err(target, th0, b.get("direction", "AUTO"))
        if abs(err) > 0.5:
            b["side"] = swing_side(err, b.get("forwards", True))

    def _paintPending(self):
        """The motion being placed: just its path (dashed, in its own colour)
        and a ring on where it goes. No handles, nothing else."""
        b, at = self.pending
        pose = self._poseAt(at)
        if b["kind"] == "shootAtGoal":
            self._paintPendingShoot(b, pose)
            return
        detour = []
        if b["kind"] == "goToPin":      # (redrawMap drew the rings); the route only once picked
            if self.pendingPin is None:
                return
            self.drawPinRing(b["x"], b["y"], PREVIEW)
        detour = self._pinDetour(b, pose)
        if detour:                      # round the goals, then straight in
            end = self._pinStop(b, pose) if b["kind"] == "goToPin" else (b["x"], b["y"])
            self.drawTrace([(pose[0], pose[1])] + detour + [end], PREVIEW, 3, (6, 3), arrow=False)
        shape = simulate([b], pose)[0][1]
        c = self.map
        if shape and shape[0] == "trace" and not detour:
            self.drawTrace(shape[1], PREVIEW, 3, (6, 3), arrow=False)
        elif shape and shape[0] == "swing":         # a line and one box where it ends
            self.drawTrace(shape[5], PREVIEW, 3, (6, 3), arrow=False)
            self.drawSwingEnd(shape, PREVIEW)
        elif shape and shape[0] == "turn":
            _, x, y, a0, a1 = shape[:5]
            self.drawTurnArc(x, y, a0, a1, PREVIEW, 3, swing=False)
        if b["kind"] in HAS_POINT:
            px, py = self.toMap(b["x"], b["y"])
            c.create_oval(px - 7, py - 7, px + 7, py + 7, outline=PREVIEW, width=2)
            self.drawExitRing(b, shape)
            if b["kind"] == "moveToPose":           # facing there, and the arc's lead point
                rad = math.radians(b["theta"])
                c.create_line(px, py, px + math.sin(rad) * 24, py - math.cos(rad) * 24,
                              fill=PREVIEW, width=2, arrow="last")
                self.drawLeadHandle(b, pose, PREVIEW)
            elif b["kind"] in self.AIM_POINT_KINDS:
                x0, y0 = self.toMap(pose[0], pose[1])
                c.create_line(x0, y0, px, py, fill=PREVIEW, width=1, dash=(2, 4))
        self.drawShapeChips(b, pose)        # the same settings chips an added swing has

    # -- the arc of a moveToPose --
    # Its curve is lemlib's boomerang: the robot chases a "carrot" that sits
    # behind the target along its finishing heading, lead * distance away.
    # So two points shape the arc, like the handles of a bezier: the
    # heading knob out the front of the target (the way it ends up
    # facing) and the lead point behind it (how wide the curve swings).
    def leadHandle(self, b, pose0):
        """Map position of the lead point of moveToPose block b."""
        tth = b["theta"] if b.get("forwards", True) else b["theta"] + 180
        dist = math.hypot(b["x"] - pose0[0], b["y"] - pose0[1])
        r, reach = math.radians(tth), b.get("lead", 0.6) * dist
        return self.toMap(b["x"] - reach * math.sin(r), b["y"] - reach * math.cos(r))

    def setLead(self, b, pose0, ex, ey):
        """Slide the lead point to the mouse, along the heading line."""
        dist = math.hypot(b["x"] - pose0[0], b["y"] - pose0[1])
        if dist < 1.0:
            return
        tth = b["theta"] if b.get("forwards", True) else b["theta"] + 180
        mx, my = self.toField(ex, ey)
        r = math.radians(tth)
        back = -((mx - b["x"]) * math.sin(r) + (my - b["y"]) * math.cos(r))
        b["lead"] = round(max(0.0, min(1.0, back / dist)), 2)

    def drawLeadHandle(self, b, pose0, color):
        hx, hy = self.leadHandle(b, pose0)
        tx, ty = self.toMap(b["x"], b["y"])
        self.map.create_line(tx, ty, hx, hy, fill=color, width=1, dash=(2, 3))
        self.map.create_oval(hx - 5, hy - 5, hx + 5, hy + 5, fill=color, outline="")

    def _hitLeadKnob(self, ex, ey):
        """Index of the selected moveToPose whose lead point is under the mouse."""
        i = self.selected
        if i is None or i >= len(self.blocks) or self.blocks[i]["kind"] != "moveToPose":
            return None
        hx, hy = self.leadHandle(self.blocks[i], self.poseBefore(i))
        return i if math.hypot(hx - ex, hy - ey) <= self.HIT_PX else None

    def _hitPendingKnob(self, ex, ey):
        """"lead" or "hdg" when the mouse is on a handle of the moveToPose
        being placed, else None."""
        b, at = self.pending
        if b["kind"] != "moveToPose":
            return None
        hx, hy = self.leadHandle(b, self._poseAt(at))
        if math.hypot(hx - ex, hy - ey) <= self.HIT_PX:
            return "lead"
        tx, ty = self.toMap(b["x"], b["y"])
        rad = math.radians(b["theta"])
        if math.hypot(tx + math.sin(rad) * 24 - ex, ty - math.cos(rad) * 24 - ey) <= self.HIT_PX:
            return "hdg"
        return None

    def applyAction(self):
        """Enter / the Apply button: add the motion being placed, or else
        turn the drawn path into blocks."""
        if self.pending:
            self.applyPending()
        elif self.mergeMode:
            self.toggleMerge()
        elif self.drawMode:
            self.applyDrawnPath()

    def applyPending(self):
        b, at = self.pending
        if b["kind"] == "goToPin" and self.pendingPin is None:
            self.mapStatus.config(text="goToPin: tap a pin first (the rings on the map)")
            return
        if b["kind"] == "shootAtGoal" and self.pendingGoal is None:
            self.mapStatus.config(text="shoot at goal: tap a goal first (the rings on the map)")
            return
        pose = self._poseAt(at)
        self.pending = None
        self.pushUndo()
        if b["kind"] == "shootAtGoal":
            # one block that stays editable; it is expanded (round the goals, in, face, shoot)
            # wherever the robot has to run it
            self.blocks.insert(at, b)
            self.selected = at
            msg = "shoot at goal added -- its parameters stay editable"
        else:
            front = []
            # a drive goes round the goals first, then makes its last run in
            for p in self._pinDetour(b, pose):
                front += [new_block("moveToPoint", x=round(p[0], 1), y=round(p[1], 1),
                                    forwards=bool(b.get("forwards", True))),
                          new_block("waitUntilDone")]
            self.blocks[at:at] = front
            at += len(front)
            self.blocks.insert(at, b)
            # lemlib motions are asynchronous: each gets its waitUntilDone, as addBlock does
            # (goToPin blocks until it has the pin, so it needs none)
            if b["kind"] in MOTIONS:
                self.blocks.insert(at + 1, new_block("waitUntilDone"))
            self.selected = at
            self.autoTimeoutRange(at - len(front), at + 1)
            msg = f"{b['kind']} added" + (f" (+{len(front) // 2} to avoid goals)" if front else "")
        self.refreshList()
        self.showParams()
        self.mapStatus.config(text=msg)

    # -- automatic timeouts --
    def _autoTimeoutAt(self, i):
        b = self.blocks[i]
        if b["kind"] not in TIMED_KINDS:
            return
        b["timeout"] = auto_timeout_ms(b, self.poseBefore(i))

    def autoTimeoutRange(self, lo, hi):
        """Give blocks lo..hi-1 a timeout that suits them, in order (each one starts where the
        one before ends), when "Pick timeouts automatically" is on."""
        if not self.autoTimeoutVar.get():
            return
        for i in range(max(0, lo), min(hi, len(self.blocks))):
            self._autoTimeoutAt(i)

    def autoTimeoutAll(self):
        """The button: work out the timeout of every movement and Grab pin block in the plan."""
        if not self.blocks:
            return
        self.pushUndo()
        for i in range(len(self.blocks)):
            self._autoTimeoutAt(i)
        self.refreshList()
        self.showParams()
        self.mapStatus.config(text="timeouts set from the simulated time of each block")

    def cancelPending(self):
        self.pending = None
        self.mapStatus.config(text="")
        self.showParams()           # back to the selected block's
        self.redrawMap()

    def addBlock(self, kind):
        # New blocks start where the route currently ends, so the map shows
        # something sensible before you've typed anything.
        aims = kind in self.AIM_KINDS + self.AIM_POINT_KINDS
        if self.drawMode and aims and self.waypoints and len(self.pathPoints()) >= 2:
            # The points drawn so far are not blocks yet, so the swing would
            # have nothing to hang off and land on the robot's pose. Apply
            # them first: the swing then goes on the end of this path.
            self.applyDrawnPath()
        self.aimPending = self.drawMode and aims
        sim = self.simulation()
        end = sim[-1][0] if sim else tuple(self.start)
        self.pushUndo()
        b = new_block(kind)
        if kind in HAS_POINT:
            b["x"], b["y"] = round(end[0], 1), round(end[1], 1)
        if kind in HAS_HEADING:
            b["theta"] = round(end[2], 1)
        at = len(self.blocks) if self.selected is None else self.selected + 1
        if self.drawMode and aims:
            at = len(self.blocks)   # drawing continues from the route's end, so does this
        # ...but never between a movement and the waitUntilDone that
        # belongs to it.
        if at < len(self.blocks) and self.blocks[at]["kind"] == "waitUntilDone":
            at += 1
        self.blocks.insert(at, b)
        # lemlib runs motions asynchronously, so every movement gets a
        # waitUntilDone block after it -- delete it if you really do want
        # the next block to start while the robot is still moving.
        if kind in MOTIONS:
            self.blocks.insert(at + 1, new_block("waitUntilDone"))
        self.selected = at
        self.refreshList()
        self.showParams()
        if kind in self.AIM_KINDS + self.AIM_POINT_KINDS:
            what = "swing" if kind.startswith("swing") else "turn"
            self.mapStatus.config(text=f"Click on the map to aim this {what}"
                                       + (", drag to adjust" if kind in self.AIM_KINDS else "")
                                       + ("  (then keep drawing)" if self.drawMode else ""))

    # Every edit below takes an undo snapshot first, so Ctrl+Z covers block
    # edits as well as drawn paths.
    def moveBlock(self, delta):
        i = self.selected
        if i is None:
            return
        j = i + delta
        if 0 <= j < len(self.blocks):
            self.pushUndo()
            self.blocks[i], self.blocks[j] = self.blocks[j], self.blocks[i]
            self.selected = j
            self.refreshList()

    def duplicateBlock(self):
        if self.selected is None:
            return
        self.pushUndo()
        self.blocks.insert(self.selected + 1, dict(self.blocks[self.selected]))
        self.selected += 1
        self.refreshList()

    def deleteBlock(self):
        if self.selected is None or self.selected >= len(self.blocks):
            return
        self.pushUndo()
        del self.blocks[self.selected]
        self.selected = min(self.selected, len(self.blocks) - 1) if self.blocks else None
        self.refreshList()
        self.showParams()

    def clearBlocks(self):
        if self.blocks and not messagebox.askyesno(
                "Clear", "Remove every block?\n(Ctrl+Z brings them back.)", parent=self):
            return
        if self.pending:
            self.cancelPending()
        self.pushUndo()
        self.blocks = []
        self.selected = None
        self.refreshList()
        self.showParams()

    # -- aiming swings and turns on the map --
    AIM_KINDS = ("turnToHeading", "swingToHeading")          # aimed by direction
    AIM_POINT_KINDS = ("turnToPoint", "swingToPoint")        # aimed by a target point

    def aimable(self):
        """True when the selected block is a turn or swing, i.e. something a
        click or drag on the map aims. (While drawing a path a click normally
        drops a point; a swing just added in draw mode is "pending" and takes
        the next click -- see aimPending.)"""
        i = self.selected
        return (i is not None and i < len(self.blocks)
                and self.blocks[i]["kind"] in self.AIM_KINDS + self.AIM_POINT_KINDS)

    def aimVisible(self):
        """Whether the selected turn / swing shows its aim line and shape
        options. While drawing, only until you carry on drawing: once more
        points follow it the swing is no longer at the end of the path, and
        its controls left hanging at the old pose looked like it still was."""
        return self.aimable() and (not self.drawMode or self.aimPending or not self.waypoints)

    def poseBefore(self, i):
        """Where the robot is as block i starts. The simulated robot stops a
        little short of (or past) the point it drove to, so when it is
        essentially at the target of the movement just before, that target is
        used: the aim line then sits on the path's last point."""
        sim = self.simulation()
        if not 0 < i <= len(sim):
            return tuple(self.start)
        x, y, th = sim[i - 1][0]
        for j in range(i - 1, -1, -1):
            b = self.blocks[j]
            if b["kind"].startswith("swing"):
                break           # the robot is where the swing left it, not at an older target
            if b["kind"] in self.ROUTE_POINTS:
                if math.hypot(b["x"] - x, b["y"] - y) <= 6.0:
                    x, y = b["x"], b["y"]
                break
        return (x, y, th)

    def autoSide(self, i):
        """Pick which side a swing pivots on from the way it turns: a swing
        forwards turning clockwise locks the right side, counter-clockwise
        the left. Set when a swing is aimed on the map; the side box in the
        parameters still overrides it (e.g. for a swing in reverse)."""
        b = self.blocks[i]
        if not b["kind"].startswith("swing"):
            return
        x0, y0, th0 = self.poseBefore(i)
        if b["kind"] == "swingToHeading":
            target = b["theta"]
        elif (b["x"], b["y"]) != (x0, y0):
            target = heading_to(x0, y0, b["x"], b["y"], b.get("forwards", True))
        else:
            return
        err = angle_err(target, th0, b.get("direction", "AUTO"))
        if abs(err) > 0.5:
            b["side"] = swing_side(err, b.get("forwards", True))

    def _aimAt(self, i, ex, ey):
        """Point turnToHeading / swingToHeading block i at map position
        (ex, ey), as seen from where the robot is when the block starts."""
        x0, y0, _ = self.poseBefore(i)
        px, py = self.toMap(x0, y0)
        if math.hypot(ex - px, ey - py) < 4:
            return
        self.blocks[i]["theta"] = round(math.degrees(math.atan2(ex - px, py - ey)) % 360.0, 1)
        self.autoSide(i)

    def _aimKnob(self, i, pose0=None):
        """Map position of the knob on the aim line of heading block i, and
        of the robot it comes out of."""
        x0, y0, _ = pose0 or self.poseBefore(i)
        px, py = self.toMap(x0, y0)
        rad = math.radians(self.blocks[i]["theta"])
        reach = self.HEAD_LEN * 1.7
        return (px + math.sin(rad) * reach, py - math.cos(rad) * reach), (px, py)

    def _hitAimKnob(self, ex, ey):
        i = self.selected
        if not self.aimVisible() or self.blocks[i]["kind"] not in self.AIM_KINDS:
            return None
        (kx, ky), _ = self._aimKnob(i)
        return i if math.hypot(kx - ex, ky - ey) <= self.HIT_PX else None

    def drawShapeChips(self, b, pose0):
        """Small option chips beside the selected swing / turn: which side it
        pivots on, which way round it turns, and for a point, forwards or in
        reverse. lemlib's swing has no radius setting -- the arc comes from
        the pivot side (the robot turns about that wheel) and the direction --
        so these are the ways to change its curve. Clicking one sets the
        block's parameter and the arc redraws."""
        params = KIND_PARAMS[b["kind"]]
        rows = []
        if "side" in params:
            rows.append(("pivot", [("L", "side", "LEFT"), ("R", "side", "RIGHT")]))
        if "direction" in params:
            rows.append(("turn", [("auto", "direction", "AUTO"), ("cw", "direction", "CW"),
                                  ("ccw", "direction", "CCW")]))
        if "forwards" in params:
            rows.append(("drive", [("fwd", "forwards", True), ("back", "forwards", False)]))
        px, py = self.toMap(pose0[0], pose0[1])
        c = self.map
        h, gap = 20, 4
        widest = max(6 + sum(18 + 7 * len(t) + gap for t, _, _ in chips) for _, chips in rows)
        x0 = px + 40
        room = c.winfo_width() - self.ICON_MARGIN - 8      # keep clear of the icon column
        if x0 + widest > room:                             # no room on the right: go left
            x0 = max(46, px - 34 - widest)
        y = max(6, py - 12 - len(rows) * (h + gap))
        for label, chips in rows:
            c.create_text(x0, y + h / 2, text=label, anchor="e", fill="#cfcfcf",
                          font=("Segoe UI", 8))
            x = x0 + 6
            for text, name, value in chips:
                w = 18 + 7 * len(text)
                on = b.get(name) == value
                c.create_rectangle(x, y, x + w, y + h, fill=PATH_GREEN if on else "#383838",
                                   outline="#101010")
                c.create_text(x + w / 2, y + h / 2, text=text, fill="#101010" if on else "#e6e6e6",
                              font=("Segoe UI", 8, "bold"))
                self._chips.append(((x, y, x + w, y + h), name, value))
                x += w + gap
            y += h + gap

    def _chipAt(self, ex, ey):
        if not (self.pending or self.aimVisible()):
            return None
        for (x0, y0, x1, y1), name, value in self._chips:
            if x0 <= ex <= x1 and y0 <= ey <= y1:
                return name, value
        return None

    def _applyChip(self, chip):
        name, value = chip
        if self.pending:                    # a motion still being placed
            b = self.pending[0]
            if b.get(name) != value:
                b[name] = value
                if name == "forwards":
                    self._pendingSide()
                self.syncPendingParams()
                self.redrawMap()
            return
        b = self.blocks[self.selected]
        if b.get(name) == value:
            return
        self.pushUndo()
        b[name] = value
        if name == "forwards" and b["kind"].startswith("swing"):
            self.autoSide(self.selected)        # a backward swing locks the other side
        self.refreshList()
        self.showParams()

    def drawAimLine(self, i, pose0):
        """The aim line of the selected turn / swing: out of the robot's
        position as the block starts, along the heading it turns to, ending
        in a knob you can drag."""
        (kx, ky), (px, py) = self._aimKnob(i, pose0)
        self.map.create_line(px, py, kx, ky, fill=PATH_SEL, width=2, dash=(5, 3))
        self.map.create_oval(kx - 6, ky - 6, kx + 6, ky + 6, fill=PATH_SEL, outline=PURPLE,
                             width=2)

    # -- point headings (path.jerryio's per-point heading line) --
    ROUTE_POINTS = ("moveToPoint", "moveToPose", "setPose")
    HEADING_STEP = 15.0      # what the rotate buttons turn by, degrees

    def turnAfter(self, i):
        """Index of the turnToHeading that belongs to point block i (it
        follows the block, past its waitUntilDone), or None."""
        j = i + 1
        if j < len(self.blocks) and self.blocks[j]["kind"] == "waitUntilDone":
            j += 1
        if j < len(self.blocks) and self.blocks[j]["kind"] == "turnToHeading":
            return j
        return None

    def pointHeading(self, i):
        """(heading, set) of route point i. moveToPose / setPose carry their
        own theta; a moveToPoint is 'set' when a turnToHeading follows it,
        and otherwise shows the way the robot is facing as it arrives."""
        b = self.blocks[i]
        if b["kind"] in HAS_HEADING and not b["kind"].startswith("swing"):
            return b["theta"], True
        j = self.turnAfter(i)
        if j is not None:
            return self.blocks[j]["theta"], True
        sim = self.simulation()
        return (sim[i][0][2] if i < len(sim) else 0.0), False

    def setPointHeading(self, i, theta):
        """Give route point i a heading. moveToPose / setPose just take the
        theta; a moveToPoint gets a turnToHeading (and waitUntilDone) right
        after it, which turns the robot in place to that heading there."""
        theta = round(theta % 360.0, 1)
        b = self.blocks[i]
        if b["kind"] in HAS_HEADING and not b["kind"].startswith("swing"):
            b["theta"] = theta
            return
        j = self.turnAfter(i)
        if j is not None:
            self.blocks[j]["theta"] = theta
            return
        at = i + 1
        if at >= len(self.blocks) or self.blocks[at]["kind"] != "waitUntilDone":
            self.blocks.insert(at, new_block("waitUntilDone"))
        self.blocks.insert(at + 1, new_block("turnToHeading", theta=theta))
        self.blocks.insert(at + 2, new_block("waitUntilDone"))

    def rotatePoint(self, delta):
        """The rotate buttons: turn the selected point's heading by delta."""
        w = self.curWp()
        if w is not None:
            self.setWpHeading(w, self.wpHeading(w)[0] + delta)
            self.syncPointPanel()
            self.redrawMap()
            return
        i = self.selected
        if i is None or i >= len(self.blocks) or self.blocks[i]["kind"] not in self.ROUTE_POINTS:
            return
        self.pushUndo()
        self.setPointHeading(i, self.pointHeading(i)[0] + delta)
        self.refreshList()
        self.showParams()

    def syncPointPanel(self):
        """Fill the point panel from the selected route point (blank if the
        selection isn't one)."""
        w = self.curWp()
        if w is not None:
            wp = self.waypoints[w]
            for v, val in zip(self.ptVars, (wp[0], wp[1], self.wpHeading(w)[0])):
                v.set(fmt(round(val, 2)))
            return
        i = self.selected
        if i is None or i >= len(self.blocks) or self.blocks[i]["kind"] not in self.ROUTE_POINTS:
            for v in self.ptVars:
                v.set("")
            return
        b = self.blocks[i]
        for v, val in zip(self.ptVars, (b["x"], b["y"], self.pointHeading(i)[0])):
            v.set(fmt(round(val, 2)))

    def applyPointPanel(self):
        w = self.curWp()
        if w is not None:
            try:
                x, y, h = (float(v.get()) for v in self.ptVars)
            except ValueError:
                return self.syncPointPanel()
            wp = self.waypoints[w]
            if (wp[0], wp[1], round(h % 360.0, 1)) == (
                    x, y, round(self.wpHeading(w)[0] % 360.0, 1)):
                return                   # nothing changed
            hdg = tuple(wp[3:])
            self.waypoints[w] = (x, y, wp[2] if len(wp) > 2 else CURVE) + hdg
            if round(h % 360.0, 1) != round(self.wpHeading(w)[0] % 360.0, 1):
                self.setWpHeading(w, h)
            self.redrawMap()
            return
        i = self.selected
        if i is None or i >= len(self.blocks) or self.blocks[i]["kind"] not in self.ROUTE_POINTS:
            return
        try:
            x, y, h = (float(v.get()) for v in self.ptVars)
        except ValueError:
            return self.syncPointPanel()
        b = self.blocks[i]
        if (b["x"], b["y"], round(h % 360.0, 1)) == (x, y, round(self.pointHeading(i)[0] % 360.0, 1)):
            return                       # nothing changed (FocusOut fires on every tab through)
        self.pushUndo()
        b["x"], b["y"] = x, y
        self.setPointHeading(i, h)
        self.refreshList()
        self.showParams()

    # -- parameter panel --
    def showParams(self):
        self.syncPointPanel()
        for w in self.paramFrame.winfo_children():
            w.destroy()
        self.paramWidgets = {}
        if self.pending:        # a motion still being placed: tune it before Apply
            b = self.pending[0]
            self.paramFrame.config(text=f" new {b['kind']}  (not applied yet) ")
        elif self.selected is None or self.selected >= len(self.blocks):
            tk.Label(self.paramFrame, text="select a block (or add one)", fg="#777",
                     bg=PANEL_BG).grid(row=0, column=0, padx=6, pady=4)
            self.codeLabel.config(text="")
            return
        else:
            b = self.blocks[self.selected]
            self.paramFrame.config(text=f"{self.selected + 1}. {b['kind']}")
        col = 0
        for name in KIND_PARAMS[b["kind"]]:
            if name == "waitLift" and b.get("position") == "DROP":
                continue                # a drop always waits: the claws open on arrival
            default, kind = PARAM_SPEC[name]
            tk.Label(self.paramFrame, text=name, fg="#aaa", bg=PANEL_BG,
                     font=("Segoe UI", 8)).grid(row=(col // 5) * 2, column=col % 5, sticky="w",
                                                padx=6)
            if kind == "bool":
                var = tk.BooleanVar(value=bool(b[name]))
                w = tk.Checkbutton(self.paramFrame, variable=var, bg=PANEL_BG,
                                   text="forwards" if var.get() else "backwards",
                                   fg="#ddd", selectcolor=INPUT_BG,
                                   command=lambda n=name: self.paramChanged(n))
            elif isinstance(kind, list):
                var = tk.StringVar(value=str(b[name]))
                w = ttk.Combobox(self.paramFrame, textvariable=var, values=kind, width=7,
                                 state="readonly")
                w.bind("<<ComboboxSelected>>", lambda _e, n=name: self.paramChanged(n))
            else:
                var = tk.StringVar(value=fmt(b[name]))
                w = tk.Entry(self.paramFrame, textvariable=var, width=9, font=("Consolas", 10))
                w.bind("<Return>", lambda _e, n=name: self.paramChanged(n))
                w.bind("<FocusOut>", lambda _e, n=name: self.paramChanged(n))
            w.grid(row=(col // 5) * 2 + 1, column=col % 5, sticky="w", padx=6, pady=(0, 4))
            self.paramWidgets[name] = var
            col += 1
        self.codeLabel.config(text=block_code(b))

    def syncPendingParams(self):
        """Put the pending block's values into the parameter boxes in place
        (tapping or dragging on the map moves them) without rebuilding the
        panel, which would steal the focus from a box being typed in."""
        if not self.pending:
            return
        b = self.pending[0]
        for name, var in self.paramWidgets.items():
            if isinstance(var, tk.BooleanVar):
                var.set(bool(b[name]))
            elif PARAM_SPEC[name][1] in ("num", "int"):
                var.set(fmt(b[name]))
            else:
                var.set(str(b[name]))
        for w in self.paramFrame.winfo_children():
            if isinstance(w, tk.Checkbutton) and "forwards" in b:
                w.config(text="forwards" if b["forwards"] else "backwards")
        self.codeLabel.config(text=block_code(b))

    def paramChanged(self, name):
        if self.pending:
            return self.pendingParamChanged(name)
        if self.selected is None:
            return
        b = self.blocks[self.selected]
        var = self.paramWidgets[name]
        _, kind = PARAM_SPEC[name]
        try:
            if kind == "bool":
                new = bool(var.get())
            elif isinstance(kind, list):
                new = var.get()
            elif kind == "int":
                new = int(float(var.get()))
            else:
                new = float(var.get())
        except (ValueError, tk.TclError):
            return
        if new == b[name]:
            return          # FocusOut fires on every tab through a field: nothing changed
        self.pushUndo()
        b[name] = new
        if name in ("x", "y", "theta", "maxSpeed", "minSpeed", "forwards", "reach", "decelRange",
                    "decelSpeed", "direction", "side") and self.autoTimeoutVar.get() \
                and "timeout" in b:
            self._autoTimeoutAt(self.selected)
            if "timeout" in self.paramWidgets:
                self.paramWidgets["timeout"].set(str(b["timeout"]))
        self.codeLabel.config(text=block_code(b))
        if name == "position":
            self.showParams()           # LIFT / DROP show different parameters
        # flipping a swing between forwards and backwards locks the other side
        if name == "forwards" and b["kind"].startswith("swing"):
            self.autoSide(self.selected)
            self.paramWidgets["side"].set(b["side"])
        # forwards checkbox text follows its value
        if kind == "bool":
            for w in self.paramFrame.winfo_children():
                if isinstance(w, tk.Checkbutton):
                    w.config(text="forwards" if b[name] else "backwards")
        self.refreshList()

    def pendingParamChanged(self, name):
        """A parameter box edited while a motion is still being placed: it
        goes into the pending block and the preview redraws. No undo step --
        nothing is in the plan yet."""
        b = self.pending[0]
        var = self.paramWidgets[name]
        _, kind = PARAM_SPEC[name]
        try:
            if kind == "bool":
                new = bool(var.get())
            elif isinstance(kind, list):
                new = var.get()
            elif kind == "int":
                new = int(float(var.get()))
            else:
                new = float(var.get())
        except (ValueError, tk.TclError):
            return
        if new == b[name]:
            return
        b[name] = new
        if name == "forwards":
            self._pendingSide()
        self.syncPendingParams()
        self.redrawMap()

    # -- robot --
    # Uploading is a burst of lines (CLEAR, one ADD per block) and over the
    # controller's radio a line can go missing or arrive late. So instead
    # of firing PLAN RUN blindly right after, wait until the brain reports
    # (in every RUI line) that it holds exactly our number of blocks, then
    # send the run. If the count doesn't settle, resend once, then give up
    # with a message instead of silently doing nothing.
    UPLOAD_TIMEOUT_S = 6.0
    UPLOAD_RETRIES = 2

    def uploadPlan(self):
        self.link._send("PLAN CLEAR")
        for b in self._uploadList:
            self.link._send(block_to_plan_add(b))
        self.link._send("NOP")   # chaser: nudges a stuck last line through the radio
        self.uploadStarted = time.time()

    def startRun(self, which):
        """which: None = all, int = that block."""
        if not self.blocks:
            return
        if self.driveMode:      # the motion is about to take the drivetrain
            self.stopDrive()
        if which is None:
            warnings = self.planWarnings()
            if warnings and not messagebox.askokcancel(
                    "Check plan", "Possible problems:\n\n" + "\n".join("- " + t for t in warnings)
                    + "\n\nRun it anyway?", icon="warning", parent=self):
                return
        flat, spans = self.flatPlan()
        self._runWhich = "ALL" if which is None else which
        self._uploadList, self._uploadSpans = flat, spans
        self.pendingRun = "ALL" if which is None else which
        if which is not None and 0 <= which < len(spans):
            lo, hi = spans[which]
            if hi - lo > 1:             # a Shoot at goal: upload just its blocks and run them all
                self._uploadList = flat[lo:hi]
                self.pendingRun = "ALL"
            else:
                self.pendingRun = lo
        self.uploadTries = 1
        if which is None and self.setStartVar.get():
            # The route is in field coordinates, so it only matches the map if
            # the robot starts where the map says it does. Without this the
            # run starts from whatever pose the robot's odometry has -- the
            # drawn start pose is never sent -- and every point is driven to
            # from the wrong place.
            self.link._send(f"POSE {self.start[0]:.2f} {self.start[1]:.2f} {self.start[2]:.2f}")
        self.uploadPlan()
        self.brainStatus.config(text=f"uploading {len(self._uploadList)} blocks to the robot...",
                                fg="#FF8C00")

    def startOffset(self):
        """(inches, degrees) between the robot's live pose and the plan's start
        pose, when that is enough to matter; None if it is close or unknown."""
        if not self.robotPose:
            return None
        d = math.hypot(self.robotPose[0] - self.start[0], self.robotPose[1] - self.start[1])
        a = abs((self.robotPose[2] - self.start[2] + 180) % 360 - 180)
        return (d, a) if d > 2.0 or a > 5.0 else None

    def runAll(self):
        self.startRun(None)

    def runSelected(self):
        if self.selected is not None:
            self.startRun(self.selected)

    def checkPendingRun(self, count):
        if self.pendingRun is None:
            return
        if count == len(self._uploadList):
            cmd = "PLAN RUN" if self.pendingRun == "ALL" else f"PLAN RUN {self.pendingRun}"
            self.link._send(cmd)
            self.link._send("NOP")
            self.pendingRun = None
            self.brainStatus.config(text="run sent", fg="#2ECC71")
        elif time.time() - self.uploadStarted > self.UPLOAD_TIMEOUT_S:
            if self.uploadTries < self.UPLOAD_RETRIES:
                self.uploadTries += 1
                self.uploadPlan()
                self.brainStatus.config(
                    text=f"robot has {count}/{len(self._uploadList)} blocks -- resending", fg="#FF8C00")
            else:
                self.pendingRun = None
                self.statusHold = time.time() + 8   # keep the message readable
                self.brainStatus.config(
                    text=f"upload failed: robot has {count}/{len(self._uploadList)} blocks. "
                         "Is it running this firmware? (check --debug output)", fg="#FF6B81")

    def _endRun(self, t, status):
        if self._stepT0:
            self._stepTimes[self._stepT0[0]] = t - self._stepT0[1]
            self._stepPoses[self._stepT0[0]] = self.robotPose
        seconds = t - self._runT0
        self._runT0 = self._stepT0 = self._idleSince = None
        self._awaitLog = time.time()        # the robot should now send its recording
        if seconds >= 0.3:          # nothing real runs for less: that was the plan being cleared
            self.recordRun(seconds, status)

    def recordRun(self, seconds, status):
        """The robot's plan just stopped RUNNING. A whole-plan run sets the OVERALL time (the big
        number, the history and the per-block times in the list); a run of one block only is
        noted under it without replacing it."""
        done = status == "DONE"
        which = self._runWhich
        self._runWhich = None
        if which not in (None, "ALL"):                   # one block (or one Shoot at goal)
            self.actualDetail.config(text=f"{self.actualDetail.cget('text').split('  |  ')[0]}  |  "
                                          f"last single-block run: {seconds:.2f} s")
            return
        if done:
            self.runHistory.append(seconds)
            self.runHistory = self.runHistory[-10:]
        self.actualLabel.config(text=f"{seconds:.2f} s",
                                fg="#2ECC71" if done and seconds <= AUTO_PERIOD_S else "#FF6B81")
        bits = ["overall, whole plan" if done else f"overall -- stopped early at {seconds:.1f} s"]
        if which is None:
            bits.append("started on the brain")
        elif done and self._uploadList:
            est = estimate_seconds(self._uploadList, self.start)[0]
            bits.append(f"estimate was {est:.1f} s ({seconds - est:+.1f} s)")
        if done and seconds > AUTO_PERIOD_S:
            bits.append(f"{seconds - AUTO_PERIOD_S:.1f} s over a {AUTO_PERIOD_S:.0f} s autonomous")
        learned = 0
        if done and which == "ALL" and self._uploadList and self._uploadSpans and \
                len(self._uploadList) == self._uploadSpans[-1][1]:
            learned = update_calibration(self._uploadList, self.start, self._stepTimes)
        if learned:
            bits.append(f"timing calibrated from {learned} moves: drive x{drive_scale():.2f}")
        if len(self.runHistory) > 1:
            bits.append(f"{len(self.runHistory)} runs: best {min(self.runHistory):.2f} s, "
                        f"average {sum(self.runHistory) / len(self.runHistory):.2f} s")
        bits.append("timed from the laptop, +-0.1 s")
        self.actualDetail.config(text=",  ".join(bits))
        # how long each block of the plan took, shown in the list until the plan is edited
        if which == "ALL" and self._uploadSpans:
            per = {}
            for bi, (lo, hi) in enumerate(self._uploadSpans):
                t = sum(self._stepTimes.get(k, 0.0) for k in range(lo, hi))
                if t > 0:
                    per[bi] = t
            drift = {}
            if self._uploadList and len(self._uploadList) == self._uploadSpans[-1][1]:
                for j, d in step_drift(self._uploadList, self.start, self._stepPoses).items():
                    for bi, (lo, hi) in enumerate(self._uploadSpans):
                        if lo <= j < hi:
                            drift[bi] = d       # the block's last motion wins
            self._blockDriftData = drift
            self._blockTimesData, self._blockTimesFor = per, repr(self.blocks)
            self.refreshList()

    def stop(self):
        self.link._send("PLAN STOP")

    def applyState(self, state):
        """Called by remote_touch with every parsed RUI state."""
        self.onBrainScreen = state["screen"] == "PLANNER"
        plot = state.get("plot")
        if plot:
            self.robotPose = (plot["poseX"], plot["poseY"], plot["poseTheta"])
        if self.onBrainScreen:
            # the brain empties its path on a new run or a pose reset; keeping
            # the old one would leave last run's line lying under the new one
            self.robotPath = state.get("path") or []
        elif state.get("path"):
            self.robotPath = state["path"]
        planKnown = bool(state.get("plan"))     # a line without the plan field says nothing about it
        status, idx, count = state.get("plan") or (self.planStatus[0], self.planStatus[1],
                                                    self.planStatus[2])
        now = time.time()
        if planKnown:
            if status == "RUNNING":
                self._idleSince = None
                if self._runT0 is None:                       # a run begins
                    self._runT0 = now
                    self._stepT0, self._stepTimes, self._stepPoses = (idx, now), {}, {}
                elif self._stepT0 and idx != self._stepT0[0]:
                    self._stepTimes[self._stepT0[0]] = now - self._stepT0[1]
                    self._stepPoses[self._stepT0[0]] = self.robotPose
                    self._stepT0 = (idx, now)
            elif self._runT0 is not None:                     # it stopped RUNNING
                if status in ("DONE", "CANCELLED"):
                    self._endRun(now, status)
                else:
                    # IDLE: the plan was cleared, or a stray frame. Only believe it once it has
                    # lasted a moment -- a blip must not end the run and time it at 0.1 s
                    self._idleSince = self._idleSince or now
                    if now - self._idleSince > 1.0:
                        self._endRun(self._idleSince, "CANCELLED")
        self.planStatus = (status, idx, count)
        self.checkPendingRun(count)
        # The robot sends a state 20 times a second whether or not anything
        # moved; redrawing an identical map just makes it shimmer.
        sig = (None if not self.robotPose else
               (round(self.robotPose[0], 1), round(self.robotPose[1], 1),
                round(self.robotPose[2] * 2) / 2),
               len(self.robotPath), status, idx, count, self.onBrainScreen)
        unchanged = sig == self._lastState
        self._lastState = sig
        if self.pendingRun is not None or time.time() < self.statusHold:
            if not unchanged:
                self.redrawLive()
            return
        if self.onBrainScreen:
            txt = f"brain: PLANNER  {status}  ({count} block{'' if count == 1 else 's'} on robot)"
            if idx >= 0:
                txt += f"  step {idx + 1}/{count}"
            off = self.startOffset()
            if off and status != "RUNNING":
                txt += f"   [robot is {off[0]:.1f} in / {off[1]:.0f} deg off the start pose]"
            if status == "RUNNING" and self._uploadSpans and len(self._uploadList) == \
                    (self._uploadSpans[-1][1] if self._uploadSpans else 0):
                for bi, (lo, hi) in enumerate(self._uploadSpans):   # robot step -> plan block
                    if lo <= idx < hi and bi in self._rowMap:
                        row = self._rowMap.index(bi)
                        self.listbox.selection_clear(0, "end")
                        self.listbox.selection_set(row)
                        self.listbox.see(row)
                        break
            self.brainStatus.config(text=txt, fg={"RUNNING": "#2ECC71", "CANCELLED": "#FF6B81"}
                                    .get(status, "#00F0FF"))
        else:
            self.brainStatus.config(text=f"brain: on {state['screen']} -- tap PATH PLANNER on the "
                                    "brain (or mirror) to run from here", fg="#FF8C00")
        if not unchanged:
            # a drawing that has no blocks or placed robot behind it starts at
            # the live robot, so that one does need the whole route repainted
            if self.waypoints and not self.blocks and not self.startPlaced:
                self.redrawMap()
            else:
                self.redrawLive()

    # -- files --
    def exportCode(self):
        self.applyStartPose()
        flat, _spans = self.flatPlan()
        code = export_code(flat, self.start, self.calibrateVar.get())
        with open(EXPORT_PATH, "w", encoding="utf-8") as f:
            f.write(code)
        self.clipboard_clear()
        self.clipboard_append(code)
        win = tk.Toplevel(self)
        win.title("Exported code")
        txt = tk.Text(win, width=100, height=min(30, len(flat) + 4), font=("Consolas", 10),
                      bg=INPUT_BG, fg="#2ECC71")
        txt.pack(fill="both", expand=True)
        txt.insert("1.0", code)
        tk.Label(win, text=f"saved to {EXPORT_PATH} and copied to the clipboard",
                 anchor="w").pack(fill="x")

    def savePlan(self):
        os.makedirs(PLANS_DIR, exist_ok=True)
        if self.editFunc:
            # editing an existing function: save over it, no file dialog. Reuse the
            # plan file that already maps to that function, else name one after it.
            path = os.path.join(PLANS_DIR, self.editFunc + ".json")
            for fn in os.listdir(PLANS_DIR):
                stem, ext = os.path.splitext(fn)
                if ext == ".json" and auto_function_name(stem) == self.editFunc:
                    path = os.path.join(PLANS_DIR, fn)
                    break
        else:
            path = filedialog.asksaveasfilename(parent=self, initialdir=PLANS_DIR,
                                                defaultextension=".json",
                                                filetypes=[("plan", "*.json")])
        if not path:
            return
        with open(path, "w", encoding="utf-8") as f:
            json.dump({"start": self.start, "blocks": self.blocks,
                       "waypoints": [[p[0], p[1], p[2] if len(p) > 2 else CURVE] + list(p[3:4])
                                     for p in self.waypoints]}, f, indent=2)
        self.planName = os.path.basename(path)
        self._retitle()
        self._markSaved()
        # ...and the plan as a function in src/autonomous.cpp, named after the file
        if not self.blocks:
            messagebox.showinfo("Save", "Plan saved. It has no blocks, so no function was "
                                "written to autonomous.cpp.", parent=self)
            return
        try:
            name, replaced = write_auto_function(
                self.editFunc or auto_function_name(os.path.splitext(os.path.basename(path))[0]),
                self.flatPlan()[0], self.start)
        except OSError as err:
            messagebox.showerror("Save", f"Plan saved, but autonomous.cpp couldn't be "
                                 f"updated:\n{err}", parent=self)
            return
        messagebox.showinfo(
            "Save", f"Plan saved.\n\n{'Updated' if replaced else 'Added'} void {name}() in "
                    "src/autonomous.cpp (declared in include/autonomous.hpp).\n\n"
                    f"Call {name}() from runAutonomous() to run it.", parent=self)

    def loadPlan(self):
        os.makedirs(PLANS_DIR, exist_ok=True)
        path = filedialog.askopenfilename(parent=self, initialdir=PLANS_DIR,
                                          filetypes=[("plan", "*.json")])
        if not path:
            return
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        self.editFunc = None
        if self.pending:
            self.cancelPending()
        self._installPlan(data, os.path.basename(path))

    def editAuto(self):
        """Pick a planner-written function in autonomous.cpp and load it back into
        the planner. Saving then rewrites that same function in place."""
        names = list_planner_functions()
        if not names:
            messagebox.showinfo("Edit auto", "No planner-written functions found in "
                                "src/autonomous.cpp.", parent=self)
            return
        win = tk.Toplevel(self)
        win.title("Edit autonomous function")
        win.transient(self)
        win.grab_set()
        tk.Label(win, text="Pick a function to edit (double-click):").pack(anchor="w", padx=8,
                                                                         pady=(8, 2))
        lb = tk.Listbox(win, height=min(14, len(names)), width=36, bg=INPUT_BG, fg="#ddd",
                        exportselection=False)
        lb.pack(fill="both", expand=True, padx=8)
        for n in names:
            lb.insert("end", n)
        lb.selection_set(0)

        def choose(_e=None):
            sel = lb.curselection()
            if not sel:
                return
            name = names[sel[0]]
            win.destroy()
            self._openAuto(name)

        lb.bind("<Double-Button-1>", choose)
        lb.bind("<Return>", choose)
        bar = tk.Frame(win)
        bar.pack(fill="x", padx=8, pady=8)
        tk.Button(bar, text="Edit", command=choose).pack(side="left")
        tk.Button(bar, text="Cancel", command=win.destroy).pack(side="left", padx=(6, 0))
        lb.focus_set()

    def _openAuto(self, name):
        try:
            start, blocks, skipped = read_auto_function(name)
        except (OSError, KeyError) as err:
            messagebox.showerror("Edit auto", f"Couldn't read {name}() from autonomous.cpp:\n{err}",
                                 parent=self)
            return
        self.editFunc = name
        self._installPlan({"start": start, "blocks": blocks}, f"{name}()  [autonomous.cpp]",
                          prebuilt=True)
        if skipped:
            messagebox.showwarning(
                "Edit auto", f"{len(skipped)} line(s) in {name}() weren't planner blocks "
                "(hand-edited?) and were left out. Saving will drop them:\n\n"
                + "\n".join(skipped[:8]) + ("\n..." if len(skipped) > 8 else ""), parent=self)

    def _installPlan(self, data, name, prebuilt=False):
        """Make `data` ({"start", "blocks", "waypoints"}) the plan being edited."""
        self.start = list(data.get("start", [0, 0, 0]))
        for v, val in zip(self.startVars, self.start):
            v.set(fmt(val))
        self.blocks = [b if prebuilt else
                       new_block(b["kind"], **{k: v for k, v in b.items() if k != "kind"})
                       for b in data.get("blocks", []) if b.get("kind") in KINDS]
        # saved control points belong to a route with no blocks behind it; with
        # blocks loaded a drawing is anchored on their end instead
        self.waypoints = [] if self.blocks else \
            [(float(p[0]), float(p[1]),
              p[2] if len(p) > 2 and p[2] in (CURVE, STRAIGHT) else CURVE) + tuple(p[3:4])
             for p in data.get("waypoints", [])]
        self.undoStack = []
        self.redoStack = []
        self.replaceOnApply = False
        self.startPlaced = True          # a loaded plan's start pose is where the robot goes
        self.planName = name
        self._retitle()
        self.selected = 0 if self.blocks else None
        self.refreshList()
        self.showParams()
        self._markSaved()
