"""Open the path planner on its own, with no robot connected.

    python tools/path_planner_offline.py

Everything on the map works -- placing motions, the previews, swings, curves,
save / load / export. Only what needs the brain (Run, Send to robot, Drive)
does nothing, since the commands go nowhere.
"""
import os
import sys
import tkinter as tk

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from path_planner import PlannerWindow  # noqa: E402


class OfflineLink:
    """Stands in for the serial link to the brain: every command is dropped."""

    def _send(self, *_args, **_kwargs):
        pass


def main():
    root = tk.Tk()
    root.withdraw()                 # the planner is its own window
    win = PlannerWindow(root, OfflineLink())
    win.title("Path Planner -- offline (no robot)")
    win.protocol("WM_DELETE_WINDOW", root.destroy)
    root.mainloop()


if __name__ == "__main__":
    main()
