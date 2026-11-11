"""Shared "Reconnect" button for every remote_touch window.

remote_touch.RemoteTouchApp sets `win.onReconnect` on each tool window it
opens; the button just calls it. Run offline (no app, no robot) the window
has no onReconnect and the click says so instead of doing nothing."""
import tkinter as tk


def add_reconnect_button(win, parent, status=None, **pack):
    """Put a Reconnect button in `parent`. `status` is an optional
    Label-like widget (anything with .config(text=...)) used for the
    offline message."""
    def go():
        cb = getattr(win, "onReconnect", None)
        if cb is None:
            if status is not None:
                status.config(text="offline -- no robot link to reconnect")
            return
        cb()

    btn = tk.Button(parent, text="⟳ Reconnect", command=go, cursor="hand2")
    btn.pack(**pack)
    return btn
