"""
field_web.py -- phone / tablet control page for tools/field_gamemaster.py

The gamemaster window starts this on port 8080. Open  http://<gamemaster laptop IP>:8080
on any phone or tablet on the same Wi-Fi, type the PIN shown at the bottom of the
gamemaster window, and you get the same controls: pick the period, START / STOP /
RESET, enable / disable each alliance, tick who is "in this run", RELEASE ROBOTS,
plus the clock and each robot's status.

Nothing here touches the robots itself. A command is queued and the gamemaster
window runs it on its own thread, exactly as if its button had been pressed, so
the window and the page always agree.

Anyone on the Wi-Fi who knows the PIN can move robots, so the PIN is checked on
every request (X-Pin header). Start the gamemaster with --pin 1234 to pick it,
or --no-pin to turn the check off (only on a network you trust).
"""

import hmac
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

COMMANDS = {"select", "start", "stop", "reset", "toggle", "inrun", "release"}


class WebServer:
    """get_state() -> dict (latest snapshot); submit(cmd_dict) queues a command."""

    def __init__(self, port, pin, get_state, submit):
        self.port = port
        self.pin = pin
        self.get_state = get_state
        self.submit = submit
        outer = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):   # keep the console quiet
                pass

            def _send(self, code, body, ctype="application/json"):
                data = body if isinstance(body, bytes) else body.encode("utf-8")
                self.send_response(code)
                self.send_header("Content-Type", ctype + "; charset=utf-8")
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                try:
                    self.wfile.write(data)
                except OSError:
                    pass

            def _authorized(self):
                if not outer.pin:
                    return True
                given = self.headers.get("X-Pin", "")
                return hmac.compare_digest(given.encode(), outer.pin.encode())

            def do_GET(self):
                path = self.path.split("?")[0].rstrip("/") or "/"
                if path in ("/", "/index.html"):
                    self._send(200, PAGE, "text/html")
                elif path in ("/red", "/blue"):
                    # View-only team page: no PIN, it can't change anything.
                    self._send(200, TEAM_PAGE.replace("__ALLIANCE__", path[1:]), "text/html")
                elif path in ("/api/team/red", "/api/team/blue"):
                    a = path.rsplit("/", 1)[1]
                    s = outer.get_state()
                    if not s:
                        self._send(503, json.dumps({"error": "starting"}))
                    else:
                        self._send(200, json.dumps({
                            "alliance": a,
                            "title": s.get("title", ""),
                            "status": s["status"],
                            "clockText": s["clockText"],
                            "clockColor": s["clockColor"],
                            "running": s["running"],
                            "released": s["released"],
                            "enabled": s["enabled"][a],
                            "inRun": s["inRun"][a],
                            "robots": s["robots"].get(a, []),
                        }))
                elif path == "/api/state":
                    if not self._authorized():
                        self._send(401, json.dumps({"error": "pin"}))
                    else:
                        state = dict(outer.get_state())
                        state["pinRequired"] = bool(outer.pin)
                        self._send(200, json.dumps(state))
                else:
                    self._send(404, json.dumps({"error": "not found"}))

            def do_POST(self):
                if self.path.split("?")[0] != "/api/cmd":
                    self._send(404, json.dumps({"error": "not found"}))
                    return
                if not self._authorized():
                    self._send(401, json.dumps({"error": "pin"}))
                    return
                try:
                    length = min(int(self.headers.get("Content-Length", "0")), 2048)
                    cmd = json.loads(self.rfile.read(length) or b"{}")
                except (ValueError, OSError):
                    self._send(400, json.dumps({"error": "bad request"}))
                    return
                if not isinstance(cmd, dict) or cmd.get("cmd") not in COMMANDS:
                    self._send(400, json.dumps({"error": "unknown command"}))
                    return
                outer.submit(cmd)
                self._send(200, json.dumps({"ok": True}))

        self.httpd = ThreadingHTTPServer(("", port), Handler)
        self.httpd.daemon_threads = True
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()


TEAM_PAGE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<meta name="theme-color" content="#14141a">
<title>Field Control</title>
<style>
  :root { --bg:#14141a; --card:#161B22; --green:#2ECC71; --red:#FF3131; --blue:#3B8BFF;
          --gray:#8a8a99; --dim:#3a3a46; --white:#e6e6e6; --orange:#FF8C00; }
  * { box-sizing:border-box; }
  html,body { margin:0; min-height:100%; background:var(--bg); color:var(--white);
              font-family:system-ui,-apple-system,"Segoe UI",Roboto,sans-serif; }
  body { display:flex; flex-direction:column; min-height:100vh; padding:14px 16px calc(14px + env(safe-area-inset-bottom)); }
  h1 { margin:0; text-align:center; font-size:clamp(22px,6vw,40px); letter-spacing:1px; }
  body.red h1 { color:var(--red); } body.blue h1 { color:var(--blue); }
  .status { text-align:center; color:var(--gray); font-weight:700; font-size:clamp(15px,3.6vw,22px); margin:8px 0 0; min-height:1.4em; }
  .clock { text-align:center; font-family:ui-monospace,Consolas,monospace; font-weight:800;
           font-size:clamp(90px,30vw,260px); line-height:1.05; margin:0; }
  .banner { text-align:center; font-weight:800; font-size:clamp(26px,8vw,64px); padding:clamp(14px,3vw,28px) 8px;
            border-radius:16px; background:var(--dim); }
  .banner.on { background:var(--green); color:#0b0b0f; }
  .banner.off { background:var(--red); color:#0b0b0f; }
  .banner.wait { background:var(--orange); color:#0b0b0f; }
  .robots { background:var(--card); border-radius:14px; padding:12px 14px; margin-top:14px; font-family:ui-monospace,Consolas,monospace;
            font-size:clamp(13px,3.4vw,18px); white-space:pre-wrap; word-break:break-word; }
  .foot { margin-top:auto; padding-top:14px; text-align:center; color:var(--gray); font-size:13px; }
  .conn { display:inline-block; width:10px; height:10px; border-radius:50%; background:var(--red); margin-right:6px; }
  .conn.ok { background:var(--green); }
</style>
</head>
<body class="__ALLIANCE__">
<h1>__ALLIANCE__ ALLIANCE</h1>
<p class="status" id="status">connecting...</p>
<p class="clock" id="clock">-:--</p>
<div class="banner" id="banner">...</div>
<div class="robots" id="robots"></div>
<div class="foot"><span class="conn" id="conn"></span><span id="connText">connecting</span></div>
<script>
const $ = id => document.getElementById(id);
const A = "__ALLIANCE__";
document.querySelector("h1").textContent = A.toUpperCase() + " ALLIANCE";
let lastOk = 0;
function setConn(ok, t) { $("conn").classList.toggle("ok", ok); $("connText").textContent = t; }
async function poll() {
  try {
    const r = await fetch("/api/team/" + A, {cache:"no-store"});
    const s = await r.json(); lastOk = Date.now();
    setConn(true, "live");
    $("status").textContent = s.status;
    $("clock").textContent = s.clockText; $("clock").style.color = s.clockColor;
    const b = $("banner");
    b.className = "banner " + (s.released ? "wait" : (s.enabled ? "on" : "off"));
    b.textContent = s.released ? "PRACTICE MODE" : (s.enabled ? "ROBOTS ENABLED" : "ROBOTS DISABLED");
    $("robots").textContent = s.robots.join("\n") || "no robots connected";
  } catch (e) { if (Date.now() - lastOk > 2000) setConn(false, "no connection to gamemaster"); }
}
poll(); setInterval(poll, 300);
try { if (navigator.wakeLock) navigator.wakeLock.request("screen"); } catch (e) {}
</script>
</body>
</html>
"""

PAGE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<meta name="theme-color" content="#14141a">
<title>Field Control</title>
<style>
  :root { --bg:#14141a; --card:#161B22; --sel:#22303c; --cyan:#00F0FF; --orange:#FF8C00;
          --green:#2ECC71; --red:#FF3131; --blue:#3B8BFF; --gray:#8a8a99; --dim:#3a3a46; --white:#e6e6e6; }
  * { box-sizing:border-box; -webkit-tap-highlight-color:transparent; }
  html,body { margin:0; background:var(--bg); color:var(--white);
              font-family:system-ui,-apple-system,"Segoe UI",Roboto,sans-serif; }
  body { padding:12px 16px calc(16px + env(safe-area-inset-bottom)); max-width:980px; margin:0 auto; }
  button { font:inherit; color:inherit; border:0; border-radius:12px; cursor:pointer; touch-action:manipulation; }
  .periods { display:grid; grid-template-columns:repeat(auto-fit,minmax(140px,1fr)); gap:8px; }
  .periods button { background:var(--card); color:var(--gray); padding:12px 8px; font-weight:700; font-size:14px; line-height:1.3; }
  .periods button.sel { background:var(--sel); color:var(--cyan); }
  .status { text-align:center; color:var(--gray); font-weight:700; font-size:clamp(15px,3.6vw,20px); margin:14px 0 0; min-height:1.4em; }
  .clock { text-align:center; font-family:ui-monospace,Consolas,monospace; font-weight:800;
           font-size:clamp(84px,26vw,200px); line-height:1.05; margin:0; }
  .ctl { display:grid; grid-template-columns:2fr 1fr 1fr; gap:10px; margin:6px 0 14px; }
  .ctl button { padding:18px 8px; font-weight:800; font-size:clamp(16px,4.4vw,22px); }
  .start { background:var(--green); color:#0b0b0f; } .stop { background:var(--red); color:#0b0b0f; }
  .reset { background:var(--dim); }
  .alliances { display:grid; grid-template-columns:repeat(auto-fit,minmax(280px,1fr)); gap:12px; }
  .card { background:var(--card); border:3px solid; border-radius:14px; padding:12px; }
  .card.red { border-color:var(--red); } .card.blue { border-color:var(--blue); }
  .head { display:flex; align-items:center; justify-content:space-between; margin-bottom:8px; }
  .head h2 { margin:0; font-size:18px; } .red h2 { color:var(--red); } .blue h2 { color:var(--blue); }
  .head label { display:flex; align-items:center; gap:8px; font-size:14px; color:var(--white); padding:6px 0 6px 10px; }
  .head input { width:22px; height:22px; accent-color:var(--cyan); }
  .toggle { width:100%; padding:18px 8px; font-weight:800; font-size:clamp(16px,4.2vw,20px); background:var(--dim); }
  .toggle.on { background:var(--green); color:#0b0b0f; }
  .robots { font-family:ui-monospace,Consolas,monospace; font-size:13px; color:var(--white); margin:10px 2px 2px;
            white-space:pre-wrap; word-break:break-word; min-height:3em; }
  .foot { display:flex; justify-content:space-between; align-items:center; gap:10px; margin-top:14px; color:var(--gray); font-size:13px; }
  .release { background:var(--card); color:var(--gray); padding:10px 14px; font-weight:700; }
  .release.on { background:var(--orange); color:#0b0b0f; }
  .conn { display:inline-block; width:10px; height:10px; border-radius:50%; background:var(--red); margin-right:6px; }
  .conn.ok { background:var(--green); }
  #pin { position:fixed; inset:0; background:rgba(10,10,15,.96); display:none; align-items:center; justify-content:center; z-index:10; padding:20px; }
  #pin.show { display:flex; }
  #pin form { background:var(--card); padding:24px; border-radius:16px; width:100%; max-width:340px; text-align:center; }
  #pin input { width:100%; font-size:28px; text-align:center; letter-spacing:6px; padding:12px; border-radius:10px;
               border:2px solid var(--dim); background:var(--bg); color:var(--white); margin:14px 0; }
  #pin button { width:100%; padding:14px; background:var(--cyan); color:#0b0b0f; font-weight:800; font-size:18px; }
  #pin p { margin:0; color:var(--gray); } #pin .err { color:var(--red); min-height:1.3em; margin-top:8px; }
</style>
</head>
<body>
<div class="periods" id="periods"></div>
<p class="status" id="status">connecting...</p>
<p class="clock" id="clock">-:--</p>
<div class="ctl">
  <button class="start" id="start">START</button>
  <button class="stop" id="stop">STOP</button>
  <button class="reset" id="reset">RESET</button>
</div>
<div class="alliances">
  <div class="card red" id="card-red">
    <div class="head"><h2>RED ALLIANCE</h2><label><input type="checkbox" id="in-red">in this run</label></div>
    <button class="toggle" id="tog-red">DISABLED</button>
    <div class="robots" id="rob-red"></div>
  </div>
  <div class="card blue" id="card-blue">
    <div class="head"><h2>BLUE ALLIANCE</h2><label><input type="checkbox" id="in-blue">in this run</label></div>
    <button class="toggle" id="tog-blue">DISABLED</button>
    <div class="robots" id="rob-blue"></div>
  </div>
</div>
<div class="foot">
  <span><span class="conn" id="conn"></span><span id="connText">connecting</span></span>
  <button class="release" id="release">RELEASE ROBOTS</button>
</div>

<div id="pin"><form id="pinForm">
  <p><b>Field Control</b></p><p>Enter the PIN shown on the gamemaster laptop</p>
  <input id="pinInput" type="password" inputmode="numeric" autocomplete="off" autofocus>
  <button type="submit">Unlock</button><div class="err" id="pinErr"></div>
</form></div>

<script>
const $ = id => document.getElementById(id);
let pin = ""; try { pin = localStorage.getItem("fcPin") || ""; } catch (e) {}
let lastOk = 0, periodsBuilt = false;

function headers() { return {"Content-Type":"application/json", "X-Pin":pin}; }

async function send(cmd) {
  try {
    const r = await fetch("/api/cmd", {method:"POST", headers:headers(), body:JSON.stringify(cmd)});
    if (r.status === 401) showPin("Wrong PIN");
    else poll();
  } catch (e) {}
}

function showPin(msg) { $("pin").classList.add("show"); $("pinErr").textContent = msg || ""; $("pinInput").focus(); }
$("pinForm").addEventListener("submit", e => {
  e.preventDefault(); pin = $("pinInput").value.trim();
  try { localStorage.setItem("fcPin", pin); } catch (e) {}
  $("pinInput").value = ""; $("pin").classList.remove("show"); poll();
});

function buildPeriods(periods) {
  const box = $("periods"); box.innerHTML = "";
  for (const p of periods) {
    const b = document.createElement("button"); b.dataset.key = p.key;
    b.innerHTML = p.title + "<br>" + Math.floor(p.secs/60) + ":" + String(p.secs%60).padStart(2,"0");
    b.addEventListener("click", () => send({cmd:"select", key:p.key}));
    box.appendChild(b);
  }
  periodsBuilt = true;
}

function render(s) {
  if (!periodsBuilt) buildPeriods(s.periods);
  for (const b of $("periods").children) b.classList.toggle("sel", b.dataset.key === s.selected);
  $("status").textContent = s.status;
  $("clock").textContent = s.clockText;
  $("clock").style.color = s.clockColor;
  for (const a of ["red","blue"]) {
    const on = s.enabled[a], t = $("tog-"+a);
    t.textContent = on ? "ENABLED - tap to disable" : "DISABLED - tap to enable";
    t.classList.toggle("on", on);
    const cb = $("in-"+a); if (document.activeElement !== cb) cb.checked = s.inRun[a];
    $("rob-"+a).textContent = s.robots[a].join("\n");
  }
  const rel = $("release"); rel.textContent = s.released ? "TAKE CONTROL" : "RELEASE ROBOTS";
  rel.classList.toggle("on", s.released);
}

async function poll() {
  try {
    const r = await fetch("/api/state", {headers:headers(), cache:"no-store"});
    if (r.status === 401) { showPin(pin ? "Wrong PIN" : ""); setConn(false, "locked"); return; }
    const s = await r.json(); lastOk = Date.now(); setConn(true, "connected"); render(s);
    if (!s.pinRequired) $("pin").classList.remove("show");
  } catch (e) { if (Date.now() - lastOk > 2000) setConn(false, "no connection to gamemaster"); }
}
function setConn(ok, text) { $("conn").classList.toggle("ok", ok); $("connText").textContent = text; }

$("start").addEventListener("click", () => send({cmd:"start"}));
$("stop").addEventListener("click", () => send({cmd:"stop"}));
$("reset").addEventListener("click", () => send({cmd:"reset"}));
$("release").addEventListener("click", () => send({cmd:"release"}));
for (const a of ["red","blue"]) {
  $("tog-"+a).addEventListener("click", () => send({cmd:"toggle", alliance:a}));
  $("in-"+a).addEventListener("change", e => send({cmd:"inrun", alliance:a, value:e.target.checked}));
}
poll(); setInterval(poll, 300);
</script>
</body>
</html>
"""
