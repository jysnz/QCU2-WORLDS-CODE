@echo off
rem BLUE alliance laptop: plug each driver's controller in by USB, then run this.
rem If the gamemaster is not found automatically, give its IP:  start_blue.bat --host 192.168.1.20
cd /d "%~dp0..\.."
where py >nul 2>nul && (py -3 tools\field_relay.py --alliance blue %*) || (python tools\field_relay.py --alliance blue %*)
pause
