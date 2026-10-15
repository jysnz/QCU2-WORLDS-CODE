@echo off
rem Gamemaster laptop: runs the clock, enables/disables the robots, serves the phone page.
rem Extra options pass through, e.g.  start_gamemaster.bat --pin 1234
cd /d "%~dp0..\.."
where py >nul 2>nul && (py -3 tools\field_gamemaster.py %*) || (python tools\field_gamemaster.py %*)
pause
