@echo off
rem Run once on the relay laptops (red, blue): the relay needs pyserial.
where py >nul 2>nul && (py -3 -m pip install pyserial) || (python -m pip install pyserial)
pause
