@echo off
rem Run once on EACH laptop, as Administrator (right-click -> Run as administrator).
rem Opens the field-control ports in the Windows firewall for all network types.
net session >nul 2>&1
if errorlevel 1 (echo Please right-click this file and choose "Run as administrator". & pause & exit /b 1)
netsh advfirewall firewall delete rule name="Field Control" >nul
netsh advfirewall firewall add rule name="Field Control" dir=in action=allow protocol=TCP localport=5800,8080 profile=any
netsh advfirewall firewall add rule name="Field Control" dir=in action=allow protocol=UDP localport=5801 profile=any
echo Done: TCP 5800 + 8080 and UDP 5801 are open.
pause
