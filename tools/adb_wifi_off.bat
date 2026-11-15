@echo off
rem Totally close all adb WiFi (TCP/IP) connections.
rem Run from the repo root:  tools\adb_wifi_off.bat
rem (also works from FtcRobotController:  ..\tools\adb_wifi_off.bat)

set ADB=%LOCALAPPDATA%\Android\Sdk\platform-tools\adb.exe
if not exist "%ADB%" set ADB=adb

rem Drop every TCP/IP adb connection (USB devices are unaffected)
"%ADB%" disconnect

rem Kill the adb server entirely so nothing lingers or auto-reconnects.
rem It restarts automatically on the next adb command.
"%ADB%" kill-server

echo All adb WiFi connections closed, server reset.
"%ADB%" devices
