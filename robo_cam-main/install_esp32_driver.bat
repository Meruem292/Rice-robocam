@echo off
cd /d "%~dp0"
echo =========================================================
echo Installing CP2102 USB to UART Driver for ESP32...
echo =========================================================
echo Please click "Yes" on the Windows Administrator prompt...
powershell -NoProfile -Command "Start-Process pnputil.exe -ArgumentList '/add-driver \""%~dp0drivers\cp210x\silabser.inf\"" /install' -Verb RunAs"
echo =========================================================
echo Installation complete! 
echo Now open Arduino IDE, go to Tools -> Port, and select your COM port.
echo =========================================================
pause

