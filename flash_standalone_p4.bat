@echo off
echo ===========================================================================
echo   FLASHING STANDALONE 4-STAGE CV FIRMWARE TO ESP32-P4 (COM17)
echo ===========================================================================
echo [1/3] Releasing COM17 from any running Python backend vision service...
powershell -NoProfile -Command "Get-CimInstance Win32_Process | Where-Object { $_.Name -eq 'python.exe' -and $_.CommandLine -match 'backend_vision_service' } | ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }"
timeout /t 1 /nobreak >nul

echo [2/3] Flashing ESP32-P4 on COM17 (Do NOT unplug until 100%% complete)...
cd /d "C:\Users\prana\Snyptr-Rail\temp_cam_inspect\build"
"C:\Espressif\python_env\idf5.5_py3.11_env\Scripts\python.exe" "C:\Espressif\frameworks\esp-idf-v5.5.5\components\esptool_py\esptool\esptool.py" --port COM17 --chip esp32p4 -b 230400 --before default_reset --after hard_reset write_flash --flash_mode dio --flash_freq 80m --flash_size 2MB 0x2000 bootloader/bootloader.bin 0x10000 ov5647_stream.bin 0x8000 partition_table/partition-table.bin

echo [3/3] Skipping laptop backend (detection now runs on P4).
echo.
echo   Laptop: connect Wi-Fi to ESP32_Camera / password123
echo   Then open dashboard.html and watch WIFI ONLINE + P4 UART stats.
echo ===========================================================================
pause
