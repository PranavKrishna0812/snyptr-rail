@echo off
setlocal
set "ORIG_PATH=%PATH%"

echo ===========================================================================
echo       SNYPTR-RAIL: AUTOMATIC COMPILE ^& FLASH TO ESP32-P4
echo ===========================================================================

set "TARGET_PORT=%~1"
if "%TARGET_PORT%"=="" set "TARGET_PORT=COM17"
if "%TARGET_PORT:~0,1%"=="-" set "TARGET_PORT=COM17"

echo Target COM Port : %TARGET_PORT%
echo Project Directory: %~dp0temp_cam_inspect
echo.

:: 1. Release COM port from view_camera_live.py
echo [1/3] Releasing %TARGET_PORT% (stopping running camera viewers)...
powershell -NoProfile -Command "Get-CimInstance Win32_Process | Where-Object { $_.Name -eq 'python.exe' -and ($_.CommandLine -match 'view_camera_live' -or $_.CommandLine -match 'backend_vision_service') } | ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }"
ping -n 2 127.0.0.1 >nul

:: 2. Set ESP-IDF Toolchain Environment & Compile with Ninja
echo [2/3] Compiling firmware with ESP-IDF toolchain (Ninja)...
set "IDF_PATH=C:\Espressif\frameworks\esp-idf-v5.5.5"
set "PATH=C:\Espressif\python_env\idf5.5_py3.11_env\Scripts;C:\Espressif\tools\ninja\1.12.1;C:\Espressif\tools\cmake\3.30.2\bin;C:\Espressif\tools\riscv32-esp-elf\esp-14.2.0_20241119\riscv32-esp-elf\bin;%PATH%"

cd /d "%~dp0temp_cam_inspect\build"
"C:\Espressif\tools\ninja\1.12.1\ninja.exe" -C "%~dp0temp_cam_inspect\build"
if errorlevel 1 (
    echo.
    echo ===========================================================================
    echo   [ERROR] Compilation failed! Review the compiler output above.
    echo ===========================================================================
    pause
    exit /b 1
)

echo.
echo [COMPILATION SUCCESSFUL] Binary: %~dp0temp_cam_inspect\build\ov5647_stream.bin
echo.

:: 3. Flash to ESP32-P4
echo [3/3] Flashing ESP32-P4 on %TARGET_PORT% (Do NOT disconnect USB)...
"C:\Espressif\python_env\idf5.5_py3.11_env\Scripts\python.exe" "C:\Espressif\frameworks\esp-idf-v5.5.5\components\esptool_py\esptool\esptool.py" --port %TARGET_PORT% --chip esp32p4 -b 460800 --before default_reset --after hard_reset write_flash --flash_mode dio --flash_freq 80m --flash_size 2MB 0x2000 bootloader/bootloader.bin 0x10000 ov5647_stream.bin 0x8000 partition_table/partition-table.bin
if errorlevel 1 (
    echo.
    echo [WARNING] Flashing at 460800 baud failed, retrying at safe 230400 baud...
    "C:\Espressif\python_env\idf5.5_py3.11_env\Scripts\python.exe" "C:\Espressif\frameworks\esp-idf-v5.5.5\components\esptool_py\esptool\esptool.py" --port %TARGET_PORT% --chip esp32p4 -b 230400 --before default_reset --after hard_reset write_flash --flash_mode dio --flash_freq 80m --flash_size 2MB 0x2000 bootloader/bootloader.bin 0x10000 ov5647_stream.bin 0x8000 partition_table/partition-table.bin
    if errorlevel 1 (
        echo.
        echo ===========================================================================
        echo   [ERROR] Flashing failed! Ensure ESP32-P4 is connected to %TARGET_PORT%.
        echo ===========================================================================
        pause
        exit /b 1
    )
)

echo.
echo ===========================================================================
echo   SUCCESS: ESP32-P4 COMPILED AND FLASHED SUCCESSFULLY!
echo ===========================================================================
echo.
cd /d "%~dp0"

:: Restore system PATH so IDF python environment does not shadow user's Python packages
set "PATH=%ORIG_PATH%"
set "USER_PYTHON=%LOCALAPPDATA%\Programs\Python\Python313\python.exe"
if not exist "%USER_PYTHON%" set "USER_PYTHON=python"

echo Launching live camera feed viewer in 2 seconds (using %USER_PYTHON%)...
ping -n 3 127.0.0.1 >nul
start cmd /k "cd /d "%~dp0" && "%USER_PYTHON%" view_camera_live.py"
