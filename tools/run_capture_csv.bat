@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
set "SCRIPT_PATH=%SCRIPT_DIR%capture_csv.py"
set "PYTHON_EXE="
set "PYTHON_ARGS="

cd /d "%SCRIPT_DIR%"

if not exist "%SCRIPT_PATH%" (
    echo Script not found: "%SCRIPT_PATH%"
    echo.
    pause
    exit /b 1
)

if exist "%SCRIPT_DIR%python.exe" (
    set "PYTHON_EXE=%SCRIPT_DIR%python.exe"
    goto run
)

where py >nul 2>nul
if not errorlevel 1 (
    set "PYTHON_EXE=py"
    set "PYTHON_ARGS=-3"
    goto run
)

where python >nul 2>nul
if errorlevel 1 (
    echo Python was not found in PATH.
    echo Install Python first, or run this BAT from a terminal where `python` is available.
    echo.
    pause
    exit /b 1
)
set "PYTHON_EXE=python"

:run
if "%~1"=="" (
    echo Starting capture with default arguments: --port auto --baud 2000000
    call "%PYTHON_EXE%" %PYTHON_ARGS% "%SCRIPT_PATH%" --port auto --baud 2000000
) else (
    call "%PYTHON_EXE%" %PYTHON_ARGS% "%SCRIPT_PATH%" %*
)

set "EXIT_CODE=%ERRORLEVEL%"
if not "%EXIT_CODE%"=="0" (
    echo.
    echo Capture script exited with code %EXIT_CODE%.
    echo.
    echo Common fixes:
    echo 1. Check Python is installed on this PC.
    echo 2. Install pyserial: python -m pip install pyserial
    echo 3. List serial ports: run_capture_csv.bat --list-ports
    echo 4. Start capture with an explicit port:
    echo    run_capture_csv.bat --port COM7 --baud 2000000
    echo.
    pause
)

endlocal & exit /b %EXIT_CODE%
