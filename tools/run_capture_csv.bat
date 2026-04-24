@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
set "SCRIPT_PATH=%SCRIPT_DIR%capture_csv.py"

where python >nul 2>nul
if errorlevel 1 (
    echo Python was not found in PATH.
    echo Install Python first, or run this BAT from a terminal where ^`python^` is available.
    exit /b 1
)

if not exist "%SCRIPT_PATH%" (
    echo Script not found: "%SCRIPT_PATH%"
    exit /b 1
)

if "%~1"=="" (
    echo Starting capture with default arguments: --port auto --baud 2000000
    python "%SCRIPT_PATH%" --port auto --baud 2000000
) else (
    python "%SCRIPT_PATH%" %*
)

set "EXIT_CODE=%ERRORLEVEL%"
if not "%EXIT_CODE%"=="0" (
    echo.
    echo Capture script exited with code %EXIT_CODE%.
)

endlocal & exit /b %EXIT_CODE%
