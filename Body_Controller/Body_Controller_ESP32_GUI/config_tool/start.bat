@echo off
title R2D2 Body Controller Config Tool
cd /d "%~dp0"

echo ============================================
echo  R2D2 Body Controller -- RC Config Tool
echo ============================================
echo.

:: Check if Python is available
python --version >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
    python3 --version >nul 2>&1
    if %ERRORLEVEL% NEQ 0 (
        echo ERROR: Python not found.
        echo Install Python from https://python.org then try again.
        pause
        exit /b 1
    )
    set PYTHON=python3
) else (
    set PYTHON=python
)

echo  Python found.  Starting HTTP server on http://localhost:8080/
echo.
echo  1. The server will start below.
echo  2. Open Chrome and navigate to:  http://localhost:8080/
echo  3. Connect your Body Controller via USB, then click Connect.
echo.
echo  Press Ctrl+C to stop the server when you are done.
echo ============================================
echo.

:: Try to open browser automatically (best-effort, silently ignore failures)
start "" "http://localhost:8080/" >nul 2>&1

%PYTHON% -m http.server 8080 --bind 127.0.0.1

echo.
echo  Server stopped.
