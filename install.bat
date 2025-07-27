@echo off
REM Installation script for 3D Cable-Driven Positioning System (Windows)

echo ==========================================
echo 3D Cable-Driven Positioning System Setup
echo ==========================================

REM Check if Python is available
echo 🐍 Checking Python installation...
python --version >nul 2>&1
if errorlevel 1 (
    echo ❌ Error: Python is not installed or not in PATH
    echo Please install Python 3.8+ from https://python.org
    pause
    exit /b 1
)

REM Check Python version
for /f "tokens=2" %%i in ('python --version') do set python_version=%%i
echo Found Python %python_version%

REM Create virtual environment if it doesn't exist
if not exist "venv" (
    echo 🔧 Creating virtual environment...
    python -m venv venv
)

REM Activate virtual environment
echo ⚡ Activating virtual environment...
call venv\Scripts\activate.bat

REM Upgrade pip
echo 📦 Upgrading pip...
python -m pip install --upgrade pip

REM Install dependencies
echo 📥 Installing dependencies...
pip install -r requirements.txt

REM Install package in development mode
echo 🔧 Installing cable system in development mode...
pip install -e .

REM Create necessary directories
echo 📂 Creating necessary directories...
if not exist "logs" mkdir logs
if not exist "config" mkdir config

REM Copy default config if it doesn't exist
if not exist "config\user_config.yaml" (
    echo 📋 Copying default configuration...
    copy "config\default_config.yaml" "config\user_config.yaml"
)

echo.
echo ✅ Installation completed successfully!
echo.
echo To run the system:
echo   1. Activate the virtual environment: venv\Scripts\activate.bat
echo   2. Connect your ATmega2560 controller via USB
echo   3. Run the system: python main.py
echo.
echo Configuration file: config\default_config.yaml
echo Logs will be saved to: logs\cable_system.log
echo.
echo For help, check README.md or goals.md
echo.
pause 