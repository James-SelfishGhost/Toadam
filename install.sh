#!/bin/bash
# Installation script for 3D Cable-Driven Positioning System

echo "=========================================="
echo "3D Cable-Driven Positioning System Setup"
echo "=========================================="

# Check Python version
echo "🐍 Checking Python version..."
python_version=$(python3 -c 'import sys; print(".".join(map(str, sys.version_info[:2])))')
echo "Found Python $python_version"

# Check if Python 3.8+ is available
if ! python3 -c 'import sys; exit(not (sys.version_info >= (3, 8)))' 2>/dev/null; then
    echo "❌ Error: Python 3.8 or higher is required"
    exit 1
fi

# Create virtual environment if it doesn't exist
if [ ! -d "venv" ]; then
    echo "🔧 Creating virtual environment..."
    python3 -m venv venv
fi

# Activate virtual environment
echo "⚡ Activating virtual environment..."
source venv/bin/activate

# Upgrade pip
echo "📦 Upgrading pip..."
pip install --upgrade pip

# Install dependencies
echo "📥 Installing dependencies..."
pip install -r requirements.txt

# Install package in development mode
echo "🔧 Installing cable system in development mode..."
pip install -e .

# Create necessary directories
echo "📂 Creating necessary directories..."
mkdir -p logs
mkdir -p config

# Copy default config if it doesn't exist
if [ ! -f "config/default_config.yaml" ]; then
    echo "📋 Copying default configuration..."
    cp config/default_config.yaml config/user_config.yaml
fi

echo ""
echo "✅ Installation completed successfully!"
echo ""
echo "To run the system:"
echo "  1. Activate the virtual environment: source venv/bin/activate"
echo "  2. Connect your ATmega2560 controller via USB"
echo "  3. Run the system: python main.py"
echo ""
echo "Configuration file: config/default_config.yaml"
echo "Logs will be saved to: logs/cable_system.log"
echo ""
echo "For help, check README.md or goals.md" 