#!/bin/bash

# Exit on error
set -e

# Enable command printing
set -x

# Save output directory
LOG_DIR="lint_logs"
mkdir -p $LOG_DIR

# Log everything to a file
exec 1> >(tee -a "${LOG_DIR}/lint.log")
exec 2> >(tee -a "${LOG_DIR}/lint.log" >&2)

echo "Starting linting checks at $(date)"

echo "Current directory: $(pwd)"
echo "Directory contents:"
ls -la

echo "Python version:"
python3 --version

echo "Flake8 version:"
flake8 --version

echo "Running flake8..."
flake8 ros2_wave_pkg --show-source --statistics > "${LOG_DIR}/flake8.log" 2>&1 || {
    echo "Flake8 found issues:"
    cat "${LOG_DIR}/flake8.log"
    exit 1
}

echo "black version:"
black --version

echo "Running black..."
black --check --diff ros2_wave_pkg > "${LOG_DIR}/black.log" 2>&1 || {
    echo "Black found formatting issues:"
    cat "${LOG_DIR}/black.log"
    exit 1
}

echo "isort version:"
isort --version

echo "Running isort..."
isort --check-only --diff ros2_wave_pkg > "${LOG_DIR}/isort.log" 2>&1 || {
    echo "isort found import ordering issues:"
    cat "${LOG_DIR}/isort.log"
    exit 1
}

echo "All linting checks passed!"