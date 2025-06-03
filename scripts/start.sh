#!/bin/bash

# Change to the project root directory (parent of scripts)
cd "$(dirname "$0")/.."

# Check if uv is available
if command -v uv >/dev/null 2>&1; then
    echo "Using uv to run the module..."
    uv run python -m impostor.run_strawberry
else
    echo "Warning: uv not found, falling back to regular python"
    echo "Consider installing uv for better dependency management: https://github.com/astral-sh/uv"
    python -m impostor.run_strawberry
fi