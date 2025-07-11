# Builds the Rust project with Python bindings and installs it using `maturin` and `uv`.
uv run maturin develop && uv pip install . --reinstall