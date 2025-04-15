def task_start():
    """Run the strawberry server"""
    return {
        'actions': ['python -m impostor.run_strawberry'],
        'verbosity': 2,
    }

def task_setup_vscode():
    """Set up VS Code configuration"""
    return {
        'actions': ['python scripts/setup_vscode.py'],
        'verbosity': 2,
    }

def task_register_jupyter_kernel():
    """Register the Jupyter kernel"""
    return {
        'actions': ['ipython kernel install --user --name=impostor-pdm --display-name="Impostor PDM"'],
        'verbosity': 2,
    }