import json
from pathlib import Path

# Add Isaac Sim sdk to vscode extra paths
sdk_path = Path(__file__).parent.parent / ".pixi/envs/default/lib/python3.10/site-packages/isaacsim/"

extra_paths = []

# for each folder in the sdk path
for folder in sdk_path.iterdir():
    if not folder.is_dir():
        continue

    print(f"Found folder: {folder}")

    # for each folder in the folder
    for subfolder in folder.iterdir():
        if not subfolder.is_dir():
            continue
        print(f"Found package: {subfolder}")
        extra_paths.append(str(subfolder))


vscode_config =  Path(__file__).parent.parent / ".vscode/settings.json"
vscode_config.parent.mkdir(parents=True, exist_ok=True)

# Read config (create if it doesn't exist)
if not vscode_config.exists():
    vscode_config.write_text("{}")

config = json.loads(vscode_config.read_text())

# Merge the extra paths into the config
old_extra_paths = config.get("python.analysis.extraPaths", [])
config["python.analysis.extraPaths"] = list(set(old_extra_paths + extra_paths))

# Write the config back
vscode_config.write_text(json.dumps(config, indent=4))
