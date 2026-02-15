import argparse
import os
from omni.isaac.lab.app import AppLauncher

parser = argparse.ArgumentParser(description="Convert URDF to USD (Isaac Lab).")
parser.add_argument("--urdf", required=True, type=str, help="Path to the robot URDF.")
parser.add_argument("--out_usd", required=True, type=str, help="Output USD file path.")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app = AppLauncher(args).app

# Imports AFTER app launch
from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg


def main():
    out_usd = os.path.normpath(args.out_usd)
    usd_dir = os.path.dirname(out_usd)
    usd_file_name = os.path.basename(out_usd)

    os.makedirs(usd_dir, exist_ok=True)

    cfg = UrdfConverterCfg(
        asset_path=os.path.normpath(args.urdf),
        usd_dir=usd_dir,
        usd_file_name=usd_file_name,
        # Leave defaults for first pass.
        # You can tune later (e.g. instanceable, collision settings, etc.)
    )

    converter = UrdfConverter(cfg)
    print(f"[OK] Generated USD at: {converter.usd_path}")


if __name__ == "__main__":
    main()
    app.close()
