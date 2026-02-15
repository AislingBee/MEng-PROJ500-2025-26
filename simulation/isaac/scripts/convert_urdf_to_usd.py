import argparse
import os
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Convert URDF to USD (Isaac Lab).")
parser.add_argument("--urdf", required=True, type=str, help="Path to the robot URDF.")
parser.add_argument("--out_usd", required=True, type=str, help="Output USD file path.")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app = AppLauncher(args).app

# Imports AFTER app launch
from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg


def main():
    cfg = UrdfConverterCfg(
        asset_path=os.path.normpath(args.urdf),
        usd_dir=usd_dir,
        usd_file_name=usd_file_name,
        fix_base=False,
    )

    cfg.joint_drive.gains.stiffness = 200.0
    cfg.joint_drive.gains.damping = 20.0
    cfg.joint_drive.drive_type = "position"

    converter = UrdfConverter(cfg)

    converter = UrdfConverter(cfg)
    print(f"[OK] Generated USD at: {converter.usd_path}")


if __name__ == "__main__":
    main()
    app.close()
