import argparse
import os
from isaaclab.app import AppLauncher

# ---------------- CLI ----------------
parser = argparse.ArgumentParser(description="Convert URDF to USD (Isaac Lab).")
parser.add_argument("--urdf", required=True, type=str, help="Path to the robot URDF.")
parser.add_argument("--out_usd", required=True, type=str, help="Output USD file path.")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

# ---------------- Launch Isaac Sim via Isaac Lab ----------------
app = AppLauncher(args).app

# Imports AFTER app launch
from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg


def main():
    urdf_path = os.path.normpath(args.urdf)
    out_usd = os.path.normpath(args.out_usd)

    if not os.path.isfile(urdf_path):
        raise FileNotFoundError(f"URDF not found: {urdf_path}")

    usd_dir = os.path.dirname(out_usd)
    usd_file_name = os.path.basename(out_usd)
    os.makedirs(usd_dir, exist_ok=True)

    # Create config with required fields
    cfg = UrdfConverterCfg(
        asset_path=urdf_path,
        usd_dir=usd_dir,
        usd_file_name=usd_file_name,
        fix_base=False,  # humanoid should be floating
    )

    # Fill required joint drive fields (typed config, not dict)
    cfg.joint_drive.gains.stiffness = 200.0
    cfg.joint_drive.gains.damping = 20.0

    converter = UrdfConverter(cfg)
    print(f"[OK] Generated USD at: {converter.usd_path}")


if __name__ == "__main__":
    main()
    app.close()
