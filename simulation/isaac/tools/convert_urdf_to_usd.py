import argparse
import json
import os
import xml.etree.ElementTree as ET
from isaaclab.app import AppLauncher

# ---------------- CLI ----------------
parser = argparse.ArgumentParser(description="Convert URDF to USD (Isaac Lab) and generate a joint limits config.")
parser.add_argument("--urdf", required=True, type=str, help="Path to the robot URDF.")
parser.add_argument("--out_usd", required=True, type=str, help="Output USD file path.")
parser.add_argument(
    "--out_joint_limits",
    type=str,
    default=None,
    help="Optional output path for the joint limits config JSON. Defaults to <out_usd_dir>/joint_limits_config.json.",
)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

# ---------------- Launch Isaac Sim via Isaac Lab ----------------
app = AppLauncher(args).app

# Imports AFTER app launch (pxr is available only after this)
from pxr import Sdf, Usd, Gf
import omni.kit.commands
from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg


def _fix_sublayer_paths(root_usd_path: str):
    """Fix broken sublayer paths like 'C:/configuration/xxx.usd' -> './configuration/xxx.usd'."""
    root_layer = Sdf.Layer.FindOrOpen(root_usd_path)
    if root_layer is None:
        raise RuntimeError(f"Failed to open root USD layer: {root_usd_path}")

    changed = False
    new_paths = []
    for p in root_layer.subLayerPaths:
        if p.replace("\\", "/").startswith("C:/configuration/"):
            fname = os.path.basename(p)
            new_paths.append(f"./configuration/{fname}")
            changed = True
        elif p.replace("\\", "/").startswith("/configuration/"):
            fname = os.path.basename(p)
            new_paths.append(f"./configuration/{fname}")
            changed = True
        else:
            new_paths.append(p)

    if changed:
        root_layer.subLayerPaths = new_paths
        root_layer.Save()

    return changed


def _ensure_default_prim(root_usd_path: str):
    """Ensure the root layer has a defaultPrim so @file@<defaultPrim> resolves."""
    stage = Usd.Stage.Open(root_usd_path)
    if stage is None:
        raise RuntimeError(f"Failed to open USD stage: {root_usd_path}")

    if stage.GetDefaultPrim():
        return False

    robot = stage.GetPrimAtPath("/Robot")
    if robot and robot.IsValid():
        stage.SetDefaultPrim(robot)
        stage.GetRootLayer().Save()
        return True

    tops = stage.GetPseudoRoot().GetChildren()
    if tops:
        stage.SetDefaultPrim(tops[0])
        stage.GetRootLayer().Save()
        return True

    return False


def _parse_joint_limits_from_urdf(urdf_path: str):
    """
    Extract non-fixed joint names and limits from the URDF in file order.

    Returns:
        list[dict]: [{"name": ..., "lower": ..., "upper": ...}, ...]
    """
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    joint_limits = []
    skipped = []

    for joint in root.findall("joint"):
        joint_name = joint.get("name")
        joint_type = joint.get("type", "").strip()

        if joint_type == "fixed":
            continue

        limit_elem = joint.find("limit")
        if limit_elem is None:
            skipped.append(joint_name)
            continue

        lower_raw = limit_elem.get("lower")
        upper_raw = limit_elem.get("upper")

        if lower_raw is None or upper_raw is None:
            skipped.append(joint_name)
            continue

        joint_limits.append(
            {
                "name": joint_name,
                "lower": float(lower_raw),
                "upper": float(upper_raw),
            }
        )

    if not joint_limits:
        raise RuntimeError(f"No non-fixed joints with limits found in URDF: {urdf_path}")

    return joint_limits, skipped


def _write_joint_limits_config(urdf_path: str, out_usd_path: str, out_config_path: str):
    """Generate a JSON config containing joint names and lower/upper limits."""
    joint_limits, skipped = _parse_joint_limits_from_urdf(urdf_path)

    config = {
        "generated_from_urdf": os.path.abspath(os.path.normpath(urdf_path)),
        "generated_for_usd": os.path.abspath(os.path.normpath(out_usd_path)),
        "joint_names": [j["name"] for j in joint_limits],
        "joint_lower_limits": [j["lower"] for j in joint_limits],
        "joint_upper_limits": [j["upper"] for j in joint_limits],
        "joints": joint_limits,
        "skipped_joints_without_limits": skipped,
    }

    out_dir = os.path.dirname(out_config_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    with open(out_config_path, "w", encoding="utf-8") as f:
        json.dump(config, f, indent=4)

    return len(joint_limits), skipped


def _find_prim_by_name(stage: Usd.Stage, prim_name: str):
    """Find the first prim in the stage with the given name."""
    for prim in stage.Traverse():
        if prim.GetName() == prim_name:
            return prim
    return None


def _add_imu_sensor_to_imu_link(
    root_usd_path: str,
    imu_link_name: str = "imu_link",
    imu_sensor_name: str = "imu_sensor",
):
    """
    Add an Isaac IMU sensor under the CAD imu_link prim.

    The sensor is placed at zero local offset relative to imu_link, so it inherits
    the imu_link transform.
    """
    stage = Usd.Stage.Open(root_usd_path)
    if stage is None:
        raise RuntimeError(f"Failed to open USD stage: {root_usd_path}")

    imu_link_prim = _find_prim_by_name(stage, imu_link_name)
    if imu_link_prim is None or not imu_link_prim.IsValid():
        raise RuntimeError(
            f"Could not find imu link prim named '{imu_link_name}' in USD: {root_usd_path}"
        )

    imu_link_path = str(imu_link_prim.GetPath())
    imu_sensor_path = f"{imu_link_path}/{imu_sensor_name}"

    if stage.GetPrimAtPath(imu_sensor_path).IsValid():
        print(f"[INFO] IMU sensor already exists at: {imu_sensor_path}")
        return imu_sensor_path

    success, _sensor_prim = omni.kit.commands.execute(
        "IsaacSensorCreateImuSensor",
        path=imu_sensor_name,
        parent=imu_link_path,
        sensor_period=0.0,
        linear_acceleration_filter_size=1,
        angular_velocity_filter_size=1,
        orientation_filter_size=1,
        translation=Gf.Vec3d(0.0, 0.0, 0.0),
        orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
    )

    if not success:
        raise RuntimeError(f"Failed to create IMU sensor under: {imu_link_path}")

    stage.GetRootLayer().Save()
    print(f"[OK] Added IMU sensor at: {imu_sensor_path}")
    return imu_sensor_path

def main():
    urdf_path = os.path.abspath(os.path.normpath(args.urdf))
    out_usd = os.path.abspath(os.path.normpath(args.out_usd))

    if not os.path.isfile(urdf_path):
        raise FileNotFoundError(f"URDF not found: {urdf_path}")

    usd_dir = os.path.dirname(out_usd)
    usd_file_name = os.path.basename(out_usd)
    os.makedirs(usd_dir, exist_ok=True)

    if args.out_joint_limits:
        out_joint_limits = os.path.abspath(os.path.normpath(args.out_joint_limits))
    else:
        out_joint_limits = os.path.join(usd_dir, "joint_limits_config.json")

    cfg = UrdfConverterCfg(
        asset_path=urdf_path,
        usd_dir=usd_dir,
        usd_file_name=usd_file_name,
        fix_base=False,
    )

    cfg.joint_drive.gains.stiffness = 200.0
    cfg.joint_drive.gains.damping = 20.0

    converter = UrdfConverter(cfg)

    root_usd_path = os.path.abspath(os.path.normpath(converter.usd_path))
    print(f"[OK] Generated USD at: {root_usd_path}")

    changed = _fix_sublayer_paths(root_usd_path)
    if changed:
        print("[OK] Fixed broken subLayerPaths to use ./configuration/...")

    if _ensure_default_prim(root_usd_path):
        print("[OK] Set defaultPrim on root USD")

    imu_sensor_path = _add_imu_sensor_to_imu_link(
        root_usd_path=root_usd_path,
        imu_link_name="imu_link",
        imu_sensor_name="imu_sensor",
    )
    print(f"[OK] IMU sensor prim path: {imu_sensor_path}")

    joint_count, skipped = _write_joint_limits_config(
        urdf_path=urdf_path,
        out_usd_path=root_usd_path,
        out_config_path=out_joint_limits,
    )
    print(f"[OK] Generated joint limits config: {out_joint_limits}")
    print(f"[OK] Wrote limits for {joint_count} joints")
    if skipped:
        print(f"[INFO] Skipped joints with no lower/upper limits: {skipped}")

    print("[DONE] URDF -> USD pipeline complete.")


if __name__ == "__main__":
    try:
        main()
    finally:
        app.close()
