import argparse
import os
from isaaclab.app import AppLauncher

# ---------------- CLI ----------------
parser = argparse.ArgumentParser(description="Convert URDF to USD (Isaac Lab) with fixed layer paths.")
parser.add_argument("--urdf", required=True, type=str, help="Path to the robot URDF.")
parser.add_argument("--out_usd", required=True, type=str, help="Output USD file path.")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

# ---------------- Launch Isaac Sim via Isaac Lab ----------------
app = AppLauncher(args).app

# Imports AFTER app launch (pxr is available only after this)
from pxr import Usd, UsdGeom, Gf
from pxr import Sdf, Usd
from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg


def _fix_sublayer_paths(root_usd_path: str):
    """Fix broken sublayer paths like 'C:/configuration/xxx.usd' -> './configuration/xxx.usd'."""
    root_layer = Sdf.Layer.FindOrOpen(root_usd_path)
    if root_layer is None:
        raise RuntimeError(f"Failed to open root USD layer: {root_usd_path}")

    changed = False
    new_paths = []
    for p in root_layer.subLayerPaths:
        # Handle the broken path your logs show
        if p.replace("\\", "/").startswith("C:/configuration/"):
            fname = os.path.basename(p)
            new_paths.append(f"./configuration/{fname}")
            changed = True
        # (optional) handle other variants just in case
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
        return False  # already good

    # Prefer /World if it exists, otherwise choose the first top-level prim.
    world = stage.GetPrimAtPath("/World")
    if world and world.IsValid():
        stage.SetDefaultPrim(world)
        stage.GetRootLayer().Save()
        return True

    tops = stage.GetPseudoRoot().GetChildren()
    if tops:
        stage.SetDefaultPrim(tops[0])
        stage.GetRootLayer().Save()
        return True

    # Nothing to set
    return False


def main():
    # IMPORTANT: use absolute paths everywhere
    urdf_path = os.path.abspath(os.path.normpath(args.urdf))
    out_usd = os.path.abspath(os.path.normpath(args.out_usd))

    if not os.path.isfile(urdf_path):
        raise FileNotFoundError(f"URDF not found: {urdf_path}")

    usd_dir = os.path.dirname(out_usd)
    usd_file_name = os.path.basename(out_usd)
    os.makedirs(usd_dir, exist_ok=True)

    cfg = UrdfConverterCfg(
        asset_path=urdf_path,
        usd_dir=usd_dir,              # absolute
        usd_file_name=usd_file_name,
        fix_base=False,               # humanoid should be floating
    )

    # Joint drive defaults (safe baseline)
    cfg.joint_drive.gains.stiffness = 200.0
    cfg.joint_drive.gains.damping = 20.0

    converter = UrdfConverter(cfg)

    # converter.usd_path is the "real" root USD it created
    root_usd_path = os.path.abspath(os.path.normpath(converter.usd_path))
    print(f"[OK] Generated USD at: {root_usd_path}")

    # Patch broken sublayer paths in the root USD
    changed = _fix_sublayer_paths(root_usd_path)
    if changed:
        print("[OK] Fixed broken subLayerPaths to use ./configuration/...")

    # Ensure defaultPrim exists to satisfy <defaultPrim> references
    if _ensure_default_prim(root_usd_path):
        print("[OK] Set defaultPrim on root USD")

    # ----------------- ADD THIS BLOCK -----------------
    # Bake a fixed rotation into a new USD so the robot is upright in Isaac (Z-up)
    from pxr import Usd, UsdGeom, Gf

    stage = Usd.Stage.Open(root_usd_path)
    if stage is None:
        raise RuntimeError(f"Failed to open USD stage: {root_usd_path}")

    # Find the prim to rotate:
    # - If defaultPrim is /World, rotate /World/Robot (or first child) instead.
    prim_to_rotate = stage.GetDefaultPrim()
    if not prim_to_rotate or not prim_to_rotate.IsValid():
        raise RuntimeError("No defaultPrim found; cannot apply rotation.")

    if prim_to_rotate.GetPath().pathString == "/World":
        # Prefer /World/Robot if it exists, else first child under /World
        robot = stage.GetPrimAtPath("/World/Robot")
        if robot and robot.IsValid():
            prim_to_rotate = robot
        else:
            kids = prim_to_rotate.GetChildren()
            if not kids:
                raise RuntimeError("Default prim is /World but has no children to rotate.")
            prim_to_rotate = kids[0]

    xform = UsdGeom.Xformable(prim_to_rotate)

    # Make it deterministic
    xform.ClearXformOpOrder()

    # Typical Y-up -> Z-up correction: rotate -90 degrees about X
    xform.AddRotateXYZOp().Set(Gf.Vec3f(-90.0, 0.0, 0.0))

    fixed_usd_path = os.path.splitext(root_usd_path)[0] + "_fixed.usd"
    stage.GetRootLayer().Export(fixed_usd_path)
    print(f"[OK] Wrote rotated USD: {fixed_usd_path}")
    # ----------------- END BLOCK -----------------

    print("[DONE] URDF -> USD pipeline complete.")


if __name__ == "__main__":
    main()
    app.close()
