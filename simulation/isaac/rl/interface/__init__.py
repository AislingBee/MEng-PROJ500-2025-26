from importlib import import_module

__all__ = [
    "BaseHardwareInterface",
    "ControlPacket",
    "IsaacHardwareInterface",
    "ObservationPacket",
    "RobotCommandMessage",
    "RobotHardwareInterface",
    "RobotInterfaceConfig",
    "RobotStateSample",
    "Ros2RobotBridge",
    "make_ros2_hardware_interface",
]


def __getattr__(name: str):
    if name in {"BaseHardwareInterface", "ControlPacket", "ObservationPacket"}:
        module = import_module(".hardware_interface", __name__)
        return getattr(module, name)

    if name == "IsaacHardwareInterface":
        module = import_module(".isaac_hardware_interface", __name__)
        return getattr(module, name)

    if name in {
        "RobotCommandMessage",
        "RobotHardwareInterface",
        "RobotInterfaceConfig",
        "RobotStateSample",
    }:
        module = import_module(".robot_hardware_interface", __name__)
        return getattr(module, name)

    if name in {"Ros2RobotBridge", "make_ros2_hardware_interface"}:
        module = import_module(".ros2_robot_bridge", __name__)
        return getattr(module, name)

    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
