from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Literal, Any
import os
import sys
import time


DEFAULT_KINESIS_DIR = Path(r"C:\Program Files\Thorlabs\Kinesis")
KINESIS_DIR_ENV_VAR = "THORLABS_KINESIS_DIR"

DEVICE_MANAGER_DLL = "Thorlabs.MotionControl.DeviceManagerCLI.dll"
GENERIC_MOTOR_DLL = "Thorlabs.MotionControl.GenericMotorCLI.dll"
BENCHTOP_BRUSHLESS_DLL = "Thorlabs.MotionControl.Benchtop.BrushlessMotorCLI.dll"
TCUBE_DCSERVO_DLL = "Thorlabs.MotionControl.TCube.DCServoCLI.dll"
KCUBE_DCSERVO_DLL = "ThorLabs.MotionControl.KCube.DCServoCLI.dll"

DCServoKind = Literal["kcube", "tcube"]

_DLL_DIRECTORY_HANDLES: list[Any] = []


class KinesisError(RuntimeError):
    """Base exception for Kinesis bootstrap / setup failures."""


class KinesisPlatformError(KinesisError):
    """Raised when Kinesis is used on an unsupported platform."""


class KinesisNotFoundError(KinesisError):
    """Raised when the Kinesis install directory or a required DLL is missing."""


class KinesisInitializationError(KinesisError):
    """Raised when a connected device fails to initialize properly."""


@dataclass(frozen=True)
class KinesisCommonAPI:
    """Shared .NET symbols used by multiple Kinesis-backed device families."""
    kinesis_dir: Path
    DeviceManagerCLI: Any
    MotorDirection: Any
    Decimal: Any


@dataclass(frozen=True)
class BrushlessAPI:
    """Shared .NET symbols for the benchtop brushless family (e.g. BBD102/BBD202)."""

    common: KinesisCommonAPI
    BenchtopBrushlessMotor: Any


@dataclass(frozen=True)
class DCServoAPI:
    """Shared .NET symbols for a DC-servo family (T-Cube or K-Cube)."""

    common: KinesisCommonAPI
    controller_kind: DCServoKind
    ControllerClass: Any


def get_kinesis_dir(explicit: str | Path | None = None) -> Path:
    """
    Resolve the Thorlabs Kinesis installation directory.

    Resolution order:
      1. explicit argument
      2. THORLABS_KINESIS_DIR environment variable
      3. default Windows install path
    """
    if sys.platform != "win32":
        raise KinesisPlatformError(
            "Thorlabs Kinesis .NET DLLs are supported only on Windows."
        )

    if explicit is not None:
        root = Path(explicit)
    else:
        root = Path(os.environ.get(KINESIS_DIR_ENV_VAR, DEFAULT_KINESIS_DIR))

    if not root.exists():
        raise KinesisNotFoundError(
            f"Kinesis directory not found: {root}. "
            f"Set {KINESIS_DIR_ENV_VAR} or install Kinesis in the default location."
        )

    return root.resolve()


def _prepare_dll_search_path(root: Path) -> None:
    """
    Add the Kinesis directory to the Windows DLL search path.
    """
    handle = os.add_dll_directory(str(root))
    _DLL_DIRECTORY_HANDLES.append(handle)


def _add_reference(root: Path, dll_name: str) -> None:
    """
    Add a .NET assembly reference by full path using pythonnet.
    """
    dll_path = root / dll_name
    if not dll_path.exists():
        raise KinesisNotFoundError(f"Required Kinesis DLL not found: {dll_path}")

    import clr  

    clr.AddReference(str(dll_path)) # type: ignore


@lru_cache(maxsize=1)
def load_common_api(kinesis_dir: str | Path | None = None) -> KinesisCommonAPI:
    """
    Load the Kinesis assemblies shared by multiple controller families.

    This function is cached so the bootstrap happens only once per process.
    """
    root = get_kinesis_dir(kinesis_dir)
    _prepare_dll_search_path(root)

    _add_reference(root, DEVICE_MANAGER_DLL)
    _add_reference(root, GENERIC_MOTOR_DLL)

    from Thorlabs.MotionControl.DeviceManagerCLI import DeviceManagerCLI  # type: ignore
    from Thorlabs.MotionControl.GenericMotorCLI import MotorDirection  # type: ignore
    from System import Decimal  # type: ignore

    return KinesisCommonAPI(
        kinesis_dir=root,
        DeviceManagerCLI=DeviceManagerCLI,
        MotorDirection=MotorDirection,
        Decimal=Decimal,
    )


@lru_cache(maxsize=1)
def load_brushless_api(kinesis_dir: str | Path | None = None) -> BrushlessAPI:
    """
    Load the benchtop brushless Kinesis API.

    Intended for BBD102/BBD202-style controllers.
    """
    common = load_common_api(kinesis_dir)
    _add_reference(common.kinesis_dir, BENCHTOP_BRUSHLESS_DLL)

    from Thorlabs.MotionControl.Benchtop.BrushlessMotorCLI import (  # type: ignore
        BenchtopBrushlessMotor,
    )

    return BrushlessAPI(
        common=common,
        BenchtopBrushlessMotor=BenchtopBrushlessMotor,
    )


@lru_cache(maxsize=1)
def load_kcube_dcservo_api(
    kinesis_dir: str | Path | None = None,
) -> DCServoAPI:
    """
    Load the KCube DC-servo Kinesis API.

    Intended for KDC101-style controllers.
    """
    common = load_common_api(kinesis_dir)
    _add_reference(common.kinesis_dir, KCUBE_DCSERVO_DLL)

    from Thorlabs.MotionControl.KCube.DCServoCLI import KCubeDCServo  # type: ignore

    return DCServoAPI(
        common=common,
        controller_kind="kcube",
        ControllerClass=KCubeDCServo,
    )


@lru_cache(maxsize=1)
def load_tcube_dcservo_api(
    kinesis_dir: str | Path | None = None,
) -> DCServoAPI:
    """
    Load the TCube DC-servo Kinesis API.

    Intended for TDC001-style controllers.
    """
    common = load_common_api(kinesis_dir)
    _add_reference(common.kinesis_dir, TCUBE_DCSERVO_DLL)

    from Thorlabs.MotionControl.TCube.DCServoCLI import TCubeDCServo  # type: ignore

    return DCServoAPI(
        common=common,
        controller_kind="tcube",
        ControllerClass=TCubeDCServo,
    )


def load_dcservo_api(
    controller_kind: DCServoKind = "kcube",
    kinesis_dir: str | Path | None = None,
) -> DCServoAPI:
    """
    Load either the KCube or TCube DC-servo API.

    Parameters
    ----------
    controller_kind:
        "kcube" for KDC101-style controllers or "tcube" for TDC001-style controllers.
    """
    if controller_kind == "kcube":
        return load_kcube_dcservo_api(kinesis_dir)
    if controller_kind == "tcube":
        return load_tcube_dcservo_api(kinesis_dir)
    raise ValueError("controller_kind must be either 'kcube' or 'tcube'")


# ----- Helpers -----
def build_device_list(common_api: KinesisCommonAPI) -> list[str]:
    """
    Build and return the current list of connected Kinesis devices.
    """
    common_api.DeviceManagerCLI.BuildDeviceList()
    return [str(sn) for sn in common_api.DeviceManagerCLI.GetDeviceList()]


def find_serial_numbers(
    common_api: KinesisCommonAPI,
    *,
    prefix: str | None = None,
) -> list[str]:
    """
    Return connected serial numbers, optionally filtered by prefix.
    """
    serials = build_device_list(common_api)
    if prefix is None:
        return serials
    return [sn for sn in serials if sn.startswith(prefix)]


def wait_for_settings_initialized(device: Any, timeout_ms: int = 10_000) -> None:
    """
    Wait until a Kinesis device reports its settings as initialized.
    """
    if not device.IsSettingsInitialized():
        device.WaitForSettingsInitialized(timeout_ms)

    if not device.IsSettingsInitialized():
        raise KinesisInitializationError(
            f"Device settings failed to initialize within {timeout_ms} ms."
        )


def start_polling_and_enable(
    device: Any,
    *,
    polling_period_ms: int = 250,
    settle_time_s: float = 0.25,
) -> None:
    """Start polling and enable a Kinesis device."""
    device.StartPolling(polling_period_ms)
    time.sleep(settle_time_s)

    device.EnableDevice()
    time.sleep(settle_time_s)


def connect_polling_and_enable(
    device: Any,
    serial_number: str,
    *,
    polling_period_ms: int = 250,
    settle_time_s: float = 0.25,
    settings_timeout_ms: int = 10_000,
) -> None:
    """
    Convenience helper: connect, wait for settings, start polling, enable.
    """
    device.Connect(serial_number)
    wait_for_settings_initialized(device, timeout_ms=settings_timeout_ms)
    start_polling_and_enable(
        device,
        polling_period_ms=polling_period_ms,
        settle_time_s=settle_time_s,
    )


def stop_polling_disconnect_device(device: Any) -> None:
    """
    Stop polling and disconnect a device.
    """
    device.StopPolling()
    device.Disconnect()


def to_decimal(common_api: KinesisCommonAPI, value: float | int) -> Any:
    """
    Convert a Python numeric value to System.Decimal using the cached common API.
    """
    return common_api.Decimal(value)


def from_decimal(common_api: KinesisCommonAPI, value: Any) -> float:
    """
    Convert a System.Decimal (or similar .NET numeric) to a Python float.
    """
    return common_api.Decimal.ToDouble(value)