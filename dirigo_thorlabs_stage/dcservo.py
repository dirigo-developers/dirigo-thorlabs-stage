from dataclasses import dataclass
from functools import cached_property
from math import pi

from dirigo import units
from dirigo.hw_interfaces.stage import RotationStage, StageInfo

from ._kinesis import (
    DCServoKind,
    DCServoAPI,
    build_device_list,
    connect_polling_and_enable,
    stop_polling_disconnect_device,
    load_dcservo_api,
    to_decimal,
)


@dataclass
class PRM1Z8Info(StageInfo):
    serial_number: str
    controller_kind: str


class PRM1Z8(RotationStage):
    """
    Dirigo rotation-stage driver for a Thorlabs PRM1Z8 controlled through either a
    KDC101 (K-Cube) or TDC001 (T-Cube) DC-servo controller.

    Parameters
    ----------
    serial_number:
        Controller serial number. Can be omitted if exactly one Kinesis device is
        connected.
    controller_kind:
        "kcube" for KDC101-style controllers or "tcube" for TDC001-style controllers.
    limits:
        Logical wrapped limits exposed through the Dirigo API. These do not prevent
        the hardware from rotating continuously underneath.
    home_on_connect:
        If True, home the stage during initialization when it is not already homed.
    kinesis_dir:
        Optional override for the Kinesis installation directory.
    """

    POLLING_PERIOD_MS = 100
    CONNECT_SETTLE_S = 0.25

    HOME_TIMEOUT = units.Time("60 s")
    MOVE_TIMEOUT = units.Time("20 s")

    DEFAULT_LIMITS = {"min": "0 deg", "max": "360 deg"}

    def __init__(
        self,
        *,
        serial_number: str | None = None,
        controller_kind: DCServoKind = "kcube",
        device_settings_name: str = "PRM1-Z8",
        limits: dict | None = None,
        home_on_connect: bool = True,
        kinesis_dir: str | None = None,
        **kwargs,
    ):
        if limits is None:
            limits = dict(self.DEFAULT_LIMITS)

        super().__init__(axis="theta", limits=limits, **kwargs)

        self._controller_kind: DCServoKind = controller_kind
        self._device_settings_name = device_settings_name
        self._kinesis_dir = kinesis_dir

        self._api: DCServoAPI = load_dcservo_api(
            controller_kind=self._controller_kind,
            kinesis_dir=self._kinesis_dir,
        )

        self._serial_number = self._resolve_serial_number(serial_number)
        self._device = self._create_and_connect_device(self._serial_number)

        self._apply_motor_configuration()

        if home_on_connect and not self.homed:
            self.home(blocking=True)

        self._last_max_velocity = None

    # -------------------------------------------------------------------------
    # Internal helpers
    # -------------------------------------------------------------------------

    def _resolve_serial_number(self, serial_number: str | None) -> str:
        # Even if serial number known, must build device list first to initialize properly
        serials = build_device_list(self._api.common) 

        if serial_number is not None:
            return serial_number

        if len(serials) == 1:
            return serials[0]

        if not serials:
            raise RuntimeError("No Kinesis devices detected.")

        raise RuntimeError(
            "Multiple Kinesis devices detected. Please specify serial_number explicitly."
        )

    def _create_and_connect_device(self, serial_number: str):
        if self._controller_kind == "kcube":
            create_name = "CreateKCubeDCServo"
        else:
            create_name = "CreateTCubeDCServo"

        create_fn = getattr(self._api.ControllerClass, create_name, None)
        if create_fn is None:
            raise RuntimeError(
                f"Unable to create {self._controller_kind} DC-servo controller "
                f"for serial number {serial_number}."
            )
        device = create_fn(serial_number)

        connect_polling_and_enable(
            device,
            serial_number,
            polling_period_ms=self.POLLING_PERIOD_MS,
            settle_time_s=self.CONNECT_SETTLE_S,
        )
        return device

    def _apply_motor_configuration(self) -> None:
        """Load and apply motor settings for the PRM1Z8."""
        config = self._device.LoadMotorConfiguration(self._serial_number)

        config.DeviceSettingsName = self._device_settings_name
        config.UpdateCurrentConfiguration()

        self._device.SetSettings(self._device.MotorDeviceSettings, True, False) # bools are update device, persist

    def _decimal_to_float(self, value) -> float:
        return float(self._api.common.Decimal.ToDouble(value))

    def _raw_position_angle(self) -> units.Angle:
        return units.Angle(self._decimal_to_float(self._device.Position) * (pi/180.0))

    @staticmethod
    def _wrap_angle(angle: units.Angle) -> units.Angle:
        return units.Angle(float(angle) % (2*pi))

    def _move_absolute_angle(self, target_angle: units.Angle, blocking: bool) -> None:
        timeout_ms = int(1000 * self.MOVE_TIMEOUT) if blocking else 0
        target_dec = to_decimal(self._api.common, float(target_angle) * (180.0/pi))

        self._device.MoveTo(target_dec, timeout_ms)

    @property
    def serial_number(self) -> str:
        return self._serial_number

    @property
    def controller_kind(self) -> DCServoKind:
        return self._controller_kind

    # -------------------------------------------------------------------------
    # Required Dirigo Stage / RotationStage API
    # -------------------------------------------------------------------------

    @cached_property
    def device_info(self) -> PRM1Z8Info:
        return PRM1Z8Info(
            manufacturer="Thorlabs",
            model="PRM1Z8",
            serial_number=self._serial_number,
            controller_kind=self._controller_kind,
        )

    @property
    def position(self) -> units.Angle:
        return self._wrap_angle(self._raw_position_angle())

    def move_to(self, angle: units.Angle, blocking: bool = False):
        """
        Move to a logical wrapped angle in the interval [0 deg, 360 deg).
        """
        if not isinstance(angle, units.Angle):
            raise ValueError(
                f"`move_to` requires a units.Angle object, "
                f"(got type: {type(angle)})")
        
        wrapped_angle = self._wrap_angle(angle)
        
        if not self.position_limits.within_range(wrapped_angle):
            raise ValueError(
                f"Requested angle {angle} is outside logical limits "
                f"{self.position_limits.min} to {self.position_limits.max}."
            )

        self._move_absolute_angle(wrapped_angle, blocking=blocking)

    @property
    def moving(self) -> bool:
        return bool(self._device.IsDeviceBusy)

    def move_velocity(self, velocity: units.AngularVelocity):
        """
        Start continuous motion at the specified angular velocity.
        """
        if not isinstance(velocity, units.AngularVelocity):
            raise ValueError(
                f"`move_velocity` requires a units.AngularVelocity object, "
                f"(got type: {type(velocity)})")
        
        if velocity == 0:
            self.stop()
            return

        if velocity > 0:
            direction = self._api.common.MotorDirection.Forward
        else: 
            direction = self._api.common.MotorDirection.Backward

        if self._last_max_velocity is None:
            self._last_max_velocity = self.max_velocity

        self.max_velocity = units.AngularVelocity(abs(velocity))
        self._device.MoveContinuous(direction)

    def stop(self):
        self._device.StopImmediate()

        # Reset max velocity after move_velocity
        if self._last_max_velocity is not None:
            self.max_velocity = self._last_max_velocity
            self._last_max_velocity = None

    def home(self, blocking: bool = False):
        timeout_ms = int(1000 * self.HOME_TIMEOUT) if blocking else 0
        self._device.Home(timeout_ms)

    @property
    def homed(self) -> bool:
        status = getattr(self._device, "Status", None)
        if status is None:
            return False
        return bool(status.IsHomed)

    @property
    def max_velocity(self) -> units.AngularVelocity:
        vel_params = self._device.GetVelocityParams()
        value = self._decimal_to_float(vel_params.MaxVelocity) # in deg/s
        return units.AngularVelocity(value * (pi/180.0))

    @max_velocity.setter
    def max_velocity(self, value: units.AngularVelocity):
        if not isinstance(value, units.AngularVelocity):
            raise ValueError(
                f"`max_velocity` setter requires a units.AngularVelocity object, "
                f"(got type: {type(value)})")
        
        vel_params = self._device.GetVelocityParams()
        vel_params.MaxVelocity = to_decimal(self._api.common, float(value) * (180.0/pi))
        self._device.SetVelocityParams(vel_params)

    @property
    def acceleration(self) -> units.AngularAcceleration:
        vel_params = self._device.GetVelocityParams()
        value = self._decimal_to_float(vel_params.Acceleration) # in deg/s^2
        return units.AngularAcceleration(value * (pi/180.0))

    @acceleration.setter
    def acceleration(self, value: units.AngularAcceleration):
        if not isinstance(value, units.AngularAcceleration):
            raise ValueError(
                f"`acceleration` setter requires a units.AngularAcceleration object, "
                f"(got type: {type(value)})")
        
        vel_params = self._device.GetVelocityParams()
        vel_params.Acceleration = to_decimal(self._api.common, float(value) * (180.0/pi))
        self._device.SetVelocityParams(vel_params)

    # ----- Cleanup -----

    def close(self):
        stop_polling_disconnect_device(self._device)
