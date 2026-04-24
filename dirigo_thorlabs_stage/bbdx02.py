from dataclasses import dataclass
from functools import cached_property
import time

from dirigo import units
from dirigo.hw_interfaces.stage import MultiAxisStage, LinearStage, StageInfo

from ._kinesis import (
    BrushlessAPI,
    build_device_list,
    load_brushless_api,
    to_decimal,
    from_decimal
)


@dataclass
class ThorlabsLinearMotorInfo(StageInfo):
    serial_number: str


class BBD102Stage(MultiAxisStage):
    """
    Communicate with a Thorlabs BBD102 brushless motor controller, which usually
    controls an MLS203 high-speed 2-axis linear motor stage.
    """

    SERIAL_NUMBER_PREFIX = "73"

    def __init__(
        self,
        x_config: dict,
        y_config: dict,
        serial_number: str | None = None,
        kinesis_dir: str | None = None,
        **kwargs,
    ):
        self._api: BrushlessAPI = load_brushless_api(kinesis_dir=kinesis_dir)

        # Build device list first, even if the device is already known.
        available_serial_numbers = build_device_list(self._api.common)

        resolved_serial_number = self._resolve_serial_number(serial_number, 
                                                             available_serial_numbers)

        self._controller = self._create_and_connect_device(resolved_serial_number)

        self._x_axis = ThorlabsLinearMotor(self._controller, api=self._api, **x_config)
        self._y_axis = ThorlabsLinearMotor(self._controller, api=self._api, **y_config)

        if not self._x_axis.homed:
            self._x_axis.home(blocking=False)
        if not self._y_axis.homed:
            self._y_axis.home(blocking=False)

    def _resolve_serial_number(self, serial_number: str | None, available_serial_numbers: list[str]):
        if serial_number is not None:
            if serial_number in available_serial_numbers:
                return serial_number
            else:
                raise RuntimeError(
                    f"BBDX02 device with serial number {serial_number} was not found in device list"
                )
        
        for candidate_sn in available_serial_numbers:
            if not candidate_sn.startswith(self.SERIAL_NUMBER_PREFIX):
                continue
            
            if serial_number is None:
                serial_number = candidate_sn
            else:
                # if we find multiple, raise an error
                raise RuntimeError(
                    "Found multiple BBDX02 stages. Use serial number argument to disambiguate."
                )
        
        if serial_number is None:
            raise RuntimeError(
                "BBDX02 not found among available devices. Check that it is "
                "connected and not in use by another program."
            )
        
        return serial_number

    def _create_and_connect_device(self, serial_number: str):
        controller = self._api.BenchtopBrushlessMotor.CreateBenchtopBrushlessMotor(serial_number)
        controller.Connect(serial_number)
        return controller


    @property
    def x(self):
        return self._x_axis

    @property
    def y(self):
        return self._y_axis

    @property
    def z(self):
        raise NotImplementedError("Z axis does not exist on BBDX02 devices.")

    def close(self):
        self._x_axis.close()
        self._y_axis.close()
        self._controller.Disconnect()


class BBD202Stage(BBD102Stage):
    """
    Alias for BBD102Stage.

    Under Kinesis, BBD102 and BBD202 use the same benchtop brushless API.
    """
    pass


class ThorlabsLinearMotor(LinearStage):
    HOME_TIMEOUT = units.Time("10 s")
    MOVE_TIMEOUT = units.Time("10 s")
    POLLING_PERIOD = units.Time("50 ms")

    def __init__(
        self,
        stage_controller,
        api: BrushlessAPI,
        position_limits: dict | None = None,
        **kwargs,
    ):
        super().__init__(**kwargs)

        self._api = api

        if position_limits is None:
            self._position_limits = None # position limit property will read limits from API
        else:
            self._validate_limits_dict(position_limits)
            self._position_limits = units.PositionRange(**position_limits)

        channel_number = 1 if self.axis == "x" else 2
        self._channel = stage_controller.GetChannel(channel_number)

        if not self._channel.IsSettingsInitialized():
            self._channel.WaitForSettingsInitialized(10000)
            if not self._channel.IsSettingsInitialized():
                raise RuntimeError("Failed to initialize stage axis")

        self._channel.StartPolling(int(1000 * self.POLLING_PERIOD))
        time.sleep(2 * self.POLLING_PERIOD)

        self._channel.EnableDevice()
        time.sleep(2 * self.POLLING_PERIOD)

        self._channel.LoadMotorConfiguration(self._channel.DeviceID)

        time.sleep(0.1) # wait 100 ms for axis to stabilize before taking initial position
        self._prev_position = units.Position(0) # this may not work if at exactly 0.0 at startup (see note on BUG in position getter)

        # test setting velo profile mode to trapezoidal
        velo_profile_params = self._channel.GetVelocityProfileParams()
        velo_profile_params.ProfileMode = velo_profile_params.ProfileMode.Trapezoidal
        self._channel.SetVelocityProfileParams(velo_profile_params)

    @cached_property
    def device_info(self) -> ThorlabsLinearMotorInfo:
        return ThorlabsLinearMotorInfo(
            manufacturer="Thorlabs",
            model="MLS203",
            serial_number=self._channel.DeviceID,
        )

    @cached_property
    def position_limits(self) -> units.PositionRange:
        if self._position_limits is not None:
            return self._position_limits

        min_position = from_decimal(
            self._api.common, 
            self._channel.AdvancedMotorLimits.LengthMinimum
        ) / 1000
        max_position = from_decimal(
            self._api.common, 
            self._channel.AdvancedMotorLimits.LengthMaximum
        ) / 1000
        
        return units.PositionRange(min_position, max_position)

    @property
    def position(self) -> units.Position:
        position = units.Position(
            from_decimal(
                self._api.common,
                self._channel.Position
            ) / 1000
        )

        # Kinesis sometimes reports exactly 0 immediately after MoveTo.
        if position == 0:
            return self._prev_position

        self._prev_position = position
        return position

    def move_to(self, position: units.Position, blocking: bool = False):
        if not isinstance(position, units.Position):
            raise ValueError(
                f"`move_to` requires a units.Position object, "
                f"(got type: {type(position)})"
            )

        if not self.position_limits.within_range(position):
            raise ValueError(
                f"Requested move ({position}) beyond limits, "
                f"min: {self.position_limits.min}, max: {self.position_limits.max}"
            )

        timeout_ms = int(1000 * self.MOVE_TIMEOUT) if blocking else 0
        self._channel.MoveTo(
            to_decimal(self._api.common, 1000 * float(position)),
            timeout_ms,
        )

    @property
    def moving(self) -> bool:
        return bool(self._channel.IsDeviceBusy)

    def move_velocity(self, velocity: units.Velocity):
        if not isinstance(velocity, units.Velocity):
            raise ValueError("velocity must be given in units.Velocity")

        if velocity > 0:
            motor_direction = self._api.common.MotorDirection.Forward
        else:
            velocity = velocity * -1
            motor_direction = self._api.common.MotorDirection.Backward

        unit_converter = self._channel.UnitConverter
        unit_type = unit_converter.UnitType.Velocity
        velocity_device_units = unit_converter.RealToDeviceUnit(
            to_decimal(self._api.common, 1000 * float(velocity)),
            unit_type,
        )

        self._channel.MoveContinuousAtVelocity(motor_direction, velocity_device_units)

    def stop(self):
        self._channel.StopImmediate()

    def home(self, blocking: bool = False):
        timeout_ms = int(1000 * self.HOME_TIMEOUT) if blocking else 0
        self._channel.Home(timeout_ms)

    @property
    def homed(self) -> bool:
        return self._channel.Status.IsHomed

    @property
    def max_velocity(self) -> units.Velocity:
        vel_params = self._channel.GetVelocityParams()
        value = self._api.common.Decimal.ToDouble(vel_params.MaxVelocity)
        return units.Velocity(value / 1000)

    @max_velocity.setter
    def max_velocity(self, new_velocity: units.Velocity):
        if not isinstance(new_velocity, units.Velocity):
            raise ValueError(
                f"`max_velocity` setter requires a units.Velocity object, "
                f"(got type: {type(new_velocity)})"
            )

        vel_params = self._channel.GetVelocityParams()
        vel_params.MaxVelocity = to_decimal(
            self._api.common, 1000 * float(new_velocity)
        )
        self._channel.SetVelocityParams(vel_params)

    @property
    def acceleration(self) -> units.Acceleration:
        vel_params = self._channel.GetVelocityParams()
        value = self._api.common.Decimal.ToDouble(vel_params.Acceleration)
        return units.Acceleration(value / 1000)

    @acceleration.setter
    def acceleration(self, new_acceleration: units.Acceleration):
        if not isinstance(new_acceleration, units.Acceleration):
            raise ValueError(
                f"`acceleration` setter requires a units.Acceleration object, "
                f"(got type: {type(new_acceleration)})"
            )

        vel_params = self._channel.GetVelocityParams()
        vel_params.Acceleration = to_decimal(
            self._api.common, 1000 * float(new_acceleration)
        )
        self._channel.SetVelocityParams(vel_params)

    def close(self):
        self._channel.StopPolling()
        self._channel.Disconnect()