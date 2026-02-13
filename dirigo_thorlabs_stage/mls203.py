import time
from typing import cast, Literal
from enum import StrEnum

import clr
from pydantic import Field

from dirigo import units
from dirigo.hw_interfaces.stage import (
    LinearStageAxis, LinearStageAxisConfig, LinearStageAxisSettings,
    XYStage, XYStageConfig, XYStageSettings
)


# Import .NET resources--unfortunately autocomplete is not helpful with these
kinesis_location = "C:\\Program Files\\Thorlabs\\Kinesis\\"
clr.AddReference(kinesis_location + "Thorlabs.MotionControl.DeviceManagerCLI.dll")
clr.AddReference(kinesis_location + "Thorlabs.MotionControl.GenericMotorCLI.dll")
clr.AddReference(kinesis_location + "Thorlabs.MotionControl.Benchtop.BrushlessMotorCLI.dll")
from Thorlabs.MotionControl.DeviceManagerCLI import DeviceManagerCLI
from Thorlabs.MotionControl.GenericMotorCLI import (
    MotorDirection, DeviceUnitConverter
)
from Thorlabs.MotionControl.Benchtop.BrushlessMotorCLI import *
from Thorlabs.MotionControl.Benchtop.BrushlessMotorCLI import BenchtopBrushlessMotor
from System import Decimal



class BBDModels(StrEnum):
    BBD102 = "bbd102"
    BBD202 = "bbd202"
    BBD302 = "bbd302"



class _MLS203AxisConfig(LinearStageAxisConfig):
    vendor: str = Field(
        default           = "Thorlabs",
        json_schema_extra = {"ui": {"hidden": True}},
    )
    model: str = Field(
        default           = "MLS203-Axis",
        json_schema_extra = {"ui": {"hidden": True}},
    )


class _MLS203Axis(LinearStageAxis):
    # TODO, timeouts be part of core Stage API?
    HOME_TIMEOUT = units.Time('10 s')
    MOVE_TIMEOUT = units.Time('10 s')
    POLLING_PERIOD = units.Time('50 ms') # --> to a custom config?

    config_model = _MLS203AxisConfig
    settings_model = LinearStageAxisSettings

    def __init__(self, 
                 cfg: _MLS203AxisConfig,
                 mls203: "MLS203Stage", 
                 **kwargs):
        super().__init__(cfg, **kwargs)

        self._mls203 = mls203

    def _connect_impl(self) -> None:
        
        # Determine channel number (assumes x=1, y=2, z=3) and get channel object
        if self.axis == "x":
            channel_number = 1
        elif self.axis == "y":
            channel_number = 2
        else:
            channel_number = 3

        self._channel = self._mls203.bbd.GetChannel(channel_number) 
        
        if not self._channel.IsSettingsInitialized():
            self._channel.WaitForSettingsInitialized(10000)  # 10 second timeout
            if not self._channel.IsSettingsInitialized():
                raise RuntimeError("Failed to initialize stage axis")

        # Start polling and enable
        self._channel.StartPolling(int(1000*self.POLLING_PERIOD))

        time.sleep(2*self.POLLING_PERIOD)
        self._channel.EnableDevice()
        time.sleep(2*self.POLLING_PERIOD)  # Wait for device to enable

        # Load any configuration settings needed by the controller/stage
        # Device ID is the [serial no]-[channel]
        motor_config = self._channel.LoadMotorConfiguration(self._channel.DeviceID)  
        device_settings = self._channel.MotorDeviceSettings

        time.sleep(0.1) # wait 100 ms for axis to stabilize before taking initial position
        
        # We need to track previous position because Thorlabs's software will 
        # erroneously report position=0 right after issuing a move call. 
        # See note in position property, below.
        self._prev_position = self.position

        # If the stage begins at exactly 0, then add a small distance so fix below works
        if self._prev_position == units.Position("0 mm"):
            self._prev_position = units.Position("1 nm")

    def _introspect_position_limits(self) -> units.PositionRange:
        min_position = Decimal.ToDouble(
            self._channel.AdvancedMotorLimits.LengthMinimum) / 1000
        max_position = Decimal.ToDouble(
            self._channel.AdvancedMotorLimits.LengthMaximum) / 1000
        
        return units.PositionRange(min_position, max_position)
    
    @property
    def position_limits(self) -> units.PositionRange:
        # narrow the type hint, but use _StageAxis's implementation
        return cast(units.PositionRange, super().position_limits)

    @property
    def position(self) -> units.Position:
        """The current spatial position."""
        position = units.Position(
            Decimal.ToDouble(self._channel.Position) / 1000
        )

        # to fix BUG: immediately after MoveTo command, Position reads exactly 0
        if position == 0:
            # overwrite the current invalid position with the previous known position
            position = self._prev_position
        else:
            self._prev_position = position # store known position

        return position

    def move_to(self, position: units.Position, blocking: bool = False):
        """
        Initiate move to specified spatial position.

        Choose whether to return immediately (blocking=False, default) or to
        wait until finished moving (blocking=True). If the method does not
        return for more than 10 seconds in a blocking move, will timeout.
        """
        # Validate move position
        if not self.position_limits.within_range(position):
            raise ValueError(
                f"Requested move, ({position}) beyond limits, "
                f"min: {self.position_limits.min}, max: min: {self.position_limits.max}"
            )
        
        timeout_or_blocking = int(1000 * self.MOVE_TIMEOUT) if blocking else 0
        
        self._channel.MoveTo(Decimal(1000 * position), timeout_or_blocking) 
        
    @property
    def moving(self) -> bool:
        """Return True if the stage axis is currently moving."""
        return self._channel.IsDeviceBusy

    def move_velocity(self, velocity: units.Velocity):
        """Initiates movement at a constant velocity until stopped."""
        if not isinstance(velocity, units.Velocity):
            raise ValueError("velocity must be given in units.Velocity")
        if velocity > 0:
            motor_direction = MotorDirection.Forward
        else:
            motor_direction = MotorDirection.Backward
        
        unit_conv = self._channel.UnitConverter
        unit_type = unit_conv.UnitType.Velocity

        v_device_units = unit_conv.RealToDeviceUnit(Decimal(abs(velocity)*1000), unit_type)
        self._channel.MoveContinuousAtVelocity(motor_direction, v_device_units)

    def stop(self):
        """Halts motion."""
        self._channel.StopImmediate()

    def home(self, blocking: bool = False):
        """
        Initiate homing. 
        
        Choose whether to return immediately (blocking=False, default) or to
        wait until finished homing (blocking=True). If the method does not
        return for more than 10 seconds in a blocking move, will timeout.
        """
        timeout_or_blocking = int(1000 * self.MOVE_TIMEOUT) if blocking else 0
        self._channel.Home(timeout_or_blocking) # arg=0 for non-blocking

    @property
    def homed(self) -> bool:
        """Return whether the stage has been home."""
        return self._channel.Status.IsHomed

    @property
    def max_velocity(self) -> units.Velocity:
        """
        Return the current maximum velocity setting.

        Note that this is the imposed velocity limit for moves. It is not
        necessarily the maximum attainable velocity for this stage.
        """
        vel_params = self._channel.GetVelocityParams()
        v_max = Decimal.ToDouble(vel_params.MaxVelocity) 
        return units.Velocity(v_max / 1000)  # API uses velocity in mm/s

    @max_velocity.setter
    def max_velocity(self, new_velocity: units.Velocity):
        if not isinstance(new_velocity, units.Velocity):
            raise ValueError("Max velocity must be given in units.Velocity")
        
        vel_params = self._channel.GetVelocityParams()
        vel_params.MaxVelocity = Decimal(new_velocity * 1000) # API uses velocity in mm/s
        self._channel.SetVelocityParams(vel_params)

    @property
    def max_velocity_range(self) -> units.VelocityRange:
        return units.VelocityRange(
            min = units.Velocity("1 um/s"), # This is not specified, but we must set something greater than 0
            max = units.Velocity("250 mm/s") # The API returns 400 mm/s max, but the spec sheet only lists 250 mm/s.
        ) 

    @property
    def acceleration(self) -> units.Acceleration:
        """
        Return the acceleration used during ramp up/down phase of move.
        """
        vel_params = self._channel.GetVelocityParams()
        acceleration = Decimal.ToDouble(vel_params.Acceleration)
        return units.Acceleration(acceleration / 1000) # API uses velocity in mm/s^2

    @acceleration.setter
    def acceleration(self, new_acceleration: units.Acceleration):
        if not isinstance(new_acceleration, units.Acceleration):
            raise ValueError("Max velocity must be given in units.Acceleration")
        
        vel_params = self._channel.GetVelocityParams()
        vel_params.Acceleration = Decimal(new_acceleration * 1000) # API uses velocity in mm/s^2
        self._channel.SetVelocityParams(vel_params)

    @property
    def acceleration_range(self) -> units.AccelerationRange:
        return units.AccelerationRange(
            min = units.Acceleration("10 um/s^2"), # This is not specified, but we must set something greater than 0
            max = units.Acceleration("2000 mm/s^2") # API returns 2.48056m/s^2, but the spec sheet lists 2000 mm/s^2
        ) 


class MLS203StageConfig(XYStageConfig):
    vendor: Literal["Thorlabs"] = Field(
        default           = "Thorlabs",
        json_schema_extra = {"ui": {"hidden": True}},
    )
    model: Literal["MLS203"] = Field(
        default           = "MLS203",
        json_schema_extra = {"ui": {"hidden": True}},
    )
    x: _MLS203AxisConfig = Field(
        default_factory = lambda: _MLS203AxisConfig(axis="x"),
        description     = "X axis (MLS203 channel 1)."
    )
    y: _MLS203AxisConfig = Field(
        default_factory = lambda: _MLS203AxisConfig(axis="y"),
        description     = "Y axis (MLS203 channel 2)."
    )
    controller_model: BBDModels | None = Field(
        default           = None, # this will be introspected
        json_schema_extra = {"ui": {"hidden": True}},
    )


class MLS203Stage(XYStage):
    """
    Entry point to control Thorlabs MLS203, a high-speed, low-profile motorized 
    XY scanning stage.
    
    https://www.thorlabs.com/item/MLS203-1

    Compatible controllers:
    https://www.thorlabs.com/item/BBD102
    https://www.thorlabs.com/item/BBD202
    https://www.thorlabs.com/item/BBD302
    """
    SERIAL_NUMBER_PREFIX = "73" # prefix identifying BBD-series controllers

    def __init__(self, cfg: MLS203StageConfig, **kwargs):
        super().__init__(cfg, **kwargs)

        self._x = _MLS203Axis(cfg.x, self)
        self._y = _MLS203Axis(cfg.y, self)

        # Build the device list, required even when device is known
        DeviceManagerCLI.BuildDeviceList()

        # Retrieve all Kinesis devices
        connected_devices_sn: list[str] = list(DeviceManagerCLI.GetDeviceList())

        # Find device matching BBD102
        self._bbd_sn = None
        for device_sn in connected_devices_sn:
            if not device_sn.startswith(self.SERIAL_NUMBER_PREFIX):
                continue # Not a BBD device, something else
            else:
                self._bbd_sn = device_sn
            
        if self._bbd_sn is None:
            raise RuntimeError(
                "BBD-series controller not found among available devices. Check "
                "that it is connected and not in used by another program."
            )

        # if not self._x_axis.homed:
        #     self._x_axis.home(blocking=False)
        # if not self._y_axis.homed:
        #     self._y_axis.home(blocking=False)

    def _connect_impl(self) -> None:
        try:
            # Attempt to create and connect to a BenchtopBrushlessMotor
            self.bbd = BenchtopBrushlessMotor.CreateBenchtopBrushlessMotor(
                self._bbd_sn
            )
            self.bbd.Connect(self._bbd_sn)
        except Exception as e:
            print(f"Failed to connect to device {self._bbd_sn}: {e}")
            # Should this raise rather than print?

        self._x.connect()
        self._y.connect()

    def _close_impl(self) -> None:
        pass # TODO

    def _introspect_identity(self) -> dict[str, str]:
        introspected = {}
        introspected["serial"] = self._bbd_sn
        # introspected["hardware_rev"]
        introspected["firmware"] = str(self.bbd.FirmwareVersion)
        # driver, driver version
        return introspected





# For testing
if __name__ == "__main__":

    cfg = MLS203StageConfig()

    stage = MLS203Stage(cfg)
    stage.connect()
    
    stage.x.position_limits


    a=1