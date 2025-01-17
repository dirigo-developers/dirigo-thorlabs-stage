import time
from dataclasses import dataclass
from functools import cached_property

import clr

from dirigo import units
from dirigo.hw_interfaces.stage import MultiAxisStage, LinearStage, StageInfo



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




class BBD102Stage(MultiAxisStage):
    """
    Communicate with Thorlabs BBD102 brushless motor controller, which usually
    controls MLS203, a high-speed 2-axis linear motor stage.
    
    https://www.thorlabs.com/thorproduct.cfm?partnumber=BBD102
    https://www.thorlabs.com/thorproduct.cfm?partnumber=MLS203-1
    """
    SERIAL_NUMBER_PREFIX = "73" # code Thorlabs uses to identify this device type

    def __init__(self, x_config: dict, y_config: dict, **kwargs):
        # Build the device list, required even when device is known
        DeviceManagerCLI.BuildDeviceList()

        # Retrieve all connected devices
        connected_devices_sn:list[str] = list(DeviceManagerCLI.GetDeviceList())

        # Find device matching BBD102
        self._controller = None
        for device_sn in connected_devices_sn:
            if not device_sn.startswith(self.SERIAL_NUMBER_PREFIX):
                # Not a BBD102 device, something else
                continue

            try:
                # Attempt to create and connect to a BenchtopBrushlessMotor
                controller = BenchtopBrushlessMotor.CreateBenchtopBrushlessMotor(device_sn)
                controller.Connect(device_sn)
            except Exception as e:
                print(f"Failed to connect to device {device_sn}: {e}")

            self._controller = controller

        if self._controller is None:
            raise RuntimeError(
                "BBD102 not found among available devices. Check that it is "
                "connected and not in used by another program."
            )

        self._x_axis = ThorlabsLinearMotor(self._controller, **x_config)
        self._y_axis = ThorlabsLinearMotor(self._controller, **y_config)
    
    @property
    def x(self):
        """Returns reference to x axis motor."""
        return self._x_axis
    
    @property
    def y(self):
        """Returns reference to y axis motor."""
        return self._y_axis
    
    @property
    def z(self):
        """No z axis motor available."""
        return None
        # alternatively we could raise a NotImplementedError?


@dataclass
class ThorlabsLinearMotorInfo(StageInfo):
    # inherits StageInfo's attributes and adds the following...
    serial_number: str
    # extend as needed


class ThorlabsLinearMotor(LinearStage): # alt name: Linear brushless?
    HOME_TIMEOUT = units.Time('10 s')
    MOVE_TIMEOUT = units.Time('10 s')
    POLLING_PERIOD = units.Time('50 ms')

    def __init__(self, stage_controller: BBD102Stage, 
                 position_limits: dict = None, **kwargs):
        super().__init__(**kwargs)

        # position limits may be set manually in system_config.toml, or omitted to use 
        if position_limits is None:
            self._position_limits = None # position limit property will read limits from API
        else:
            self._validate_limits_dict(position_limits)
            self._position_limits = units.PositionRange(**position_limits)
        
        channel_number = 1 if self.axis == 'x' else 2
        self._channel = stage_controller.GetChannel(channel_number) 
        
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

        self._prev_position = self.position # BUG this doesn't work if at 0.0 at startup (see note on BUG in position getter)

    @cached_property 
    def device_info(self) -> ThorlabsLinearMotorInfo:
        """Returns an object describing permanent properties of the stage."""
        return ThorlabsLinearMotorInfo(
            manufacturer="Thorlabs",
            model="MLS203", # should name the actuator
            serial_number=self._channel.DeviceID
            # controller name, etc
            # serial number -- self._channel.DeviceID
        )

    @cached_property
    def position_limits(self) -> units.PositionRange:
        if self._position_limits is None:
            min_position = Decimal.ToDouble(
                self._channel.AdvancedMotorLimits.LengthMinimum) / 1000
            max_position = Decimal.ToDouble(
                self._channel.AdvancedMotorLimits.LengthMaximum) / 1000
            return units.PositionRange(min_position, max_position)
        else:
            return self._position_limits
        
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
        
        self._last_move_timestamp = time.perf_counter() # why?

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
            velocity = abs(velocity)
            motor_direction = MotorDirection.Backward
        
        unit_conv = self._channel.UnitConverter
        unit_type = unit_conv.UnitType.Velocity

        v_device_units = unit_conv.RealToDeviceUnit(Decimal(velocity*1000), unit_type)
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
        # TODO: validate
        vel_params = self._channel.GetVelocityParams()
        vel_params.MaxVelocity = Decimal(new_velocity * 1000) # API uses velocity in mm/s
        self._channel.SetVelocityParams(vel_params)

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
        # TODO, validate
        vel_params = self._channel.GetVelocityParams()
        vel_params.Acceleration = Decimal(new_acceleration * 1000) # API uses velocity in mm/s^2
        self._channel.SetVelocityParams(vel_params)





# For testing
if __name__ == "__main__":

    x_config = {
        "axis": "x",
        "position_limits": {"min": "1 mm", "max": "99 mm"} 
    }
    y_config = {
        "axis": "y",
        "position_limits": {"min": "1 mm", "max": "74 mm"} 
    }

    stage = BBD102Stage(x_config, y_config)

    print(stage.x.position)
    stage.x.move_velocity(units.Velocity('0.35 mm/s'))

    time.sleep(3)
    stage.x.stop()
    print(stage.x.position)