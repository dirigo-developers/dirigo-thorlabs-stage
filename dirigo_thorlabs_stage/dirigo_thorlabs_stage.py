import time

import clr

from dirigo.interfaces.stage import Stage, StageAxis


# Import .NET resources
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


class BBD102Stage(Stage):
    """
    Communicate with Thorlabs BBD102 brushless motor controller, which normally
    controls MLS203, a high-speed 2-axis linear motor stage.
    
    https://www.thorlabs.com/thorproduct.cfm?partnumber=BBD102
    https://www.thorlabs.com/thorproduct.cfm?partnumber=MLS203-1
    """
    SERIAL_NUMBER_PREFIX = "73"

    def __init__(self, defaults=None):
        # Build the device list, required even when device is known
        DeviceManagerCLI.BuildDeviceList()

        # Retrieve all connected devices
        connected_devices_sn:list[str] = list(DeviceManagerCLI.GetDeviceList())

        # Find device matching BBD102
        self._controller = None
        for device_sn in connected_devices_sn:
            if not device_sn.startswith(self.SERIAL_NUMBER_PREFIX):
                # Not a BBD102 device
                continue

            try:
                # Attempt to create and connect to a BenchtopBrushlessMotor
                controller = BenchtopBrushlessMotor.CreateBenchtopBrushlessMotor(device_sn)
                    
                controller.Connect(device_sn)

                self._controller = controller

            except Exception as e:
                print(f"Failed to connect to device {device_sn}: {e}")

        if self._controller is None:
            raise RuntimeError("BBD102 not found among connected devices.")

        self._x_axis = BBD102Axis(
            self._controller, 
            channel_number=1, 
            defaults=defaults
        ) 

        self._y_axis = BBD102Axis(
            self._controller, 
            channel_number=2,
            defaults=defaults
        )
    
    @property
    def x(self):
        return self._x_axis
    
    @property
    def y(self):
        return self._y_axis
    
    @property
    def z(self):
        return None


class BBD102Axis(StageAxis):
    """
    Represents a stage single axis controlled by BBD102.

    Note: this should not be instantiated alone, requires BBD102Stage.
    """
    HOMING_TIMEOUT = 10 # seconds
    MOVE_TIMEOUT = 10 # seconds
    POLLING_PERIOD = 0.050 # seconds

    def __init__(self, controller, channel_number:int, defaults:dict):
        self.channel_number = channel_number
        self._channel = controller.GetChannel(self.channel_number) # 1-indexed channel count
        self._prev_position = None
        self._last_move_timestamp = None
        
        # if "distance_per_pulse" in settings: # If position quad encoders are connected
        #     self.distance_per_pulse = to_si_units(settings["distance_per_pulse"])

        #     if self.channel_number == 1:
        #         axis_label = "x"
        #     elif self.channel_number == 2:
        #         axis_label = "y"
        #     else:
        #         raise ValueError(">2 channels not supported")
        #     self.encoder_a:str = settings["wiring"][axis_label]["encoder_a"]
        #     self.encoder_b:str = settings["wiring"][axis_label]["encoder_b"]

        # if "trigger_out" in settings["wiring"][axis_label]:
        #     terminal_name = settings["wiring"][axis_label]["trigger_out"]
        #     if terminal_name[:4] == "/Dev":
        #         terminal_name = terminal_name
        #     elif terminal_name[:3]:
        #         terminal_name = "/Dev1/" + terminal_name
        #     self.trigger_out = terminal_name
            
        if not self._channel.IsSettingsInitialized():
            self._channel.WaitForSettingsInitialized(10000)  # 10 second timeout
            assert self._channel.IsSettingsInitialized() is True

        # Start polling and enable
        self._channel.StartPolling(int(1000*self.POLLING_PERIOD))

        time.sleep(2*self.POLLING_PERIOD)
        self._channel.EnableDevice()
        time.sleep(2*self.POLLING_PERIOD)  # Wait for device to enable

        # Load any configuration settings needed by the controller/stage
        # Device ID is the [serial no]-[channel]
        motor_config = self._channel.LoadMotorConfiguration(self._channel.DeviceID)  
        device_settings = self._channel.MotorDeviceSettings

        # if self.homed is False:
        #     self.home()

        # step_size = 1e-3
        # max_velocity = to_si_units(settings["velocity"])
        # acceleration = to_si_units(settings["acceleration"])
        # self.set_jog_parameters(step_size, max_velocity, acceleration) 

    @property
    def busy(self):
        return self._channel.IsDeviceBusy

    @property
    def homed(self):
        return bool(self._channel.Status.IsHomed)

    def home(self, blocking=False):
        
        if not blocking:
            self._channel.Home(0) # argument = 0 codes for non-blocking
        else:
            self._channel.Home(int(1000*self.HOMING_TIMEOUT))
    
    @property
    def max_velocity(self):
        vel_params = self._channel.GetVelocityParams()
        v_max_mm_per_sec = Decimal.ToDouble(vel_params.MaxVelocity) 
        return v_max_mm_per_sec / 1000 # convert to m/s

    @max_velocity.setter
    def max_velocity(self, max_velo_m_per_s:float):
        value_mm_per_sec = max_velo_m_per_s * 1000 # convert to mm/s
        vel_params = self._channel.GetVelocityParams()
        vel_params.MaxVelocity = Decimal(value_mm_per_sec)
        self._channel.SetVelocityParams(vel_params)

    @property
    def acceleration(self):
        velo_params = self._channel.GetVelocityParams()
        a_mm_per_sec2 = Decimal.ToDouble(velo_params.Acceleration)
        return a_mm_per_sec2 / 1000 # convert to m/s^2

    @acceleration.setter
    def acceleration(self, accel_m_per_s2:float):
        value_mm_per_sec2 = accel_m_per_s2 * 1000
        vel_params = self._channel.GetVelocityParams()
        vel_params.Acceleration = Decimal(value_mm_per_sec2)
        self._channel.SetVelocityParams(vel_params)

    @property
    def position(self):
        p_mm = Decimal.ToDouble(self._channel.Position)
        p = p_mm / 1000 # convert to meters

        # to fix BUG: immediately after issuing MoveTo command, reads 0.0
        if p == 0.0:
            # overwrite invalid position (0.0) with the last known postiion
            p = self._prev_position
        else:
            self._prev_position = p

        return p

    def move_to_position(self, position:float, blocking:bool=False):
        # Check within limits
        if not (self.min_position <= position <= self.max_position):
            raise ValueError(f"Requested move beyond limits ({position})")

        new_pos_mm = Decimal(position * 1000) # convert to mm
        if not blocking:
            self._channel.MoveTo(new_pos_mm, int(0)) # 2nd arg=0 means non-blocking
        else:
            self._channel.MoveTo(new_pos_mm, int(1000*self.MOVE_TIMEOUT)) 
            
        self._last_move_timestamp = time.perf_counter() # TODO, how is this used?

    def stop(self):
        self._channel.StopImmediate()

    @property
    def min_position(self):
        min_position_mm = Decimal.ToDouble(
            self._channel.AdvancedMotorLimits.LengthMinimum)
        return min_position_mm / 1000 # convert to mm

    @property
    def max_position(self):
        max_position_mm = Decimal.ToDouble(
            self._channel.AdvancedMotorLimits.LengthMaximum)
        return max_position_mm / 1000 # convert to mm

    @property
    def home_position(self):
        return (self.max_position + self.min_position) / 2


if __name__ == "__main__":

    stage = BBD102Stage()

    None