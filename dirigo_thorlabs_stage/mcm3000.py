from enum import IntEnum
from functools import cached_property
from typing import cast
import time
import serial
import threading

from dirigo import units
from dirigo.hw_interfaces.scanner import ObjectiveZScanner



class MCM3000Enums(IntEnum):
    CMD_SET_ENCODER_COUNTER         = 0x09
    CMD_STOP                        = 0x65
    CMD_QUERY_POSITION              = 0x0A
    RESP_QUERY_POSITION             = 0x0B
    CMD_GO_TO_POSITION              = 0x53
    CMD_QUERY_REQUEST_MOTOR_STATUS  = 0x80
    NO_MODE                         = 0x00
    STOP_MODE_ABRUPT                = 0x01


class MCM3000Controller:
    """Controller for command composition, send, and receive via Serial port."""
    def __init__(self, com_port: int, channel: int, baud_rate: int = 460800):
        if not isinstance(com_port, int):
            raise ValueError("`com_port` must be an integer")
        self._port = com_port

        if not isinstance(baud_rate, int):
            raise ValueError("`baud_rate` must be an integer")
        self._baud_rate = baud_rate

        if not isinstance(channel, int):
            raise ValueError("`channel` must be an integer")
        self._channel = channel

        self._lock = threading.Lock()

        self.open_serial_port()

    def open_serial_port(self) -> None:       
        self._serial_port = serial.Serial(
            port=f"COM{self._port}",
            baudrate=self._baud_rate,
            timeout=1,
            write_timeout=1
        )

    def make_command(self, 
                     command: MCM3000Enums, 
                     mode: MCM3000Enums = MCM3000Enums.NO_MODE, 
                     data: int | None = None) -> bytearray:
        cmd = bytearray()

        # 6 Header bytes:
        cmd.append(command)         # byte0: command
        cmd.append(0x04)            # byte1: constant
        cmd.append(0x06 if command is MCM3000Enums.CMD_GO_TO_POSITION
                   else self._channel)  # byte6: constant or channel identifier
        cmd.append(mode)            # byte3: command mode/sub-command
        cmd.extend(b'\x00\x00')     # bytes4-5: empty

        # Data bytes, if applicable
        if data is not None:
            cmd.extend(self._channel.to_bytes(2, 'little'))
            cmd.extend(data.to_bytes(4, 'little', signed=True))
        return cmd

    def send_receive(self, cmd: bytearray):
        with self._lock:
            self._serial_port.reset_input_buffer()
            self._serial_port.reset_output_buffer()
            self._serial_port.write(cmd)
            self._serial_port.flush()

            if cmd[0] == MCM3000Enums.CMD_QUERY_POSITION:
                read_bytes = 12
            elif cmd[0] == MCM3000Enums.CMD_QUERY_REQUEST_MOTOR_STATUS:
                read_bytes = 34
            else:
                read_bytes = 0

            if read_bytes:
                response = self._serial_port.read(read_bytes)
            else:
                response = None

        return self._interpret_response(response)
    
    def _interpret_response(self, resp: bytes | None):
        if resp is not None:
            if resp[0] == MCM3000Enums.RESP_QUERY_POSITION and len(resp) == 12:
                return int.from_bytes(resp[8:], byteorder='little', signed=True)
            elif len(resp) == 34:
                return (resp[16] & 0x30) != 0
            else:
                raise RuntimeError(f"Received an unexpected number of bytes, {len(resp)}")
        else:
            return None


class MCM3000(ObjectiveZScanner):
    LENGTH_PER_COUNT = units.Position("211.6667 nm")

    def __init__(self, com_port: int, **kwargs):
        super().__init__(**kwargs)

        z_channel = 2
        self._controller = MCM3000Controller(com_port, channel=z_channel)

    @cached_property
    def _position_command(self):
        return self._controller.make_command(
            command=MCM3000Enums.CMD_QUERY_POSITION,
            mode=MCM3000Enums.NO_MODE,
        )
    
    @property
    def position(self) -> units.Position:
        """The current position."""
        encoder_cnts = cast(int, self._controller.send_receive(self._position_command))
        return self.LENGTH_PER_COUNT * encoder_cnts

    def move_to(self, position: units.Position, blocking: bool = False):
        """
        Initiate move to specified spatial position.

        Choose whether to return immediately (blocking=False, default) or to
        wait until finished moving (blocking=True).
        """
        cmd = self._controller.make_command(
            command=MCM3000Enums.CMD_GO_TO_POSITION,
            mode=MCM3000Enums.NO_MODE,
            data=round(position / self.LENGTH_PER_COUNT)
        )
        self._controller.send_receive(cmd)

        if blocking: # TODO, check if this works, adjust polling times
            time.sleep(0.050)
            while self.moving:
                time.sleep(0.050)

    @property
    def moving(self) -> bool:   
        """Return True if the stage axis is currently moving."""
        cmd = self._controller.make_command(
            command=MCM3000Enums.CMD_QUERY_REQUEST_MOTOR_STATUS,
            mode=MCM3000Enums.NO_MODE
        )
        return cast(bool, self._controller.send_receive(cmd))
    
    def move_velocity(self, velocity):
        raise NotImplementedError("Constant velocity moves not supported on MCM3000 device.")

    def stop(self):
        """Halts motion."""
        cmd = self._controller.make_command(
            command=MCM3000Enums.CMD_STOP,
            mode=MCM3000Enums.STOP_MODE_ABRUPT
        )
        self._controller.send_receive(cmd)

    def home(self, blocking: bool = False):
        raise NotImplementedError("Homing is not supported on MCM3000 device.")

    @property
    def homed(self) -> bool:
        raise NotImplementedError("Homing is not supported on MCM3000 device.")
    
    @property
    def max_velocity(self) -> units.Velocity:
        raise NotImplementedError("Cannot change max velocity on MCM3000 device.")

    @max_velocity.setter
    def max_velocity(self, value:units.Velocity):
        raise NotImplementedError("Cannot change max velocity on MCM3000 device.")

    @property
    def acceleration(self) -> units.Acceleration:
        raise NotImplementedError("Cannot change acceleration on MCM3000 device.")

    @acceleration.setter
    def acceleration(self, value: units.Acceleration):
        raise NotImplementedError("Cannot change acceleration on MCM3000 device.")

    @property
    def device_info(self):
        pass

    @property
    def position_limits(self):
        return units.PositionRange('-5 mm', '5 mm')
    
    

