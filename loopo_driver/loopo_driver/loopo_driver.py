import serial
import re


class LoopODriver:
    """Loop-O Driver Class

    This class is used to create an ergonomic interface with the Loop-O device.
    It uses pyserial to communicate with device. It provides a method that exposes
    the command inteface directly as well as series of convenience method
    that can abstract the command interface into more intuitive functions.
    """

    def __init__(self, com_port="/dev/ttyACM0", baud=115200):
        """Loop-O Driver Initialiser method

        Parameters
        ----------
        com_port: str
            Path to the serial interface of the Loop-O
        baud: int
            Baudrate for the serial communication interface
        """
        self._com_port = com_port
        self._baud = baud

        self._serial_interface = serial.Serial(self._com_port, self._baud, timeout=1)

        self.EXTESION = 1
        self.TWIST = 2
        self.LOOP = 3

        self.SET_ENABLE = 0
        self.SET_CONTROL = 1
        self.SET_SPEED = 2
        self.SET_POSITION = 3
        self.SET_VELOCITY = 4
        self.HOME = 5
        self.SET_TWIST = 6
        self.SET_FORCE = 6

        self.SPEED = 0.0
        self.POSITION = 1.0
        self.VELOCITY = 2.0
        self.FORCE = 3.0

        self.extension_position = 0
        self.extension_status = 0
        self.extension_control = 0

        self.twist_size = 0
        self.twist_offset = 0
        self.twist_status = 0
        self.twist_control = 0

        self.loop_size = 0
        self.force = 0.53
        self.loop_status = 0
        self.loop_control = 0

    def __send(self, id=None, command=None, value=None):
        """Internal method that formats and send command to the Loop-0

        Parameters
        ----------
        id: int
            identity of the device to send the command to
            0 - No Device
            1 - Extension
            2 - Twist
            3 - Loop
            sending 0 will prompt a reply from the device.
        command: int
            command code to be sent
            0 - Set Enable
                possible values: 0 - Disable
                                 1 - Enable
            1 - Set Control Approach
                possible values: 0 - Speed
                                 1 - Position
                                 2 - Velocity
                                 3 - Force
            2 - Set Motor Speed
            3 - Set Position
            4 - Set Velocity
            5 - Homing
                requires a homing speed for the value
            6 - Set force
        value: float
            value associted withe command to be sent
        """
        command = str(id) + str(command) + str(value) + r"\n"
        self._serial_interface.write(command.encode("ascii"))
        self._serial_interface.flush()

    def __receive(self):
        """Interanl method that reads the Loop-O reply and updates the classe's state"""
        line = self._serial_interface.readline().decode("ascii").rstrip()

        values = re.findall(r"\d+(?:\.\d+)?", line)

        self.extensionposition = int(values[0])
        self.extensionstatus = int(values[1])
        self.extensioncontrol = int(values[2])

        self.twist_size = int(values[3])
        self.twist_offset = int(values[4])
        self.twist_status = int(values[5])
        self.twist_control = int(values[6])

        self.loop_size = int(values[7])
        self.force = float(values[8])
        self.loop_status = int(values[9])
        self.loop_control = int(values[10])

    def send_command(self, id=None, command=None, value=None):
        """Internal method that formats and send command to the Loop-0 and updates the class with the reply

        Parameters
        ----------
        id: int
            identity of the device to send the command to
            0 - No Device
            1 - Extension
            2 - Twist
            3 - Loop
            sending 0 will prompt a reply from the device.
        command: int
            command code to be sent:
            0 - Set Enable
                possible values: 0 - Disable
                                 1 - Enable
            1 - Set Control Approach
                possible values: 0 - Speed
                                 1 - Position
                                 2 - Velocity
                                 3 - Force
            2 - Set Motor Speed
            3 - Set Position
            4 - Set Velocity
            5 - Homing
                requires a homing speed for the value
            6 - Set force
        value: float
            value associted withe command to be sent
        """
        self.__send(id, command, value)
        self.__receive()

    def update(self):
        """Send an empty command string to receive a status update from the Loop-O"""
        self._serial_interface.write("\n".encode("ascii"))
        self._serial_interface.flush()
        self.__receive()

    def enable(self, id=0):
        """Send enable command to a device

        Parameters
        ----------
        id: int
            identity of the device to send the command to
            1 - Extension
            2 - Twist
            3 - Loop
        """
        self.__send(id, self.SET_ENABLE, 1.0)
        self.__receive()

    def disable(self, id=0):
        """Send disable command to a device

        Parameters
        ----------
        id: int
            identity of the device to send the command to
            1 - Extension
            2 - Twist
            3 - Loop
        """
        self.__send(id, self.SET_ENABLE, 0.0)
        self.__receive()

    def switch_to_speed_control(self, id=0):
        """Switch a device to speed control

        Parameters
        ----------
        id: int
            identity of the device to send the command to
            1 - Extension
            2 - Twist
            3 - Loop
        """
        self.__send(id, self.SET_CONTROL, self.SPEED)
        self.__receive()

    def switch_to_position_control(self, id=0):
        """Switch a device to position control

        Parameters
        ----------
        id: int
            identity of the device to send the command to
            1 - Extension
            2 - Twist
            3 - Loop
        """
        self.__send(id, self.SET_CONTROL, self.POSITION)
        self.__receive()

    def switch_to_velocity_control(self, id=0):
        """Switch a device to velocity control

        Parameters
        ----------
        id: int
            identity of the device to send the command to
            1 - Extension
            2 - Twist
            3 - Loop
        """
        self.__send(id, self.SET_CONTROL, self.VELOCITY)
        self.__receive()

    def switch_to_force_control(self):
        """Switch a device to position control

        Parameters
        ----------
        id: int
            identity of the device to send the command to
            1 - Extension
            2 - Twist
            3 - Loop
        """
        self.__send(self.LOOP, self.SET_CONTROL, self.FORCE)
        self.__receive()

    def set_extension_position(self, postion=0.0):
        """Set the target postion for the extension device

        Parameters
        ----------
            position: float
                target postion
        """
        self.__send(self.EXTESION, self.SET_POSITION, postion)
        self.__receive()

    def home_extension(self, speed=10.0):
        """Start the homing procedure fopr the extension device

        Parameters
        ----------
            speed: float
                homing speed
        """
        self.__send(self.EXTESION, self.HOME, speed)
        self.__receive()

    def set_twist_size(self, size=0.0):
        """Set the target size for the twist device

        Parameters
        ----------
            position: int
                target postion
        """
        self.__send(self.TWIST, self.SET_POSITION, size)
        self.__receive()

    def set_twist(self, twist=0.0):
        """Set the target offset for the twist device

        Parameters
        ----------
            position: int
                target postion
        """
        self.__send(self.TWIST, self.SET_TWIST, twist)
        self.__receive()

    def home_twist(self, speed=10.0):
        """Start the homing procedure fopr the twist device

        Parameters
        ----------
            speed: float
                homing speed
        """
        self.__send(self.TWIST, self.HOME, speed)
        self.__receive()

    def set_loop_size(self, size=0.0):
        """Set the target size for the loop device

        Parameters
        ----------
            twist: float
                target twist
        """
        self.__send(self.LOOP, self.SET_POSITION, size)
        self.__receive()

    def set_force(self, force=0.0):
        """Set the target size for the loop device

        Parameters
        ----------
            force: float
                target force
        """
        self.__send(self.LOOP, self.SET_FORCE, force)
        self.__receive()

    def home_loop(self, speed=10.0):
        """Start the homing procedure fopr the loop device

        Parameters
        ----------
            speed: float
                homing speed
        """
        self.__send(self.LOOP, self.HOME, speed)
        self.__receive()
