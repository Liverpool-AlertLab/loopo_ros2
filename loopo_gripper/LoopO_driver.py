import serial
import re

EX = 0
TR = 1
TL = 2
LP = 3

TORQUE_ENABLE = 0,
CONTROL_APPROACH = 1
GOAL_SPEED = 2
GOAL_POSITION = 3
GOAL_VELOCITY = 4
ZERO_ENCODER = 5

NO_CONTROL = 0
POSITION_CONTROL = 1
VELOCITY_CONTROL = 2

class loopo_driver:
    def __init__(self, com_port = '/dev/ttyACM0', baud = 115200):

            self._com_port = com_port
            self._baud = baud
            self._serial_interface = serial.Serial(self._com_port, self._baud, timeout=1)

            self.ex_endstop = False
            self.tw_endstop = False
            self.tr_runout = False
            self.tl_runout = False
            self.lp_runout = False
            self.force = 0.53
            self.ex_position = 0
            self.tr_position = 0
            self.tl_position = 0
            self.lp_position = 0

    
    def update(self):

        if self._serial_interface.in_waiting > 0:
            
            line = self._serial_interface.readline().decode('ascii').rstrip()
            values = re.findall(r'\d+(?:\.\d+)?',line)

            self.ex_endstop = bool(values[0])
            self.tw_endstop = bool(values[1])
            self.tr_runout = bool(values[2])
            self.tl_runout = bool(values[3])
            self.lp_runout = bool(values[4])
            self.force = float(values[5])

            self.ex_position = int(values[6])
            self.tr_position = int(values[7])
            self.tl_position = int(values[8])
            self.lp_position = int(values[9])

    def send_command(self, id, command, value):

        command = str(id) + str(command) + str(value) + "\n"
        self._serial_interface.write(command.encode())
        while(self._serial_interface.inWaiting):
            print(1)
        line = self._serial_interface.readline().decode('ascii').rstrip()
        if line == "ACK\n":
            return 0
        else:
            return 1


