import serial
import re


class LoopODriver:
    def __init__(self, com_port="/dev/ttyACM0", baud=115200):
        self._com_port = com_port
        self._baud = baud
        self._serial_interface = serial.Serial(self._com_port, self._baud, timeout=1)

        self.ex_position = 0
        self.ex_status = 0
        self.ex_control = 0
        self.ex_position = 0
        self.ex_status = 0
        self.ex_control = 0

        self.tw_size = 0
        self.tw_offset = 0
        self.tw_status = 0
        self.tw_control = 0
        self.tw_size = 0
        self.tw_offset = 0
        self.tw_statuts = 0
        self.tw_control = 0

        self.lp_size = 0
        self.force = 0.53
        self.lp_status = 0
        self.lp_control = 0

    def send_command(self, id=None, command=None, value=None):
        command = str(id) + str(command) + str(value) + r"\n"

        self._serial_interface.write(command.encode("ascii"))
        self._serial_interface.flush()

        line = self._serial_interface.readline().decode("ascii").rstrip()

        values = re.findall(r"\d+(?:\.\d+)?", line)

        self.ex_position = int(values[0])
        self.ex_status = int(values[1])
        self.ex_control = int(values[2])

        self.tw_size = int(values[3])
        self.tw_offset = int(values[4])
        self.tw_status = int(values[5])
        self.tw_control = int(values[6])

        self.lp_size = int(values[7])
        self.force = float(values[8])
        self.lp_status = int(values[9])
        self.lp_control = int(values[10])
