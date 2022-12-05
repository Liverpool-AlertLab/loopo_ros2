import serial
import re
from time import sleep

class loopo_driver:
    def __init__(self, com_port = '/dev/ttyACM0', baud = 115200):

            self._com_port = com_port
            self._baud = baud
            self._serial_interface = serial.Serial(self._com_port, self._baud, timeout=1)

            self.ex_position = 0
            self.ex_status = 0
            self.ex_control = 0

            self.tw_position = 0
            self.tw_offset = 0
            self.tw_statuts = 0
            self.tw_control = 0

            self.lp_position = 0
            self.force = 0.53
            self.lp_status = 0
            self.lp_control = 0

            self.ack = 0

            sleep(0.5)
    
    def update(self):

        if self._serial_interface.in_waiting > 0:
            
            line = self._serial_interface.readline().decode('ascii').rstrip()
            values = re.findall(r'\d+(?:\.\d+)?',line)

            if (len(values)>0):
                print(line)
                print(values)
                
                self.ex_position = int(line[0])
                self.ex_status = int(values[1])
                self.ex_control = int(values[2])

                self.tw_position = int(values[3])
                self.tw_offset = int(values[4])
                self.tw_statuts = int(values[5])
                self.tw_control = int(values[6])

                self.lp_position = int(values[7])
                self.force = float(values[8])
                self.lp_status = int(values[9])
                self.lp_control = int(values[10])

                self.ack = bool(int(values[11]))


    def send_command(self, id, command, value):
        command = str(id) + str(command) + str(value) + "\n"
        self._serial_interface.write(command.encode('ascii'))
        self._serial_interface.flush()
        while not self.ack:
            sleep(0.01)
        return 1




