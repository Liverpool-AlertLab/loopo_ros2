import serial
import re
from time import sleep

EX = 0
TR = 1
TL = 2
LP = 3

TORQUE_ENABLE = 0
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
        while self.ack:
            sleep(0.01)
        return 1

    def home_extension(self, speed):
        homing_speed = -speed
        self.send_command(EX, GOAL_SPEED, homing_speed)
        if self.ex_control_flag != NO_CONTROL:
            self.send_command(EX, CONTROL_APPROACH, NO_CONTROL)
            self.ex_control_flag = NO_CONTROL
        if not self.ex_torque_flag:
            self.send_command(EX, TORQUE_ENABLE, 1)
            self.ex_torque_flag = 1
        while self.ex_endstop:
            sleep(0.01)
        self.send_command(EX, GOAL_SPEED, 0)
        self.send_command(EX, ZERO_ENCODER, 0)
        return 1
    

    def move_extension(self, position, blocking = True, tolerance = 10):
        self.send_command(EX, GOAL_POSITION, position)
        if self.ex_control_flag != POSITION_CONTROL:
            self.send_command(EX, CONTROL_APPROACH, POSITION_CONTROL)
            self.ex_control_flag = POSITION_CONTROL
        if not self.ex_torque_flag:
            self.send_command(EX, TORQUE_ENABLE, 1)
            self.ex_torque_flag = 1
        if blocking:
            while ((self.ex_position <= position - 10) or (self.ex_endstop >= position + 10)):
                sleep(0.01)
        return 1
    
    def home_twist(self, speed):
        homing_speed = -speed
        self.send_command(TR, GOAL_SPEED, -homing_speed)
        self.send_command(TL, GOAL_SPEED, -homing_speed)
        if self.tr_control_flag != NO_CONTROL:
            self.send_command(TR, CONTROL_APPROACH, NO_CONTROL)
            self.tr_control_flag = NO_CONTROL
        if self.tl_control_flag != NO_CONTROL:
            self.send_command(TL, CONTROL_APPROACH, NO_CONTROL)
            self.tl_control_flag = NO_CONTROL
        if not self.tr_torque_flag:
            self.send_command(TR, TORQUE_ENABLE, 1)
            self.tr_torque_flag = 1
        if not self.tl_torque_flag:
            self.send_command(TL, TORQUE_ENABLE, 1)
            self._torque_flag = 1
        r_homed = 0
        l_homed = 0
        while 1:
            if not self.tr_runout :
                if not r_homed:
                    self.send_command(TR, GOAL_SPEED, 0)
                    r_homed = 1
                    print("r_homed")
            if not self.tl_runout:
                if not l_homed:
                    self.send_command(TL, GOAL_SPEED, 0)
                    l_homed = 1
                    print("l_homed")
            if r_homed and l_homed:
                break
            sleep(0.01)
        self.send_command(TR, GOAL_SPEED, homing_speed)
        self.send_command(TL, GOAL_SPEED, homing_speed)
        while self.tw_endstop:
            sleep(0.01)
        self.send_command(TR, GOAL_SPEED, 0)
        self.send_command(TL, GOAL_SPEED, 0)            
        self.send_command(TR, ZERO_ENCODER, 0)
        self.send_command(TL, ZERO_ENCODER, 0)  

    def change_twist_loop_length(self, size):
        per_motor_move = size/2
        self.send_command(TR, GOAL_POSITION, per_motor_move)
        self.send_command(TL, GOAL_POSITION, per_motor_move)
        if self.tr_control_flag != POSITION_CONTROL:
            self.send_command(TR, CONTROL_APPROACH, POSITION_CONTROL)
            self.tr_control_flag = POSITION_CONTROL
        if self.tl_control_flag != POSITION_CONTROL:
            self.send_command(TL, CONTROL_APPROACH, POSITION_CONTROL)
            self.t;_control_flag = POSITION_CONTROL
        if not self.tr_torque_flag:
            self.send_command(TR, TORQUE_ENABLE, 1)
            self.TR_torque_flag = 1
        if not self.tr_torque_flag:
            self.send_command(TL, TORQUE_ENABLE, 1)
            self.Tl_torque_flag = 1


    def home_loop(self, speed, threshold):
        homing_speed = -speed
        starting_force = self.force
        self.send_command(LP, GOAL_SPEED, -homing_speed)
        if self.lp_control_flag != NO_CONTROL:
            self.send_command(LP, CONTROL_APPROACH, NO_CONTROL)
            self.lp_control_flag = NO_CONTROL
        if not self.lp_torque_flag:
            self.send_command(LP, TORQUE_ENABLE, 1)
            self.lp_torque_flag = 1
        while self.lp_runout:
            sleep(0.01)  
        starting_force = self.force
        self.send_command(LP, GOAL_SPEED, homing_speed)
        while self.force <= (starting_force + threshold):
            sleep(0.01)
        self.send_command(LP, GOAL_SPEED, 0)
        self.send_command(LP, ZERO_ENCODER, 0)
        self.send_command(LP, GOAL_POSITION, 1000)
        self.send_command(LP, CONTROL_APPROACH, POSITION_CONTROL)
        sleep(1)
        self.send_command(LP, ZERO_ENCODER, 0)


