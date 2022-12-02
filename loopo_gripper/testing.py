from LoopO_driver import *  
from time import sleep
import threading

com_port = '/dev/ttyACM0'
baud = 115200

loopo = loopo_driver(com_port, baud)

def backgroung():
    while True:
        loopo.update()
        sleep(0.001)

def foreground():
    #while True:
    #    print("Input acutator mesage")
    #    id = int(input("id: "))
    #    command = int(input("command: "))
    #    value = float(input("value: "))
    #   
    #    if loopo.send_command(id=id,command=command,value=value):
    #        print("error sending command")
    #    else:
    #        print("command sent successfully")
    #print(loopo.home_extension(100))
    sleep(1)
    ##loopo.send_command(1, 0, 1)
    ##loopo.send_command(1, 1, 1)
    ##loopo.send_command(1, 3, 10000)
    #loopo.home_extension(4)
    #loopo.move_extension(5000)
    #loopo.move_extension(1000)
    #loopo.home_twist(4)
    #loopo.change_twist_loop_length(3000)

b = threading.Thread(name='background', target = backgroung)
f = threading.Thread(name='forground', target=foreground)

if __name__ == "__main__":
    
    b.start()
    f.start()