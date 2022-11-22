from LoopO_driver import *  
from time import sleep
import threading

com_port = '/dev/ttyACM0'
baud = 115200

loopo = loopo_driver(com_port, baud)

def backgroung():
    while True:
        loopo.update()
        time.sleep(0.1)

def foreground():
    while True:
        print("Input acutaotr mesage")
        id = int(input("id: "))
        command = int(input("command: "))
        value = float(input("value: "))

        if loopo.send_command(id=id,command=command,value=value):
            print("error sending command")
        else:
            print("command sent successfully")

b = threading.Thread(name='background', target = backgroung)
f = threading.Thread(name='forground', target=foreground)

if __name__ == "__main__":
    
    b.start()
    f.start()