import serial
import time

class relay_controls():

    def __init__(self):
        self.ser = None

    def open_ports(self,port,baudRate,time_out):
        self.ser = serial.Serial(port, baudRate,timeout=time_out)

    def close_ports(self):
        self.ser.close
        
    def get_sw_version(self):
        num = 90
        self.ser.write(num.to_bytes(1,'big'))     # write a string
        time.sleep(1)
        version = self.ser.read(size=2)
        print(version)

    def set_relay_state(self,relay_num:int):
        relay_num.to_bytes(1,'big')
        self.ser.write(bytes([0x5C, relay_num]))
    
           # write a string
        
    def set_all_relays(self,state):
        num=0
        if state==True:
            num=100
            self.ser.write(num.to_bytes(1,'big'))

        elif state==False:
            num=110
            self.ser.write(num.to_bytes(1,'big'))
    
    def get_state(self,relay_num:int):
        relay_num.to_bytes(1,'big')
        self.ser.write(bytes([0x5B,relay_num]))
        current_state=self.ser.read(size=1)
        print(current_state)


def main():
    loc = relay_controls()
    loc.open_ports('/dev/ttyACM1', 9600,1)
    loc.set_all_relays(False)
    time.sleep(2)
    #loc.set_all_relays(True)
    loc.get_sw_version()
    time.sleep(3)
    #loc.set_relay_state()
    loc.get_state(111)
    loc.close_ports()


if __name__ == '__main__':
    main()
 