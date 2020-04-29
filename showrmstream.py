import time
from TelloRMVideoClient import TelloRMVideoClient
from rm import RMS1
import threading
import socket
import os
import sys
import socket
from tello import Tello

def main(mode='host'):
    global e
    global t
    if mode == 'host':
        e = RMS1('192.168.2.1')
        t = Tello('192.168.10.1')
    else:
        robot_ip = robotlistener()
        if robot_ip == '':
            print('no robot connected to network')
        else:
            e = RMS1(robot_ip)

    if e is not None and t is not None: 
        if e.connect() and t.connect(): 
            v = TelloRMVideoClient(e,t)
            v.start()
            while v.stopApp == False:
                time.sleep(2)
        else:
            print('no RM online or no Tello online')
    else:
        print('no RM online or no Tello online')


def robotlistener():
    try:
        broad_sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        broad_address = ('0.0.0.0', 40926)
        broad_sock.bind(broad_address)

        found_robot_ip = ''
        i = 0
        print('waiting for broadcast')

        while found_robot_ip == '' and i < 10:
            data, address = broad_sock.recvfrom(4096)
            d = data.decode('UTF-8')
            print("Received broadcast from %s - %s " % (address, d))
            if 'robot ip' in d:
                s = d.split()
                if len(s) >= 3:
                    found_robot_ip = s[2]
                    try:
                        socket.inet_aton(found_robot_ip)
                        print('Found robot - %s' % found_robot_ip)
                    except socket.error:
                        found_robot_ip =''
                        print('invalid ip address.')
                else:
                    print('not robot ip broadcast')
            else:
                print('not robot ip broadcast')
                
            i = i + 1
            time.sleep(0.5)
            print('scan %s' % str(i))
        
        return found_robot_ip
    except socket.error as err:
        print("Unable to listen for robots - %s" % err)
        sys.exit(0)

if __name__ == "__main__":
    try:
        e = None
        t = None
        main('host')
    except KeyboardInterrupt:
        e.close()
        sys.exit(0)