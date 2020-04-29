#TODO: Add states handling in process_sock. command->takeoff>in_flight->land
#TODO: When in flight, no command in 5 seconds. send command to keep hovering.
#TODO: when error land, go into land
#TODo: command sent are added into queue.
#TODO: in_action state = false or true

import socket
import select
import types
import threading
import time
import queue
import libh264decoder
import numpy as np
import signal
import cv2
import sys


class Tello:
    def __init__(self,tello_ip,command_port=8889,video_port=11111,telem_port=8890,interface=None):
        self.tello_ip = tello_ip
        self.command_port = command_port
        self.video_port = video_port
        self.telem_port = telem_port
        self.interface = interface

        self.command_sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.video_sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.telem_sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

        if self.interface:
            self.command_sock.setsockopt(socket.SOL_SOCKET, 25,self.interface.encode())
            self.video_sock.setsockopt(socket.SOL_SOCKET, 25,self.interface.encode())
            self.telem_sock.setsockopt(socket.SOL_SOCKET, 25,self.interface.encode())

        self.command_sock.bind(('0.0.0.0', self.command_port))
        self.command_sock.setblocking(False)
        self.command_sock.settimeout(10)
        self.sock_process_thread = threading.Thread(target=self._process_socks)
        self.sock_process_thread.daemon = True

        self.telem_sock.bind(('0.0.0.0', int(self.telem_port)))
        self.telem_sock.setblocking(False)
        self.telem_sock.settimeout(10)

        self.video_sock.bind(('0.0.0.0', self.video_port))
        self.video_sock.setblocking(False)
        self.decoder_queue = queue.Queue(128)
        self.decoder = libh264decoder.H264Decoder()
        libh264decoder.disable_logging()
        self.videothread = threading.Thread(target=self._receive_video_data)
        self.videothread.daemon = True

        self.r_socks =[]
        self.w_socks =[]
        self.a_socks =[]

        #state variable
        self.in_command_mode = False
        self.in_action_mode = False
        self.in_flight = False
        self.error_and_stop_flight = False
        self.last_cmd = ''
        self.start_cmd_time = -1
        self.timeout = 9.0
        self.wait_time = 8.0 #wait duration for a response to an action, after which wait ends and next command is sent. TODO: to change wait_time based on command and past data 
        self.socket_closed = False
        self.connecting = False
        self.connect_attempt = 0
        self.max_connect_attempt = 3 #number of attempts to connect
        self.prevent_tello_inactivity_and_land_duration = 8 #duration after which hover command is sent to prevent auto land of tello due to inactivity

        self.last_frame = None                
        self.frame = None
        self.is_freeze = False
        self._cmdseq = 0

        self.response =''
        self.data_queue = {
            self.command_sock : queue.Queue(32), #for buffering data from drone
            self.video_sock : queue.Queue(32), #for buffering video data from drone
            self.telem_sock : queue.Queue(32), #for buffering telemetry data from drone
        }

        self.cmd_queue = queue.Queue() #for buffering instructions from users

        self.data_timeout = 3
        self.batterylevel = -1
        self.tof = -1

    def __del__(self):
        self.close()

    def close(self):
        self.send('land')
        self.socket_closed = True
        self.sock_process_thread.join()
        self.videothread.join()
        self.videothread.join()
        self.video_sock.close()
        self.telem_sock.close()
        self.command_sock.close()

    def connect(self):
        try:
            self.w_socks.append(self.command_sock)
            self.r_socks.append(self.command_sock)
            self.a_socks.append(self.command_sock)
            self.sock_process_thread.start()
            self.send('command')

            while self.in_command_mode == False and self.connect_attempt < self.max_connect_attempt:
                time.sleep(0.1)
            
            if self.in_command_mode:
                return True
            else:
                return False
        except socket.error as err:                    
            print("Drone %s entering command mode failed.(%s)" % (self.tello_ip,err))
            self.command_sock.close()
            return False
    
    def send(self, data):
        self.last_cmd = data.lower()
#        self._cmdseq = (self._cmdseq + 1) % 100 

        try:
            if data.lower() == 'command' and self.in_command_mode == False:
                print("Connecting to %s" % self.tello_ip)
                self.connecting = True
#                data = data.rstrip() + ' ' + str(self._cmdseq)
                self.cmd_queue.put(data.rstrip())
            else:
                if self.in_command_mode == False:
                    print('Drone %s unable to process as not in command mode' % self.tello_ip)
                else:
#                    data = data.rstrip() + ' ' + str(self._cmdseq)                    
                    self.cmd_queue.put(data.rstrip())
            return True
        except queue.Full as err:
            print("Drone %s (send) Queue Full- %s" % (self.tello_ip,err))
            return False


    def readframe(self):
        if self.in_command_mode:
            try:
                self.frame = self.decoder_queue.get(timeout=2)
            except queue.Empty:
                if self.socket_closed:
                    print("Drone %s - socket closed" % self.tello_ip)
                print("Drone %s rf- video queue empty" % self.tello_ip)
                return self.last_frame
            else:
                if self.is_freeze:
                    return self.last_frame
                else:
                    self.last_frame = self.frame
                    return self.frame
        else:
            print("Drone %s is not in command mode.")

    def video_freeze(self,is_freeze=True):
        self.is_freeze= is_freeze
        if is_freeze:
            self.last_frame = self.frame
                        
    def _process_going_into_command(self, result):    

        r = result

        if self.in_command_mode == False:
            if r.lower() == 'ok':
                try:
                    self.r_socks.append(self.video_sock)
                    self.a_socks.append(self.video_sock)
                    self.videothread.start()
                    print('Drone %s recieving video stream' % self.tello_ip)
                except socket.error as err:
                    self.video_sock.close()
                    print('Drone %s opening video stream failed' % self.tello_ip)

#TODO: move the below telemetry socket opening codes to the manager
                try:
                    self.r_socks.append(self.telem_sock)
                    self.a_socks.append(self.telem_sock)
                    print('Drone %s started telemetry receiver' % self.tello_ip)
                except socket.error as err:
                    self.telem_sock.close()
                    print("Drone %s error at binding to telemetry port - %s" % (self.tello_ip,err))
#TODO: ends here
                self.in_command_mode = True
                self.in_action_mode = False
                    
            else:
                if r.lower() == 'error':
                    print('Drone %s command mode failed' % self.tello_ip)
#                    self.socket_closed = True
                else:
                    print('Drone %s encountered unexpected data when going into command mode' % self.tello_ip)
        else:
            print('Drone %s is not going into command mode' % self.tello_ip)

# To-Do: to handle telemetry    
#    def _process_telem(self):
#        try:
#            while True:
#                data, address =self.telem_sock.recvfrom(4096)
#                d = data.decode('UTF-8')
#                print(d)
#                time.sleep(0.5)
#        finally:
#            print('Drone %s - telem_port error. closed' % self.tello_ip)
#            self.telem_sock.close()

    def _receive_video_data(self):
        print('Started Video Receiver for Drone %s' % self.tello_ip)
        self.video_packet_data = b''
        print('')
        while self.socket_closed == False and self.in_command_mode == True:
            try:
                data = self.data_queue[self.video_sock].get(timeout=None)
            except queue.Empty:
                print("Drone %s - video data stream queue no data" %(self.tello_ip))
            else:                
                self.video_packet_data += data            
                if len(data) != 1460:
                    for frame in self._h264_decode(self.video_packet_data):
                        try:
                            self.decoder_queue.put(frame,timeout=2)
#                            print('xx')
                        except queue.Full:
                            if self.socket_closed == True:
                                break
                            print("Drone %s - video queue full" % self.tello_ip)
                            continue
#                            self.frame = frame
                    self.video_packet_data = b''
                #except socket.error as err:
               # print('Drone %s - video error. (%s)' % (self.tello_ip, err))


    def _h264_decode(self, packet_data):
        res_frame_list = []
        frames = self.decoder.decode(packet_data)
        for framedata in frames:
            (frame, w, h, ls) = framedata
            if frame is not None:
#                print 'frame size %i bytes, w %i, h %i, linesize %i' % (len(frame), w, h, ls)
                frame = np.fromstring(frame, dtype=np.ubyte, count=len(frame), sep='')
                frame = (frame.reshape((h, int(ls / 3), 3)))
                frame = frame[:, :w, :]
                res_frame_list.append(frame)
        return res_frame_list

    def _process_command_response(self,result):
        if 'ok' in result:
            self.in_action_mode = False
            if self.connecting == True:
                self.connecting = False
                self.in_command_mode = True
            
            if 'takeoff' in self.last_cmd:
                self.in_flight = True
                print('Drone %s in flight.' % self.tello_ip)

            if 'land' in self.last_cmd:
                self.in_flight = False
                self.in_command_mode = False
                print('Drone %s exit flight.' % self.tello_ip)

            return 1
        else:
            print("process_command_ack : %s" % result)
            if 'error' in result:

                if 'takeoff' in self.last_cmd: #error response to takeoff means drone low battery and thus all action should stop.
                    self.in_flight = False
                    self.in_command_mode = False
                    self.in_action_mode = False
                    self.error_and_stop_flight = True
                    return 2

                if self.connecting == True or self.in_command_mode == False:
                    return 2

                if self.in_flight and ('motor' in result or 'auto land' in result or 'forced stop' in result):
                    self.in_flight = False
                    self.in_action_mode = False
                    self.in_command_mode = False
                    self.error_and_stop_flight = True
#                    else:
#                        self.command_sock.sendto('emergency'.encode('UTF-8'), (self.tello_ip, self.command_port)) # whenever error, stop immediately.
#                        self.in_flight = False
#                        self.in_action_mode = False

                 #to address error and land
                self.in_action_mode = False
                return 0
            else:
                print("Drone %s respond with %s for %s." % (self.tello_ip, result, self.last_cmd))
                return 1


    def _process_socks(self):
        data =[]
#        while True:
        while self.socket_closed == False and self.error_and_stop_flight == False:
            try:
                readable, writable, exceptional = select.select(self.r_socks,self.w_socks,self.a_socks)
            except Exception:
                readable = []
                writable = []
                exceptional = []

            for r in readable:
                if r is self.command_sock:
                    print("command port")
                    decode_error = False
                    try:
                        cdata,address = self.command_sock.recvfrom(4096)
                        d = cdata.decode('UTF-8')
                        print("received from %s- %s : %s" % (address,self.last_cmd,cdata))
                        d = d.lower()

                    except socket.error as err:
                        print('Drone %s - recevie data failed. (%s)' % (self.tello_ip, err))
                    except UnicodeDecodeError as err:
                        decode_error = True

                    if decode_error == False:
                        if self.in_command_mode == False and self.connecting == True:
                            self._process_going_into_command(d)
                        else:
                            if self.in_command_mode == True:
                                self._process_command_response(d)
                else:
                    if r is self.video_sock:
                        vdata,address = self.video_sock.recvfrom(4096)
                        if self.data_queue[self.video_sock].full():
                            self.data_queue[self.video_sock].get()
                        self.data_queue[self.video_sock].put(vdata)
#                        print(vdata)
                    else:
                        if r is self.telem_sock:

                            tdata,address = self.telem_sock.recvfrom(4096)
#                            print(tdata)
                            tdata = tdata.decode('UTF-8')
                            tel = tdata.split(';')
                            s,bat = tel[15].split(":")
                            self.batterylevel = int(bat)
                            s,tof = tel[13].split(":")
                            self.tof = int(tof)
#                            print(self.tof)
                            


                if self.error_and_stop_flight:
                    break
        
            for w in writable:
                try:
                    if self.cmd_queue.empty() == False and w is self.command_sock and self.error_and_stop_flight == False:
                        # sending command that is in the queue
                        if self.in_action_mode == False:
                            msg_to_send = self.cmd_queue.get_nowait()
                            self.start_cmd_time = time.time()
                            self.command_sock.sendto(msg_to_send.encode('UTF-8'), (self.tello_ip, self.command_port))
                            print('Drone %s send %s at %s ' % (self.tello_ip, msg_to_send, str(self.start_cmd_time)))
                            self.last_cmd = msg_to_send

                            if 'command' in self.last_cmd and self.connecting == False and self.in_command_mode == False:
                                # if command is sent and drone is not in command state.
                                self.connecting = True
                                self.connect_attempt = 1

                            self.in_action_mode = True
                            # need a function to calculate wait time
                        else:
                            if (time.time() - self.start_cmd_time) > self.wait_time and (self.in_action_mode == True or self.connecting == True):
                                # handling of no response beyond the wait time
                                print('Drone %s last command timeout (%s) at %s' % (self.tello_ip, str(self.start_cmd_time), str(time.time())))
                                if 'command' in self.last_cmd and self.connect_attempt < self.max_connect_attempt and self.connecting == True:
                                    # reconnecting when no respond from drone beyond wait time
                                    self.command_sock.sendto('command'.encode('UTF-8'), (self.tello_ip,self.command_port))
                                    self.start_cmd_time = time.time()
                                    self.connect_attempt = self.connect_attempt + 1
                                    print('Connecting to Drone %s timeout. Attempting %s time' % (self.tello_ip, str(self.connect_attempt)))
                                else:
                                    if self.in_command_mode and self.in_action_mode:
                                        # issue next command in queue since no respond from drone beyond wait time
                                        msg_to_send = self.cmd_queue.get_nowait()
                                        self.command_sock.sendto(msg_to_send.encode('UTF-8'),(self.tello_ip,self.command_port))
                                        print('Drone %s send %s' % (self.tello_ip, msg_to_send))
                                        self.last_cmd = msg_to_send
                                        self.start_cmd_time = time.time()
                                        print('Drone %s - Last command timeout. Proceed with next command %s' % (self.tello_ip, self.last_cmd))
                                        self.in_action_mode = True
                                

                    else:
                        if w is self.command_sock and self.error_and_stop_flight == False: # handle no command and keep hovering to prevent auto land
                            if (time.time() - self.start_cmd_time) > self.prevent_tello_inactivity_and_land_duration and self.in_action_mode == False and self.in_command_mode == True:
                                if self.in_flight == True:
                                    self.command_sock.sendto('stop'.encode('UTF-8'),(self.tello_ip,self.command_port))
                                    self.last_cmd = 'stop'
                                    self.start_cmd_time = time.time()
                                else:
                                    if self.in_command_mode:
                                        self.command_sock.sendto('command'.encode('UTF-8'),(self.tello_ip,self.command_port))
                                        self.last_cmd = 'command'
                                        self.start_cmd_time = time.time()
                            else:
                                if (time.time() - self.start_cmd_time) > self.wait_time and (self.in_action_mode == True or self.connecting == True):
                                     # reconnecting when no respond from drone beyond wait time
                                    self.command_sock.sendto('command'.encode('UTF-8'), (self.tello_ip,self.command_port))
                                    self.start_cmd_time = time.time()
                                    self.connect_attempt = self.connect_attempt + 1
                                    print('Connecting to Drone %s timeout. Attempting %s more time.' % (self.tello_ip, str(self.connect_attempt)))


                except socket.error as err:
                    print('Drone %s - send command (%s) failed. (%s)' % (self.tello_ip, msg_to_send, err))
                    self.error_and_stop_flight = True

                if self.error_and_stop_flight:
                    break


            for e in exceptional:
                print ("handling exceptional")
                if e is self.command_sock:
                    self.w_socks.remove(e)
                    self.r_socks.remove(e)
                    self.a_socks.remove(e)
                    self.error_and_stop_flight = True
                    e.close()
                    print("error in command port.")
                    
                if e is self.video_sock:
                    self.r_socks.remove(e)
                    self.a_socks.remove(e)
                    e.close()
                    print("error in video port")


if __name__ == '__main__':
    try:

        t = Tello('192.168.10.1')
        if t.connect(): # connect function returns true when connected. otherwise
            t.send('takeoff') #send function put tello command on queue and execute one by one after response or timeout of previous commands. refer tello 2.0 sdk for tello commands
            t.send('forward 50')
            t.send('mon')
            t.send('left 100')
            i = 0
            while i < 20:
                time.sleep(2)
                print('tof: %s' % str(t.tof)) 
                print('bat: %s' % str(t.batterylevel))
                i = i + 1
            t.send('land')
            i = 0
            while i < 20:
                time.sleep(1)
                i = i + 1
    except KeyboardInterrupt:
        sys.exit(0)
    
