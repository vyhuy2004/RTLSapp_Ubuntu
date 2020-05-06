import threading
import numpy as np
import pandas as pd
import serial
import socket
import time
import collections
import os
import random
import matplotlib.pyplot as plt
from queue import Queue 

registered_tags = { '0397': 'Manager', '5385': 'Employee' }
class LocationTracker():
    def __init__(self, remote=False, address=None, debug=False, use_mockdata=False):
        self.names = registered_tags
        self.debug = False
        self.stop_thread = False
        self.remote = remote
        self.mock = use_mockdata
        # tracking components
        try:
            system = os.uname()[1]
        except:
            system = 'unknown'

        if (system == 'raspberrypi'):
            # initialize serial communication
            self.port_dir = '/dev/ttyACM0'
            self.ser = None
            if remote: # In Remote Mode, Raspberry Pi is the sender
                self.thread_name = self.DWM_Thread_RemoteSender
                self.address = address
                print('tracker will run in RemoteSender mode')
            else:
                self.thread_name = self.DWM_Thread_Local
                self.anchors = pd.read_csv(r'TrackerModule/anchors.csv')
                self.tags_log = pd.DataFrame(columns = ['id', 'x', 'y', 'z', 'time']) 
                self.tags_latest = pd.DataFrame(columns = ['id', 'x', 'y', 'z', 'time'])
                print('tracker will run in Local mode')
        else:
            self.anchors = pd.read_csv(r'TrackerModule/anchors.csv')
            self.x_max, self.y_max, self.z_max = self.anchors['x'].max(), self.anchors['y'].max(), self.anchors['z'].max()
            self.tags_log = pd.DataFrame(columns = ['id', 'x', 'y', 'z', 'time']) 
            self.tags_latest = pd.DataFrame(columns = ['id', 'x', 'y', 'z', 'time'])
            self.tags_info = {}
            self.tags_color = []
            if use_mockdata:
                self.thread_name = self.DWM_Thread_RemoteReceiver_Mock
            else:
                self.address = address
                self.socket = socket.socket()
                self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.socket.bind(self.address)       
                self.socket.listen()
                self.thread_name = self.DWM_Thread_RemoteReceiver

            print('tracker will run in RemoteReceiver mode')


    def run(self):      
        # Create DWM thread
        self.thread = threading.Thread(target=self.thread_name)    
        # Run thread
        self.thread.start()
            

    def stop(self):
        # close thread
        self.stop_thread = True
        self.thread.join()
        print('dwm thread terminated')
        # reset stop_thread flag
        self.stop_thread = False
        

    def DWM_Thread_Local(self):
        # Open serial port
        self.ser = serial.Serial(self.port_dir, 115200, timeout=1.0)
        # Enter UART shell of DWM module, and read the initial sequence 
        self.ser.write(b'\r\r')
        lines = self.ser.readlines()
        for line in lines:
            print(line.decode('ascii'))
        print('end init')
        time.sleep(0.1)         
        # Send 'lec' command to start collecting tags'locations
        self.ser.write(b'lec\r')
        time.sleep(0.1)
        self.start_time = time.time()
        while True:
            data = self.ser.read_until().decode('ascii').split(',')
            if (data[0] == 'POS'):
                tagid, x, y, z = data[2:6]
                time_now = time.time()
                self.tags_log = self.tags_log.append({'id':tagid, 'x':float(x), 'y':float(y), 'z':float(z), 'time': time_now}, ignore_index=True)
                if tagid in self.tags_latest['id'].unique():
                    idx = self.tags_latest.index[self.tags_latest['id']==tagid]
                    self.tags_latest.iloc[idx[0], 1:] = np.array([x, y, z, time_now], dtype=np.float)
                else:
                    self.tags_latest = self.tags_latest.append({'id':tagid, 'x':x, 'y':y, 'z':z, 'time': time_now}, ignore_index=True)
                    # self.color_code[tagid] = np.random.rand(3,)
                print(f'Tag: {tagid} | x:{x}, y:{y}, z:{z}')
            if self.stop_thread:
                break
            time.sleep(0.1)
            
        # close serial port
        self.ser.write(b'lec\r')
        time.sleep(0.1)
        self.ser.write(b'quit\r')
        time.sleep(0.1)
        self.ser.close()
        # save tags history
        self.stop_time = time.time()
        start_str = time.strftime('%m-%d-%y_%H:%M:%S', time.localtime(self.start_time))
        stop_str = time.strftime('%m-%d-%y_%H:%M:%S', time.localtime(self.stop_time))
        self.tags_log.to_csv(f'{start_str} to {stop_str}.csv', index=None)
            
            
    def DWM_Thread_RemoteSender(self):
        self.socket = socket.socket()
        self.socket.connect(self.address)
        # Open serial port
        self.ser = serial.Serial(self.port_dir, 115200, timeout=1.0)
        # Enter UART shell of DWM module, and read the initial sequence 
        self.ser.write(b'\r\r')
        lines = self.ser.readlines()
        for line in lines:
            print(line.decode('ascii'))
        print('end init')
        time.sleep(0.1)         
        # Send 'lec' command to start collecting tags'locations
        self.ser.write(b'lec\r')
        time.sleep(0.1)
        while True:
            data = self.ser.read_until().decode('ascii').split(',')
            if (data[0] == 'POS'):
                tagid, x, y, z = data[2:6]
                message = f'{tagid},{x},{y},{z}'
                print(message)
                self.socket.send(message.encode())
                response = self.socket.recv(1024).decode()
                if response == 'done':
                    self.socket.close()
                    self.ser.write(b'lec\r')
                    time.sleep(0.1)
                    self.ser.write(b'quit\r')
                    time.sleep(0.1)
                    self.ser.close()
                    self.stop_thread = True                   
                    break                    
            time.sleep(0.1)
        
    def DWM_Thread_RemoteReceiver(self):
        self.conn, self.addr = self.socket.accept()
        print(f'Got connection from {self.addr}')
        while True:
            data = self.conn.recv(1024).decode()
            if data == '':
                self.conn.close()
                break
            if self.stop_thread:
                self.conn.send(b'done')
            else:
                self.conn.send(b'ok')
            tagid, x, y, z = data.split(',')
            time_now = time.time()
            self.tags_log = self.tags_log.append({'id':tagid, 'x':float(x), 'y':float(y), 'z':float(z), 'time': time_now}, ignore_index=True)
            if tagid in self.tags_latest['id'].unique():
                idx = self.tags_latest.index[self.tags_latest['id']==tagid]
                self.tags_latest.iloc[idx[0], 1:] = np.array([x, y, z, time_now], dtype=np.float)
            else:
                self.tags_latest = self.tags_latest.append({'id':tagid, 'x':x, 'y':y, 'z':z, 'time': time_now}, ignore_index=True)
                self.tags_color.append(np.random.random())
                # self.color_code[tagid] = np.random.rand(3,)
            print(f'Tag: {tagid} | x:{x}, y:{y}, z:{z}')
                

    def DWM_Thread_RemoteReceiver_Mock(self):
        alphas = 'abcdefgh'
        print(f'RemoteReceiver: Using Mock Data')
        self.mock_index = 0
        self.mock_step = 0.01
        self.mock_tags = 2 
        for i in range(self.mock_tags):
            row = {'id': hex(np.random.randint(0xFFF, 0x10000)[2:]),
                   'x' : np.random.uniform(0, self.x_max),
                   'y' : np.random.uniform(0, self.y_max),
                   'z' : np.random.uniform(0, self.z_max),
                   'time': time.time() }
            self.tags_latest = self.tags_latest.append(row, ignore_index=True)
            self.tags_color.append(np.random.random())
        while True:
            if self.stop_thread:
                break
            tagid, x, y, z, _ = self.tags_latest.iloc[self.mock_index]
            x_new, y_new, z_new = (x + self.mock_step) if x < self.x_max else 0, (y + self.mock_step) if y < self.y_max else 0, z 
            time_now = time.time()
            self.tags_log = self.tags_log.append({'id':tagid, 'x':float(x_new), 'y':float(y_new), 'z':float(z_new), 'time': time_now}, ignore_index=True)
            self.tags_latest.iloc[self.mock_index, 1:] = np.array([x_new, y_new, z_new, time_now], dtype=np.float)
            print(f'Tag: {tagid} | x:{x_new:.3f}, y:{y_new:.3f}, z:{z_new:.3f}')
            self.mock_index = (self.mock_index + 1) %  self.mock_tags
            time.sleep(0.05)


    def _setup_plot(self, ax):
        self.ax = ax
        self.ax.grid(b=True, which='major', color='#999999', linestyle='-', alpha=0.2)
        self.ax.set_axisbelow(True)

        anchors_x = self.anchors['x'].to_numpy()
        anchors_y = self.anchors['y'].to_numpy()
        tags_x = self.tags_latest['x'].to_numpy()
        tags_y = self.tags_latest['y'].to_numpy()
        tags_id = self.tags_latest['id']
        # tags_color = np.array(list(self.color_code.values()), dtype=np.float)
        self.anchors_pts = self.ax.scatter(anchors_x, anchors_y, s=80, color='r')
        self.tags_pts = self.ax.scatter(tags_x, tags_y, s=80, color='g')
        self.annotations = []
        for idx, tag_id in enumerate(tags_id):
            self.annotations.append(self.ax.annotate(f'{tag_id}({float(tags_x[idx]):.2f},{float(tags_y[idx]):.2f})', 
                                    (float(tags_x[idx]), float(tags_y[idx]) + 0.08), ha='center')) 
        return self.tags_pts, self.anchors_pts, (*self.annotations)
    

    def _update_plot(self, i):
        """Update the scatter plot."""
        colors = np.array(self.tags_color)
        tags_x = self.tags_latest['x'].to_numpy()
        tags_y = self.tags_latest['y'].to_numpy()
        tags_id = self.tags_latest['id']
        self.tags_pts.set_offsets(self.tags_latest[['x', 'y']].to_numpy())
        tags_num = len(tags_id)
        ann_size = len(self.annotations)
        for idx, tag_id in enumerate(tags_id):
            if idx < ann_size:
                self.annotations[idx].set_text(f'{tag_id}({float(tags_x[idx]):.2f},{float(tags_y[idx]):.2f})')
                self.annotations[idx].set_position((float(tags_x[idx]), float(tags_y[idx]) + 0.08))
            else:
                self.annotations.append(self.ax.annotate(f'{tag_id}({float(tags_x[idx]):.2f},{float(tags_y[idx]):.2f})', 
                                    (float(tags_x[idx]), float(tags_y[idx]) + 0.08), ha='center')) 
        if tags_num > 0:
            self.tags_pts.set_array(np.array(self.tags_color, dtype=float))

        return self.tags_pts, self.anchors_pts, (*self.annotations)