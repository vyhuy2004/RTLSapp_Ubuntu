import threading
import numpy as np
import pandas as pd
import serial
import time
import collections
import matplotlib.pyplot as plt

class LocationTracker():
    def __init__(self):
        self.debug = False
        self.stop_thread = False

        # tracking components 
        self.anchors = pd.read_csv(r'TrackerModule/anchors.csv')
        self.tags_log = pd.DataFrame(columns = ['id', 'x', 'y', 'z', 'time']) 
        self.tags_latest = pd.DataFrame(columns = ['id', 'x', 'y', 'z', 'time'])
        # self.color_code = collections.OrderedDict()
        self.available = False
        # initialize serial communication
        self.port_dir = '/dev/ttyACM0'
        self.ser = None
        self.start_time = 0
        self.stop_time = 0

    def run(self, duration=None, show_plot=False):
        
        self.ser = serial.Serial(self.port_dir, 115200, timeout=1.0)
        # Enter UART shell, and read the initial sequence 
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
        self.thread = threading.Thread(target=self.DWM_Thread)
        self.thread.start()

        if not (duration is None):
            while True:
                if (time.time()-self.start_time > duration):
                    break
            self.stop()
            

    def stop(self):
        # close thread
        self.stop_thread = True
        self.thread.join()
        print('dwm thread terminated')

        # reset stop_thread flag
        self.stop_thread = False

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


    def DWM_Thread(self):
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
            

    def _setup_plot(self, ax):
        ax.grid(b=True, which='major', color='#999999', linestyle='-', alpha=0.2)
        ax.set_axisbelow(True)

        anchors_x = self.anchors['x'].to_numpy()
        anchors_y = self.anchors['y'].to_numpy()
        tags_x = self.tags_latest['x'].to_numpy()
        tags_y = self.tags_latest['y'].to_numpy()
        tags_id = self.tags_latest['id']
        # tags_color = np.array(list(self.color_code.values()), dtype=np.float)
        self.anchors_pts = ax.scatter(anchors_x, anchors_y, s=80, color='r')
        self.tags_pts = ax.scatter(tags_x, tags_y, s=80, color='g')
        # for idx, tag_id in enumerate(tags_id):
        #     self.annotations.append(ax.annotate(f'(tag_id{tags_x[idx]}, {tags_y[idx]})', tuple(tags_x[idx], tags_y[idx]), 
        #                     textcoords="offset points", # how to position the text
        #                     xytext=(0,10), 
        #                     ha='center')) 
        return self.tags_pts, self.anchors_pts
    

    def _update_plot(self, i):
        """Update the scatter plot."""
        # tags_color = np.array(list(self.color_code.values()), dtype=np.float)
        self.tags_pts.set_offsets(self.tags_latest[['x', 'y']].to_numpy())
        # self.tags_pts.set_array(tags_color)

        return self.tags_pts, self.anchors_pts