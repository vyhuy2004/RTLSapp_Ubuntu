import os
import tkinter as tk
import datetime
import cv2
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
from PIL import Image, ImageTk

class GUI:
    def __init__(self, output_path, camera, tracker, video_getter):
        self.output_path = output_path    
        self.current_image = None
        
        # Keep references to other class objects
        self.camera = camera
        self.tracker = tracker
        self.video_getter = video_getter
        
        # Create Main Window
        self.root = tk.Tk()  
        self.root.title("RTLS & Camera Integrated System")  
        self.root.protocol('WM_DELETE_WINDOW', self.destructor)
        
        # Create Main Blocks
        self.control_frame = tk.Label(self.root, padx=5, pady=5)
        self.plot_frame = tk.LabelFrame(self.root, text="Grid Map", padx=5, pady=5, labelanchor='n')
        self.video_frame = tk.LabelFrame(self.root, text="Camera", padx=5, pady=5, labelanchor='n')
        
        self.control_frame.grid(row = 0, column = 0, columnspan=2, sticky = 'WE')
        self.plot_frame.grid(row = 1, column = 0, sticky = 'NSEW')
        self.video_frame.grid(row = 1, column = 1, sticky = 'NSEW')

        self.add_control_content(self.control_frame)
        self.add_plot_content(self.plot_frame)
        self.add_video_content(self.video_frame)


    def add_control_content(self, master):
        tk.Grid.rowconfigure(master, 0, weight=1)
        tk.Grid.columnconfigure(master, 0, weight=1)
        tk.Grid.columnconfigure(master, 1, weight=1)

        b1 = tk.Button(master, text = " Run ", command=self.tracker.run) 
        b2 = tk.Button(master, text = "Stop", command=self.tracker.stop) 

        b1.grid(row = 0, column = 0, sticky = 'NSEW') 
        b2.grid(row = 0, column = 1, sticky = 'NSEW') 


    def add_plot_content(self, master):
        self.plot_fig, self.plot_ax = plt.subplots()
        self.plot_bar = FigureCanvasTkAgg(self.plot_fig, master)
        self.plot_bar.draw()
        self.plot_bar.get_tk_widget().grid(row = 0, column = 0, sticky = 'WE')
        self.plot_ani = animation.FuncAnimation(self.plot_fig, self.tracker._update_plot, interval=250, 
                                     init_func= lambda: self.tracker._setup_plot(self.plot_ax), blit=True)

    def add_video_content(self, master):
        self.img_up = tk.PhotoImage(file="GUI_resources/up.gif", master=master)
        self.img_down = tk.PhotoImage(file="GUI_resources/down.gif", master=master)
        self.img_left = tk.PhotoImage(file="GUI_resources/left.gif", master=master)
        self.img_right = tk.PhotoImage(file="GUI_resources/right.gif", master=master)

        self.vid_panel = tk.Label(master)  # initialize image panel

        self.vid_panel.grid(row = 0, column = 0, columnspan=3, sticky = 'nsew')
        self.vid_btn_up = tk.Button(master, image=self.img_up)
        self.vid_btn_up.bind("<Button-1>", lambda event: self.camera.setCMD('up'))
        self.vid_btn_up.bind("<ButtonRelease-1>", lambda event: self.camera.resetCMD())

        self.vid_btn_down = tk.Button(master, image=self.img_down)
        self.vid_btn_down.bind("<Button-1>", lambda event: self.camera.setCMD('down'))
        self.vid_btn_down.bind("<ButtonRelease-1>", lambda event: self.camera.resetCMD())

        self.vid_btn_left = tk.Button(master, image=self.img_left)
        self.vid_btn_left.bind("<Button-1>", lambda event: self.camera.setCMD('left'))
        self.vid_btn_left.bind("<ButtonRelease-1>", lambda event: self.camera.resetCMD())
        
        self.vid_btn_right = tk.Button(master, image=self.img_right)
        self.vid_btn_right.bind("<Button-1>", lambda event: self.camera.setCMD('right'))
        self.vid_btn_right.bind("<ButtonRelease-1>", lambda event: self.camera.resetCMD())
        
        self.calib_str = tk.StringVar()
        self.calib_str.set("CALIBRATE: OFF")
        self.calib_bol = False 
        self.vid_btn_calib = tk.Button(master, textvariable=self.calib_str, bg="#b8dcff", command=self.toggle_calib)
        self.vid_btn_capture = tk.Button(master, text = " CAPTURE ", bg="#b8dcff", command=self.video_capture) 
        self.vid_btn_follow = tk.Button(master, text = "FIND TAG: ON", bg="#b8dcff") 

        # arranging button widgets 
        self.vid_btn_up.grid(row = 1, column = 1, sticky = 'nsew') 
        self.vid_btn_down.grid(row = 2, column = 1, sticky = 'nsew')  
        self.vid_btn_left.grid(row = 1, column = 0, rowspan=2, sticky = 'nsew')
        self.vid_btn_right.grid(row = 1, column = 2, rowspan=2, sticky = 'nsew') 
        self.vid_btn_calib.grid(row = 3, column = 0, sticky = 'nsew')
        self.vid_btn_capture.grid(row = 3, column = 2, sticky = 'nsew')
        self.vid_btn_follow.grid(row = 3, column = 1, sticky = 'nsew')       
        self.video_update()
        
    def video_update(self):
        frame = self.video_getter.frame
        if self.camera.pid_active.value == True:
            frame = self.camera.draw_refpoint_estimation(frame)
        if not self.tracker.tags_latest.empty:
            pts = self.tracker.tags_latest[['x', 'y', 'z']].to_numpy().astype(float)
            #pts[:, 2] = 0.54
            pts = np.where(pts<0, 0, pts) 
            pixpts = self.camera.world_to_screen(pts)
            frame = self.camera.draw_points(frame, pixpts)
        frame = cv2.circle(frame, self.camera.center, 2, (0, 255, 0), -1)
        cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)  # convert colors from BGR to RGBA
        self.current_image = Image.fromarray(cv2image)  # convert image for PIL
        imgtk = ImageTk.PhotoImage(image=self.current_image)  # convert image for tkinter
        self.vid_panel.imgtk = imgtk  # anchor imgtk so it does not be deleted by garbage-collector
        self.vid_panel.config(image=imgtk)  # show the image
        self.after_id = self.root.after(50, self.video_update)  # call the same function after 30 milliseconds

    def video_capture(self):
        """ Take snapshot and save it to the file """
        ts = datetime.datetime.now() # grab the current timestamp
        filename = "{}.jpg".format(ts.strftime("%Y-%m-%d_%H-%M-%S"))  # construct filename
        p = os.path.join(self.output_path, filename)  # construct output path
        self.current_image.save(p, "JPEG")  # save image as jpeg file
        print("[APP] saved {}".format(filename))
        
    def toggle_calib(self):
        if self.calib_bol == False:
            self.calib_str.set("CALIBRATE: ON")
            self.calib_bol = True
            self.camera.enablePID()
        else:
            self.calib_str.set("CALIBRATE: OFF")
            self.calib_bol = False
            self.camera.disablePID()

    def destructor(self):
        """ Destroy the root object and release all resources """
        # Todo: Send SIGINT signal to other processes
        print("[APP] closing...")
        self.camera.stop_thread = True
        self.video_getter.stop()
        self.video_getter.stream.release()
        self.root.after_cancel(self.after_id)
        self.root.destroy()
        cv2.destroyAllWindows()  # it is not mandatory in this application

