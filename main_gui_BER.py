#-----------------------------
# Import Modules
#----------------------------- 
import tkinter as tk
import matplotlib
import random
import math
import sys
matplotlib.use("TkAgg")
from matplotlib import pyplot as plt
from matplotlib.ticker import MaxNLocator
import csv
import re
import os
import numpy as np
from collections import namedtuple
from PIL import ImageTk,Image
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

#-----------------------------
# Bird's Eye Radar
#-----------------------------

#-----------------------------                                                                                                                                                                                                                    
# Create main interface window  
#-----------------------------                                                                                                                                                                                                                   
class MainInterfaceBEF:
    def __init__(self, master):
        self.master = master
        self.canvas = tk.Canvas(self.master, bg="blue", height=250, width=300)
        self.image = Image.open("/Users/Nida/Desktop/background_ali_proj.jpg") 
        self.background_image=ImageTk.PhotoImage(self.image)
        self.background_label = tk.Label(self.canvas, image=self.background_image, anchor= tk.NW)
        self.background_label.image = self.background_image
        self.background_label.pack()
        self.header = tk.Label(self.canvas, text="Welcome to Bird's Eye Radar", anchor=tk.CENTER, font=('Comic Sans MS', '40', 'bold'), fg="Orange", bg="Sky blue")
        self.header.place(x=100, y=25)
        self.startbutton = tk.Button(self.canvas, text = "Start", command = self.new_window, highlightbackground="Red", fg="Black", font=('Comic Sans MS', '25', 'bold'), relief=tk.RAISED )
        self.startbutton.place(height = 100, width =100, x= 375, y = 250)
        self.quitbutton = tk.Button(self.canvas, text = "Quit", command = self.close_windows, highlightbackground="Orange", fg="Black", font=('Comic Sans MS', '25', 'bold'))
        self.quitbutton.place(x=385,y=420)
        self.canvas.pack()
    
    def close_windows(self):
        self.master.destroy()
    def new_window(self):
        self.master.withdraw()
        self.newWindow = tk.Toplevel(self.master)
        self.app = InputWindowBEF(self.newWindow)

#-----------------------------                                                                                                                                                                                                                    
# Create file input  window                                                                                                                                                                                                                   
#-----------------------------                                                                                                                                                                                                                   
class InputWindowBEF:
    def __init__(self, master):
        self.file_path = ''
        self.master = master                                                                                                                                                                                                           
        self.canvas = tk.Canvas(self.master, bg="blue", height=250, width=300)
        self.image = Image.open("/Users/Nida/Desktop/background_ali_proj.jpg")
        self.background_image=ImageTk.PhotoImage(self.image)
        self.background_label = tk.Label(self.canvas, image=self.background_image, anchor= tk.NW)
        self.background_label.image = self.background_image
        self.background_label.pack()
        self.header = tk.Label(self.canvas, text="Enter full path to the image.", anchor=tk.CENTER, font=('Comic Sans MS', '35', 'bold'), fg="Black", bg="Sky blue")
        self.header.place(x=100, y=40)
        self.enterbutton = tk.Button(self.canvas, text = "Enter", command = self.getfilepath, highlightbackground="Green", fg="Black", font=('Comic Sans MS', '20', 'bold'), relief=tk.RAISED )
        self.enterbutton.place(height = 50, width =100, x=700, y=250 )
        self.filepath_entry = tk.Entry(self.canvas, textvariable=self.file_path)
        self.filepath_entry.place(height = 50, width = 600, x = 75, y= 250)
        self.quitbutton = tk.Button(self.canvas, text = "Quit", command = self.close_windows, highlightbackground="Orange", fg="Black", font=('Comic Sans MS', '25', 'bold'))
        self.quitbutton.place(height = 50, width =100, x= 275, y = 400)
        self.resetbutton = tk.Button(self.canvas, text = "Clear", command = self.reset_window, highlightbackground="Orange", fg="Black", font=('Comic Sans MS', '25', 'bold'))
        self.resetbutton.place(height = 50, width =100, x= 400, y = 400)                                                                                                                                                                     
        self.canvas.pack()
    def reset_window(self):
        if (self.error.winfo_exists() == 1):
            self.error.destroy()
        self.filepath_entry.delete(first=0, last=100)
        self.new_image= ''        
        self.file_path = ''

    def close_windows(self):
        self.master.destroy()
     
    def new_window_results(self, new_image):
        self.master.withdraw()
        self.newWindow = tk.Toplevel(self.master)
        self.app = ResultWindowBEF(self.newWindow, new_image)


    def getfilepath(self):
        ## Add Exceptions here if file name is Invalid. 
        if os.path.isfile(self.filepath_entry.get()):
            try: 
                open(self.filepath_entry.get())
                self.new_image = Image.open(self.filepath_entry.get()) 
                self.new_window_results(self.new_image)
            except IOError as e:
                self.error_message = "Unable to open file"
                self.error = tk.Label(self.canvas, text = self.error_message)
                self.error.place(height = 50, width = 100 , x= 75, y = 350)
        else :
            self.error_message = "This is not a valid path name.Try again."
            self.error = tk.Label(self.canvas, text = self.error_message)
            self.error.place(x= 0, y = 350)
        return
    

#-----------------------------                                                                                                                                                                                                                       
# Create main interface window                                                                                                                                                                                                                   
#-----------------------------                                                                                                                                                                                                                         
class ResultWindowBEF:
    def __init__(self, master, new_image):
        self.master = master
        self.canvas = tk.Canvas(self.master, bg="maroon", height=800, width=800)
        self.new_image = new_image.resize((350,350))                                                                                                                                                     
        self.test_image=ImageTk.PhotoImage(self.new_image)                                                                                                                                                                                 
        self.test_label = tk.Label(self.canvas, image=self.test_image)                                                                                                                                                   
        self.test_label.image = self.test_image                                                                                                                                                                                              
        self.test_label.pack(side = tk.LEFT)
        self.header = tk.Label(self.canvas, text="Tensorflow Results", anchor=tk.CENTER, font=('Comic Sans MS', '35', 'bold'), fg="White", bg="Maroon")
        self.header.place(x=0, y=0)  
        self.quitbutton = tk.Button(self.canvas, text = "Quit", command = self.close_windows, highlightbackground="Green", fg="Black", font=('Comic Sans MS', '18', 'bold'), relief=tk.RAISED)
        self.quitbutton.place( x=75, y = 450)
        self.resetbutton = tk.Button(self.canvas, text = "Reset", command = self.reset_window, highlightbackground="Green", fg="Black", font=('Comic Sans MS', '18', 'bold'), relief=tk.RAISED)
        self.resetbutton.place( x=175, y=450)
        self.canvas.pack()
        self.parse_and_display_data()
        self.gps_coordinates()
        

    def reset_window(self):
        self.master.withdraw()
        self.newWindow = tk.Toplevel(self.master)
        self.app = InputWindowBEF(self.newWindow)

    def close_windows(self):
        self.master.destroy()

    def gps_coordinates(self):
        radius = 10000
        radiusInDegrees = radius / 111300
        r = radiusInDegrees
        x0 = 30.64
        y0 = -96.29
        
        for i in range(1,2):
            u = float(random.uniform(0.0, 1.0))
            v = float(random.uniform(0.0,1.0))

            w = r * math.sqrt(u)
            t = 2 * math.pi * v
            x = w * math.cos(t)
            y = w * math.sin(t)

            xLat = x + x0
            yLong = y + y0
            
            text_var = tk.StringVar()         
            text_var.set('GPS Coordinates: (' + str(xLat) + ',' + str(yLong) + ')') 
            text_box = tk.Entry(self.canvas, textvariable=text_var,width=57, bg='maroon', fg='white',font=('Comic Sans MS', '10', 'bold'))
            text_box.place(x='0', y='430')
        
        
    #-----------------------------                                                  
    # Parse the csv file and extract 
    # data to plot  
    #-----------------------------
    def parse_and_display_data(self):
        row_count = 0
        all_data = []
        tick_labels = []
        data_per_label = []

        with open('/Users/Nida/Desktop/results.csv', 'r') as csvfile:
            for row in csvfile:
                row_count += 1
                all_data.append(row)
            print (all_data)
            print(row_count)
            for line in all_data:
                new_line = line.split("(")[0]
                tick_labels.append(new_line)
            print(tick_labels)
            for line in all_data:
                new_line = re.search("=.*?\)", line).group(0)
                new_line = new_line.split("=")[1]
                new_line = new_line.split(")")[0]
                new_line = new_line.strip()
                new_line = float(new_line) * 100
                data_per_label.append(new_line)
            print (data_per_label)
    
        # Use Matplotlib to display
        num_group = len(tick_labels)
        tick_labels_tuple = tuple(tick_labels)
        data_per_label_tuple = tuple (data_per_label)
        result_dict = dict(zip(tick_labels_tuple,data_per_label_tuple))
        max_key = max(result_dict, key=result_dict.get)
        max_value = round(max(result_dict.values()), 2)
        print (data_per_label_tuple)
        print (result_dict)
        fig,a = plt.subplots()
        index = np.arange(num_group)
        bar_width = 0.60
        opacity = 0.65
        rects1 = a.bar(index, data_per_label_tuple, bar_width,alpha =opacity, color='b') 
        a.set_xlabel('Object')
        a.set_ylabel('Percentage Match (%)')
        a.set_title('Tensorflow Results')
        a.set_xticks(index+bar_width/2)
        a.set_xticklabels(tick_labels_tuple, rotation='vertical')
        fig.subplots_adjust(bottom=0.3)
        bar1 = FigureCanvasTkAgg(fig, self.canvas)
        bar1.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH)
        text_var = tk.StringVar()         
        text_var.set('Image is '+ str(max_value) + '% likely to be '+ max_key) 
        text_box = tk.Entry(self.canvas, textvariable=text_var,width=38, bg='maroon', fg='white',font=('Comic Sans MS', '14', 'bold'))
        text_box.place(x='0', y='403')
        
#-----------------------------                                                                                                                                                                                                                   
 # Start window  
#-----------------------------
def start_window():
    root = tk.Tk()
    root.geometry("800x500")
    terminal = MainInterfaceBEF(root)
    root.mainloop()
 
#-----------------------------                                                                                                                                                                                                                   
# Main loop 
#-----------------------------
if __name__=='__main__':
    start_window()


