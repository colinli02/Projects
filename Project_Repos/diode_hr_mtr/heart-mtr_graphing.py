import string


import time
import serial
import serial.tools.list_ports



import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys, time, math

xsize=900

# configure the serial port
try:
    ser = serial.Serial(
        port='COM4', # Change as needed
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_TWO,
        bytesize=serial.EIGHTBITS
    )
    ser.isOpen()
except:
    portlist=list(serial.tools.list_ports.comports())
    print ('Available serial ports:')
    for item in portlist:
       print (item[0])
    exit()

   
def data_gen():
    t = data_gen.t
    while True:
       #val = ''
       t+=1
       
       val = ser.readline() #reads the port as a byte
       #print(type(val))#currently is a byte

       #convert byte to float
       #convert byte into string first
       val = val.decode('utf-8') 
       #print(type(val)) #currently is a string
       
       #removes all letters except beginning 
       #removes all letters
       val = val.replace('V0=', '')
       val = val.replace('V1=', '')
       val = val.replace('V2=', '')
       val = val.replace('V3=', '')
       val = val.replace('V4=', '')
       val = val.replace('V5=', '')
       val = val.replace('V6=', '')
       val = val.replace('V7=', '')
       val = val.replace('V', '')
       val = val.replace('=', '')
       #removes random left over characters that block float conversion 
       val = val.replace('=', '')
       val = val.replace(' ', '')
       val = val.replace(' '' ', '')
       
       val = val[0:5] #keeps first 4 digits of string

       #then converts string to float
       #print(type(val)) #currently is a string
       #print(len(val)) #for debugging
       val = float(val)
       val = val/45
       #val = val*2

       #appends t
       #time = "*t"
       #listOfStrings = [val,time]
       #val = "".join(listOfStrings)
       
       #print(type(val))
       #val=100.0*math.sin(t*2.0*3.1415/100.0) #this is a float
       print(val)
       yield t, val

def run(data):
    # update the data
    t,y = data
    if t>-1:
        xdata.append(t)
        ydata.append(y)
        if t>xsize: # Scroll to the left.
            ax.set_xlim(t-xsize, t)
        line.set_data(xdata, ydata)

    return line,

def on_close_figure(event):
    sys.exit(0)

data_gen.t = -1
fig = plt.figure()
fig.canvas.mpl_connect('close_event', on_close_figure)
ax = fig.add_subplot(111)
line, = ax.plot([], [], lw=2)
ax.set_ylim(0, 120)
ax.set_xlim(0, xsize)
ax.grid()
xdata, ydata = [], []

# Important: Although blit=True makes graphing faster, we need blit=False to prevent
# spurious lines to appear when resizing the stripchart.
ani = animation.FuncAnimation(fig, run, data_gen, blit=False, interval=50, repeat=False)
plt.show()
