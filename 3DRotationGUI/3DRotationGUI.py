from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import matplotlib
from Tkinter import *
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
matplotlib.use('TkAgg') #for placing matplotlib animation in tkinter

import serial
import time

ser = serial.Serial('COM3', 115200)


top = Tk()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
X, Y, Z = axes3d.get_test_data(0.1)
ax.plot_wireframe(X, Y, Z, rstride=5, cstride=5)

ax.view_init(elev=100, azim=30)

canvas = FigureCanvasTkAgg(fig, master=top)
canvas.get_tk_widget().pack()

def rotate():
	while(ser.inWaiting()):
		if(ser.inWaiting() > 0):
			s = ser.readline().split()
			ax.azim = (float(s[2]))
		else:
			print "nothing"
	fig.canvas.draw()
		
	
button = Button(top,text = "Click to rotate",command = rotate)
button.pack()



def presskey(event):
	if event.char is 'w':
		ax.elev += 5
	elif event.char is 'a':
		ax.azim -=5
	elif event.char is 's':
		ax.elev -= 5
	elif event.char  is 'd':
		ax.azim += 5
	fig.canvas.draw()

top.bind("<KeyPress>",presskey)

top.mainloop()
