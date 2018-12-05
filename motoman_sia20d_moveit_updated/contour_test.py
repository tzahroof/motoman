#!/usr/bin/env python
import rospy
#import Tkinter
import matplotlib
import numpy as np
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import time
from mttkinter import mtTkinter as tk

# from matplotlib.backends.backend_tkagg import (
#     FigureCanvasTkAgg, NavigationToolbar2Tk)
# # Implement the default Matplotlib key bindings.
# from matplotlib.backend_bases import key_press_handler
# from matplotlib.figure import Figure
from matplotlib.widgets import Slider


# TODO: Look into plt.ion() for interactive plot

class Graph_Plotter:
	def __init__(self, fig, ax):
		self.fig = fig
		self.ax = ax

	def plot_contour(self):
		self.CS = self.ax.contour(self.X,self.Y,self.Z)
		self.ax.clabel(self.CS, inline=1, fontsize = 10)

	def set_contour_values(self,X,Y,Z):
		self.X = X
		self.Y = Y
		self.Z = Z
	
	def graph_contour(self,X,Y,Z):
		self.set_contour_values(X,Y,Z)
		self.plot_contour()


	def plot_scatter(self,X_scat,Y_scat):
		self.CS2 = self.ax.scatter(X_scat,Y_scat)

	def show(self):
		plt.show(block=False)

	def clear(self):
		self.ax.clear()

delta = 0.025
# x = np.arange(-3.0,3.0,delta)
# y = np.arange(-2.0,2.0,delta)
# X, Y = np.meshgrid(x,y)
# Z1 = np.exp(-X**2 - Y**2)
# Z2 = np.exp(-(X-1)**2 - (Y-1)**2)
# Z = (Z1-Z2) * 2

x = [[-5,-4,-3,-2,-1,0,1,2,3,4,5],[-5,-4,-3,-2,-1,0,1,2,3,4,5],[-5,-4,-3,-2,-1,0,1,2,3,4,5]]
X = np.array(x)
y = [[ 0, 0, 0, 0, 0,0,0,0,0,0,0],[ 1, 1, 1, 1, 1,1,1,1,1,1,1],[ 2, 2, 2, 2, 2,2,2,2,2,2,2]]
Y = np.array(y)
#X,Y = np.meshgrid(x,y)
z = [[0,4,6,5,1,0,1,2,3,4,5],[-0,-4,-6,-5,-1,0,1,2,3,4,5],[0,4,-6,5,-1,0,1,2,3,4,5]]
Z = np.array(z)


X_scat = np.arange(-2,2,0.1)
Y_scat = np.arange(-1,3,0.1)

# fig, ax = plt.subplots()
# plt.subplots_adjust(left=0.25, bottom=0.25)
# t = np.arange(0.0, 1.0, 0.001)
# a0 = 5
# f0 = 3
# s = a0*np.sin(2*np.pi*f0*t)
# l, = plt.plot(t, s, lw=2, color='red')
# plt.axis([0, 1, -10, 10])

# axcolor = 'lightgoldenrodyellow'
# axfreq = plt.axes([0.25, 0.1, 0.65, 0.03], facecolor=axcolor)
# axamp = plt.axes([0.25, 0.15, 0.65, 0.03], facecolor=axcolor)

# sfreq = Slider(axfreq, 'Freq', 0.1, 30.0, valinit=f0)
# samp = Slider(axamp, 'Amp', 0.1, 10.0, valinit=a0)


# def update(val):
#     amp = samp.val
#     freq = sfreq.val
#     l.set_ydata(amp*np.sin(2*np.pi*freq*t))
#     fig.canvas.draw_idle()
# sfreq.on_changed(update)
# samp.on_changed(update)

# resetax = plt.axes([0.8, 0.025, 0.1, 0.04])
# button = Button(resetax, 'Reset', color=axcolor, hovercolor='0.975')


# def reset(event):
#     sfreq.reset()
#     samp.reset()
# button.on_clicked(reset)

# rax = plt.axes([0.025, 0.5, 0.15, 0.15], facecolor=axcolor)
# radio = RadioButtons(rax, ('red', 'blue', 'green'), active=0)


# def colorfunc(label):
#     l.set_color(label)
#     fig.canvas.draw_idle()
# radio.on_clicked(colorfunc)

# plt.show()



fig, ax = plt.subplots()
# plt.subplots_adjust(left=0.25,bottom=0.25)

# axcolor = 'lightgoldenrodyellow'
# axfreq = plt.axes([0.25, 0.1, 0.65, 0.03], facecolor=axcolor)
# axamp = plt.axes([0.25, 0.15, 0.65, 0.03], facecolor=axcolor)

plt.ion()
fig, ax = plt.subplots()
ax.set_title('Trajectory Viewer')
ax.set_xlabel('Joint 1')
ax.set_ylabel('Trajectory Point Index')

time.sleep(2)

gp = Graph_Plotter(fig,ax)
gp.graph_contour(X,Y,Z);
plt.pause(0.0001)
gp.plot_scatter(X_scat,Y_scat)
plt.pause(0.0001)

# time.sleep(3);

# print("finished timer")
# X_scat = np.arange(-4,0,0.1)
# Y_scat = np.arange(-2,2,0.1)

# gp.clear()
# gp.graph_contour(X,Y,Z)
# plt.pause(0.001)
# gp.plot_scatter(X_scat,Y_scat)
# plt.pause(0.001)



print("We made it here")


def return_angle_value(event):
	"""Gets and prints the content of the entry"""
	if(str.isdigit(angle_value.get())):
		content = angle_value.get()
		print(content)

def callback():
	"""Gets and prints the content of the entry"""
	print("click!")




top = tk.Tk()

#Code to add widgets goes here
angle_label = tk.Label(top, text="Joint Angle")
angle_label.grid(row=0,column=0)

angle_value = tk.Entry(top, bd=5)
angle_value.bind("<Return>", return_angle_value)
#angle_value.pack(side=tk.RIGHT)
angle_value.grid(row=0,column=1)

point_label = tk.Label(top, text="Trajectory Point Index")
#point_label.pack(side=tk.LEFT)
point_label.grid(row=1,column=0)
point_value = tk.Entry(top,bd=5)
#point_value.pack(side=tk.RIGHT)
point_value.grid(row=1,column=1)

confirm = tk.Button(top, text="Confirm", command=callback)
confirm.grid(row = 2,column=1)

top.mainloop()



# top.wm_title("Embedding in Tk")

# fig = Figure(figsize=(5, 4), dpi=100)
# t = np.arange(0, 3, .01)
# fig.add_subplot(111).plot(t, 2 * np.sin(2 * np.pi * t))

# canvas = FigureCanvasTkAgg(fig, master=top)  # A tk.DrawingArea.
# canvas.draw()
# canvas.get_tk_widget().pack(side=Tkinter.TOP, fill=Tkinter.BOTH, expand=1)

# toolbar = NavigationToolbar2Tk(canvas, top)
# toolbar.update()
# canvas.get_tk_widget().pack(side=Tkinter.TOP, fill=Tkinter.BOTH, expand=1)


# def on_key_press(event):
#     print("you pressed {}".format(event.key))
#     key_press_handler(event, canvas, toolbar)


# canvas.mpl_connect("key_press_event", on_key_press)


# def _quit():
#     top.quit()     # stops mainloop
#     top.destroy()  # this is necessary on Windows to prevent
#                     # Fatal Python Error: PyEval_RestoreThread: NULL tstate


# button = Tkinter.Button(master=top, text="Quit", command=_quit)
# button.pack(side=Tkinter.BOTTOM)

# Tkinter.mainloop()
# If you put root.destroy() here, it will cause an error if the window is
# closed with the window manager.