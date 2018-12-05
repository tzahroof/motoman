#!/usr/bin/env python
import rospy
# import Tkinter
from mttkinter import mtTkinter as tk
import matplotlib
import numpy as np
import matplotlib.cm as cm
import matplotlib.pyplot as plt

#message location for topic
from motoman_sia20d_moveit_updated.msg import trajPlot, trajPlotPoint, trajPlotPointInfo


#TODO: add plot.ion() for interactive plot
#TODO: turn this into a class so that variable info and stuff are shared


class Graph_Plotter:
	"""contains all the information from the graph"""
	""" graph_<plot> does the whole stack. set_<plot> sets the X,Y, etc values
	    plot_<plot> turns the X,Y, etc values into a graph"""
	def __init__(self,fig,ax):
		self.fig=fig
		self.ax=ax

	def plot_contour(self): #uses set X,Y,Z values when plotting
		self.CS = self.ax.contour(self.X,self.Y,self.Z)
		self.ax.clabel(self.CS, inline=1, fontsize = 10)

	def set_contour_values(self,X,Y,Z):
		self.X = X
		self.Y = Y
		self.Z = Z
		self.plot_contour()

	def graph_contour(self,X,Y,Z):
		self.set_contour_values(X,Y,Z)
		self.plot_contour()

	def plot_scatter(self):
		self.CS2 = self.ax.scatter(self.X_scat,self.Y_scat)


	def set_scatter_values(self,X_scat,Y_scat):
		self.X_scat = X_scat
		self.Y_scat = Y_scat
		self.plot_scatter()

	def graph_scatter(self,X_scat,Y_scat):
		self.set_scatter_values(X_scat,Y_scat)
		self.plot_scatter()

	def clear(self):
		self.ax.clear()


def callback(data):

	"""Translates ROSTOPIC information into useable graph data"""
	x_lil = []
	y_lil = []
	z_lil = []

	x_scat_lil =[]
	y_scat_lil =[]

	message = data #trajPlot.msg
	points = message.points #trajPlotPoint[]
	p_index = 0
	for p in points: #trajPlotPoint

		x_lil.append([])
		y_lil.append([])
		z_lil.append([])

		pointInfo = p.pointInfo #trajPlotPointInfo[]
		
		for pinfo in pointInfo: #trajPlotPointInfo

			if(pinfo.isPoint == True):
				x_scat_lil.append(pinfo.joint_value)
				y_scat_lil.append(p.point_number)

			x_lil[-1].append(pinfo.joint_value)
			y_lil[-1].append(p.point_number)
			z_lil[-1].append(pinfo.JacobianDeterminant)

	X = np.array(x_lil)
	Y = np.array(y_lil)
	Z = np.array(z_lil)

	X_scat = np.array(x_scat_lil)
	Y_scat = np.array(y_scat_lil)

	gp.graph_contour(X,Y,Z)
	plt.pause(0.001)
	gp.graph_scatter(X_scat,Y_scat)
	plt.pause(0.001)



# def listener():
# 	rospy.init_node('listener',anonymous = True)
# 	rospy.Subscriber("/trajectory_viewer_main/trajectory_plot", trajPlot, callback)
	
def is_number(s):
	"""is the string a number?"""
	try:
		float(s)
		return True
	except ValueError:
		return False


def receive_values_from_gui():
	"""Gets and prints the content of the entry"""
	if(is_number(angle_value.get()) and is_number(point_value.get())):
		newTrajectoryPoint = float(point_value.get())
		newAngleValue = int(angle_value.get())

		X_scat = gp.X_scat
		Y_scat = gp.Y_scat

		for i in range(0,Y_scat.shape[0]):
			if(newTrajectoryPoint == Y_scat[i]):
				X_scat[i] = newAngleValue
				Y_scat[i] = newTrajectoryPoint

		gp.clear()
		gp.plot_contour()
		plt.pause(0.001)
		gp.graph_scatter(X_scat, Y_scat)
		plt.pause(0.001)


plt.ion()
rospy.init_node('listener',anonymous = True)
rospy.Subscriber("/trajectory_viewer_main/trajectory_plot", trajPlot, callback)


fig, ax = plt.subplots()
ax.set_title('Trajectory Viewer')
ax.set_xlabel('Joint 1')
ax.set_ylabel('Trajectory Point Index')
gp = Graph_Plotter(fig,ax)


"""
use mttkinter to avoid main thread issues

"""
top = tk.Tk()
#Code to add widgets goes here
angle_label = tk.Label(top, text="Joint Angle")
angle_label.grid(row=0,column=0)
#angle_label.pack(side=Tkinter.LEFT)
angle_value = tk.Entry(top, bd=5)
#angle_value.pack(side=Tkinter.RIGHT)
angle_value.grid(row=0,column=1)

point_label = tk.Label(top, text="Trajectory Point Index")
#point_label.pack(side=Tkinter.LEFT)
point_label.grid(row=1,column=0)
point_value = tk.Entry(top,bd=5)
#point_value.pack(side=Tkinter.RIGHT)
point_value.grid(row=1,column=1)

confirm = tk.Button(top, text="Confirm", command=receive_values_from_gui) #
confirm.grid(row = 2,column=1)

top.mainloop()

	


# delta = 0.025
# # x = np.arange(-3.0,3.0,delta)
# # y = np.arange(-2.0,2.0,delta)
# # X, Y = np.meshgrid(x,y)
# # Z1 = np.exp(-X**2 - Y**2)
# # Z2 = np.exp(-(X-1)**2 - (Y-1)**2)
# # Z = (Z1-Z2) * 2

# x = [-5,-4,-3,-2,-1,0,1,2,3,4,5]
# y = [0,1,2]
# X,Y = np.meshgrid(x,y)
# Z = [[0,4,6,5,1,0,1,2,3,4,5],[-0,-4,-6,-5,-1,0,1,2,3,4,5],[0,4,-6,5,-1,0,1,2,3,4,5]]

# fig, ax = plt.subplots()
# CS = ax.contour(X,Y,Z)
# ax.clabel(CS, inline=1, fontsize = 10)
# ax.set_title('Trajectory Viewer')
# ax.set_xlabel('Joint 1')
# ax.set_ylabel('Trajectory Point Index')

# plt.show()
