#!/usr/bin/env python

from threading import Thread
import serial
from serial import Serial
import time
import datetime
import collections
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import struct
import copy
import pandas as pd
from matplotlib.pyplot import cm
import numpy as np

from tkinter import *
import tkinter.messagebox as tkMessageBox

import data_extender as rc

class serialPlot:
	def __init__(self, serialPort='/dev/cu.usbmodem14101', serialBaud=9600, plotLength=100, dataNumBytes=2, numPlots=22):
		self.port = serialPort
		self.baud = serialBaud
		self.plotMaxLength = plotLength
		self.dataNumBytes = dataNumBytes
		self.numPlots = numPlots
		self.rawData = bytearray(numPlots * dataNumBytes)
		self.dataType = None
		if dataNumBytes == 2:
			self.dataType = 'h'     # 2 byte integer
		elif dataNumBytes == 4:
			self.dataType = 'f'     # 4 byte float
		self.data = []
		self.dataCheck = None;
		for i in range(numPlots):   # give an array for each type of data and store them in a list
			self.data.append(collections.deque([0] * plotLength, maxlen=plotLength))
		self.isRun = True
		self.isReceiving = False
		self.thread = None
		self.plotTimer = 0
		self.previousTimer = 0
		self.csvData = []
		self.counter = 0
		self.path = 'path/to/files/'
		self.filename = None
		self.saveToPath = None
		self.params_dict = {}

		print('Trying to connect to: ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
		try:
			self.serialConnection = serial.Serial(serialPort, serialBaud, timeout=4)
			print('Connected to ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
		except:
			print("Failed to connect with " + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
	
	def sendData(self, msg):
		if not self.serialConnection.is_open:
			self.serialConnection.open()
		self.serialConnection.write(msg.encode())
		self.serialConnection.flush()
	
	def start_and_calibrate(self):
		serialPlot.sendData(self, '&\n')
		time.sleep(5)
		
	def startCalc(self):
		serialPlot.sendData(self, '$')
	
	def stopCalc(self):
		serialPlot.sendData(self, '%')
		
	def killMotors(self):
		serialPlot.sendData(self, 'K')
	
	def stance(self):
		serialPlot.sendData(self, 'S')
		time.sleep(1)
		
	def getParameterInfos(self):
		serialPlot.sendData(self, 'i')
		params_data = 0
		while params_data == 0:
			params_data = self.serialConnection.readline()
			params_str = str(params_data.decode("utf-8")).rstrip()
		print(params_str)
		params_arr = serialPlot.params_parser(params_str)
		self.params_dict =  {x[0] : x[1] for x in params_arr}

	def readSerialStart(self):
		if self.thread == None:
			self.thread = Thread(target=self.backgroundThread)
			self.thread.start()
			# Block till we start receiving values
			while self.isReceiving != True:
				time.sleep(0.1)

	def getSerialData(self, frame, lines, lineValueText, lineLabel, timeText):
		currentTimer = time.perf_counter()
		self.plotTimer = int((currentTimer - self.previousTimer) * 1000)     # the first reading will be erroneous; will be ignored
		self.previousTimer = currentTimer
		timeText.set_text('Plot Interval = ' + str(self.plotTimer) + 'ms')
		privateData = copy.deepcopy(self.rawData[:])    # so that the values in our plots will be synchronized to the same sample time
		for i in range(self.numPlots):
			data = privateData[(i*self.dataNumBytes):(self.dataNumBytes + i*self.dataNumBytes)]
			value,  = struct.unpack(self.dataType, data)
			self.data[i].append(value)    # we get the latest data point and append it to our array
			lines[i].set_data(range(self.plotMaxLength), self.data[i])
			lineValueText[i].set_text('[' + lineLabel[i] + '] = ' + str(value))
		if self.dataCheck != self.data[0][-1] and self.plotTimer < 1000:	
			self.csvData.append([self.data[i][-1] for i in range(self.numPlots)])
		self.dataCheck = self.data[0][-1]
		
	def getSerialData_noPlot(self):
			currentTimer = time.perf_counter()
			self.plotTimer = int((currentTimer - self.previousTimer) * 1000)     # the first reading will be erroneous; will be ignored with self.plotTimer < 1000
			self.previousTimer = currentTimer
			privateData = copy.deepcopy(self.rawData[:])    # so that the values in our plots will be synchronized to the same sample time
			for i in range(self.numPlots):
				data = privateData[(i*self.dataNumBytes):(self.dataNumBytes + i*self.dataNumBytes)]
				value,  = struct.unpack(self.dataType, data)
				self.data[i].append(value)    # we get the latest data point and append it to our array
			if self.dataCheck != self.data[0][-1] and self.plotTimer < 1000:	
				self.csvData.append([self.data[i][-1] for i in range(self.numPlots)])
			self.dataCheck = self.data[0][-1]

		
	def backgroundThread(self):    # retrieve data
		time.sleep(1.0)  # give some buffer time for retrieving data
		self.serialConnection.reset_input_buffer()
		while (self.isRun):
			self.serialConnection.readinto(self.rawData)
			self.isReceiving = True
			#print(self.rawData)
			
	def params_parser(params_str):
		string_arr = params_str.split(', ')
		params_arr = np.empty([1,2])
		for x in string_arr:
			y = x.split('=')
			params_arr = np.vstack((params_arr, y))
		params_arr = np.delete(params_arr, 0, 0)
		return params_arr
		
	
	def open_file(self, ground_num, load_num=0):
		ts = time.time()
		st = datetime.datetime.fromtimestamp(ts).strftime('%d-%m-%Y_%H'+'h-%Mm-%Ss')
		
		gait_arr = ['tripod', 'tetrapod', 'wave']
		gait_num = int(self.params_dict['gait'])
		feedback_loop_arr = ['open', 'closed']
		feedback_loop_num = int(self.params_dict['loop'])
		sensors_arr = ['vertical', 'horizontal']
		sensors_num = int(self.params_dict['sensors'])
		ground_arr = ['air', 'floor', 'inclined', 'declined']
		load = ['unloaded', 'loaded']
		data = ['x', 'b', 'sensors']
		data_str = data[0] + '_' + data[1] + '_' + data[2]

		legenum = '#0: front left\n#1: middle left\n#2: back left\n#3: back right\n#4: middle right\n#5: front right'
		columns = '#number,' + str(['#out' + str(i) for i in range(self.numPlots)])[1:-1].replace("'", "")
		
		if str(self.params_dict['identity']) == '1':
			self.filename = 'identity' + '_' + gait_arr[gait_num] + '_' + data_str + '_' + feedback_loop_arr[feedback_loop_num] + '_' + sensors_arr[sensors_num] + '_' + ground_arr[ground_num] + '_' + load[load_num] + '_' + st
		else:
			self.filename = str(gait_arr[gait_num]) + '_' + str(data_str) + '_' + str(feedback_loop_arr[feedback_loop_num]) + '_' + str(sensors_arr[sensors_num]) + '_' + str(ground_arr[ground_num]) + '_' + str(load[load_num]) + '_' + str(st)
		
		self.saveToPath = self.path + self.filename + '.csv'
		
		f = open(self.saveToPath, 'a')
		
		surface = 'rubber'
		dtype = 'int'
		comments = '#-calibration: for gait specific motor limits due to weight force\n#-sensor_start: number of loops when feedback loop closes\n#-sensor_start ensures starting stability for closed loop\n#-because motors are in stance position at the start'
		
		f.write('#Recorded data from Servotor32\n')
		f.write('#datetime: ' + str(st) + '\n')
		f.write('#filename: ' + self.filename + '.csv\n')
		f.write('#\n')
		f.write('#------------------------------------------------------\n')
		f.write('#Settings:\n')
		f.write('#\n')
		f.write('#Identity: ' + str(self.params_dict['identity']) + '\n')
		f.write('#Gait: ' + str(gait_arr[gait_num]) + '\n')
		f.write('#Feedback config: ' + str(feedback_loop_arr[feedback_loop_num]) + '\n')
		f.write('#Sensors: ' + str(sensors_arr[sensors_num]) + '\n')
		f.write('#Sensors cut: ' + str(self.params_dict['sensor_cutoff']) + '\n')
		f.write('#\n')
		f.write('#------------------------------------------------------\n')
		f.write('#Parameters:\n')
		f.write('#\n')
		f.write('#g=' + str(self.params_dict['g']) + '\n')
		f.write('#y_bar=' + str(self.params_dict['y_bar']) + '\n')
		f.write('#a=' + str(self.params_dict['a']) + '\n')
		f.write('#tau_x=' + str(self.params_dict['tau_x']) + '\n')
		f.write('#tau_b=' + str(self.params_dict['tau_b']) + '\n')
		f.write('#Y_max=' + str(self.params_dict['Y_max']) + '\n')
		f.write('#sensor_start=' + str(self.params_dict['sensor_start']) + '\n')
		f.write('#dt=' + str(self.params_dict['dt']) + 'ms\n')
		f.write('#direction=' + str(self.params_dict['direction']) + '\n')
		f.write('#ics_tau=' + str(self.params_dict['ics_tau']) + '\n')
		f.write('#\n')
		f.write('#------------------------------------------------------\n')
		f.write('#Recording info:\n')
		f.write('#\n')
		f.write('#Ground: ' + ground_arr[ground_num] + '\n')
		f.write('#Extra load: ' + load[load_num] + '\n')
		f.write('#Tripod calibration: ' + str(self.params_dict['tripod_calibration']) + '\n')
		f.write('#Surface: ' + surface + '\n')
		f.write('#Recorded datatype: ' + dtype + '\n')
		f.write('#\n')
		f.write(comments + '\n')
		f.write('#\n')
		f.write('#------------------------------------------------------\n')
		f.write('#Leg numbering:\n')
		f.write('#\n') 
		f.write(legenum + '\n')
		f.write('#\n')
		f.write('#------------------------------------------------------\n')
		f.write('#Recorded variables:\n')
		f.write('#\n')
		f.write('#n: step count\n')
		f.write('#t: total time\n')
		f.write('#dt: step time\n')
		f.write('#delay: waiting time for calculation time to reach step time dt\n')
		f.write('#\n')
		f.write('#x_i: neural membrane\n')
		f.write('#b_v_i: neural bias (vertical)\n')
		f.write('#sensor_i: normalized sensor values\n')
		f.write('#\n')
		f.write('#------------------------------------------------------\n')
		f.write('#Calculated variables from recorded data:\n')
		f.write('#\n')
		f.write('#fb_i: membrane feedback unscaled\n')
		f.write('#y_v_i: neural activation (vertical)\n')
		f.write('#b_v_avg_i: neural bias average value (vertical)\n')
		f.write('#b_v_min: minimal neural bias value of current period\n')
		f.write('#b_v_max: maximal neural bias value of current period\n')
		f.write('#y_h_i: neural activation (horizontal)\n')
		f.write('#b_v_sens_h_i: vertical neural bias (via horizontal sensors)\n')
		f.write('#\n')
		f.write('#------------------------------------------------------\n')
		f.write('#File info:\n')
		f.write('#\n')
		f.write('##out marks the columns recorded on Servotor32\n')
		f.write('#Initial conditions in first row at n=-1\n')
		f.write('#\n')
		f.write('#------------------------------------------------------\n')
		f.write('#Data:')
		f.write('#\n')
		
		f.write(columns + '\n')
		
		f.close()
		
	def save_file(self):
		df = pd.DataFrame(self.csvData)
		f = open(self.saveToPath, 'a')
		df.to_csv(f)
		f.close()
		print('Saved data in ', self.saveToPath)
		
	def close(self):
		self.isRun = False
		self.thread.join()
		self.serialConnection.close()
		print('Disconnected...')


def main():	
	# portName = 'COM5'     # for windows users
	#portName = '/dev/cu.usbmodem14101' # MacBook Pro
	portName = '/dev/cu.ArcBotics-DevB'	# bluetooth
	# portName = '/dev/ttyUSB0'
	
	if portName == '/dev/cu.ArcBotics-DevB':
		baudRate = 115200
	else:
		baudRate = 9600
		
	maxPlotLength = 100     # number of points in x-axis of real time plot
	dataNumBytes = 2        # number of bytes of 1 data point
	numPlots = 22           # number of plots in 1 graph
	
	ground_arr = ['air', 'floor', 'inclined', 'declined']
	load = ['unloaded', 'loaded']
	
	ground_num = 3	#0: air, 1: floor
	load_num = 1	#0: unloaded, 1: loaded
	recording_time = 45
	
	s = serialPlot(portName, baudRate, maxPlotLength, dataNumBytes, numPlots)   # initializes all required variables
	s.getParameterInfos()
	
	info_str = 'On ground: ' + str(ground_arr[ground_num]) + '\nLoad: ' + str(load[load_num]) + '\nRecording time: ' + str(recording_time) + 's\nTripod calibration: ' + str(s.params_dict['tripod_calibration']) + '\nSurface: rubber'
	print(info_str)
	window = Tk()
	window.wm_withdraw()
	#message at x:200,y:200
	window.geometry("1x1+200+200")	#.geometry("WidthxHeight(+or-)X(+or-)Y")
	tkMessageBox.showerror(title="Info",message=info_str,parent=window)
	

	show_plot = False
	try:
		if show_plot:	# not optimized for display yet
			# plotting starts below
			pltInterval = 100    # Period at which the plot animation updates [ms]
			xmin = 0
			xmax = maxPlotLength
			ymin = -5000
			ymax = 5000
			fig = plt.figure(figsize=(10, 8))
			ax = plt.axes(xlim=(xmin, xmax), ylim=(ymin, ymax))
			ax.set_title('Arduino Analog Read')
			ax.set_xlabel("time")
			ax.set_ylabel("AnalogRead Value")
			
			lineLabel = ['out' + str(i) for i in range(numPlots)]
		#	style = ['b-', 'g-', 'r-', 'c-', 'm-', 'y-','k-']  # linestyles for the different plots
			timeText = ax.text(0.70, 0.95, '', transform=ax.transAxes)
			lines = []
			lineValueText = []
			for i in range(numPlots):
		#		lines.append(ax.plot([], [], color = (0, i / 16.0, 0, 1), label=lineLabel[i])[0])
				lines.append(ax.plot([], [], label=lineLabel[i])[0])
				lineValueText.append(ax.text(0.70, 0.90-i*0.05, '', transform=ax.transAxes))
			
			plt.legend(loc="upper left")
		
		s.stopCalc()
		s.stance()
			
		s.open_file(ground_num, load_num)	# getParameterInfos() needed
		s.readSerialStart()			# starts background thread
		s.start_and_calibrate()
		
		warning_str = 'Start running'
		window = Tk()
		window.wm_withdraw()
		#message at x:200,y:200
		window.geometry("1x1+200+200")	#.geometry("WidthxHeight(+or-)X(+or-)Y")
		tkMessageBox.showerror(title="Run",message=warning_str,parent=window)
		
		s.startCalc()
			
		if show_plot:
			anim = animation.FuncAnimation(fig, s.getSerialData, fargs=(lines, lineValueText, lineLabel, timeText), interval=pltInterval)    # fargs has to be a tuple
			plt.show()
		else:
			timeout = time.time() + recording_time   # approx. recording_time seconds from now
			while True:
				if time.time() > timeout:
					break
				s.getSerialData_noPlot()
				time.sleep(0.05)
		
		s.save_file()
		s.stopCalc()
		s.killMotors()
		s.stance()
		s.killMotors()
		s.close()
				
		header = rc.read_header(s.saveToPath)
		df = rc.read_to_pandas(s.saveToPath)
		df = rc.rename_cols(df)
		df = rc.insert_missing(df)
		df = rc.calculate(df=df, weight=float(s.params_dict['g']), neuron_avg=float(s.params_dict['y_bar']), gain_vert=float(s.params_dict['a']), tau_b=float(s.params_dict['tau_b']), tau_x=float(s.params_dict['tau_x']), neuron_horiz_max=float(s.params_dict['Y_max']), gait_num=int(s.params_dict['gait']), feedback_loop=int(s.params_dict['loop']), sensors=int(s.params_dict['sensors']), sensor_start=int(s.params_dict['sensor_start']), direction=float(s.params_dict['direction']), ics_tau=float(s.params_dict['ics_tau']), dt=float(s.params_dict['dt'])*0.001, cut=bool(s.params_dict['sensor_cutoff']),
			identity=bool(s.params_dict['identity']))
		
		df = rc.to_float(df)
		rc.write_csv(header, df, s.path, s.filename)
		print('Modified datafile saved.')
	
	except Exception as e: 
		print('Data recording failed: ', e)
		s.stopCalc()
		s.stance()
		s.killMotors()	

if __name__ == '__main__':
	main()
