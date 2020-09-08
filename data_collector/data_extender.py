import numpy as np
import pandas as pd

from itertools import takewhile


def read_to_pandas(file_path):
	df = pd.read_csv(file_path, comment='#', index_col=0)
	return df


def rename_cols(df):
	df = df.rename(columns={"0": "n", "1": "t", "2": "dt", "3": "delay"})
	for i in range(6):
		df = df.rename(columns={str(i + 4): "x_" + str(i)})
		df = df.rename(columns={str(i + 10): "b_v_" + str(i)})
		df = df.rename(columns={str(i + 16): "sensor_" + str(i)})
	return df


def insert_missing(df):
	for i in range(6):
		df.insert(22 + i, 'sensor_cut_' + str(i), 0)
	for i in range(6):
		df.insert(28 + i, 'fb_' + str(i), 0)
	for i in range(6):
		df.insert(34 + i, 'y_v_' + str(i), 0)
	for i in range(6):
		df.insert(40 + i, 'b_v_avg_' + str(i), 0)
	for i in range(6):
		df.insert(46 + i, 'b_v_min_' + str(i), 0)
	for i in range(6):
		df.insert(52 + i, 'b_v_max_' + str(i), 0)
	for i in range(6):
		df.insert(58 + i, 'y_h_' + str(i), 0)
	for i in range(6):
		df.insert(64 + i, 'b_v_sens_h_' + str(i), 0)
	for i in range(6):
		df.insert(70 + i, 'a_h_' + str(i), 0)
	return df


def	read_header(file_path):
	with open(file_path, 'r') as fobj:
		# takewhile returns an iterator over all the lines 
		# that start with the comment string
		headiter = takewhile(lambda s: s.startswith('#'), fobj)
		# you may want to process the headers differently, 
		# but here we just convert it to a list
		header = list(headiter)
	return header


#calculation: deviation from the actual data due to rounding to 10^-3
#higher accuracy not needed
def calculate(df, gait_num, feedback_loop, sensors, weight, neuron_avg, gain_vert, tau_b, tau_x, neuron_horiz_max, sensor_start, direction, ics_tau, dt, cut):
	
	legsets = [2, 3, 6]
	legpairs = [3, 2, 1]

	matrix = np.zeros((6, 6))

	tripod_seq = [1, 2, 1, 2, 1, 2]
	tetrapod_seq = [1, 2, 3, 1, 3, 2]
	wave_seq = [1, 2, 3, 4, 5, 6]
	gait_seq_arr = [tripod_seq, tetrapod_seq, wave_seq]

	membrane = np.zeros(6)
	bias_vert = np.zeros(6)
	analog_sensor_norm_real = np.zeros(6)
	analog_sensor_norm = np.zeros(6)
	feedback = np.zeros(6)
	neuron_vert = np.zeros(6)
	bias_vert_old = np.zeros(6)
	bias_vert_diff_old = np.zeros(6)
	bias_vert_avg = np.zeros(6)
	bias_vert_min = np.zeros(6)
	bias_vert_max = np.zeros(6)
	bias_vert_sens_horiz = np.zeros(6)
	neuron_horiz = np.zeros(6)
	gain_horiz = np.zeros(6)

	for i in range(6):
		for j in range(6):
			matrix[i][j] = 1 - 2 * ((gait_seq_arr[gait_num][j] + (legsets[gait_num]
					- gait_seq_arr[gait_num][i])) % legsets[gait_num]) / (legsets[gait_num] - 1)
		membrane[i] = df['x_' + str(i)].values[0] * 0.001
		bias_vert[i] = df['b_v_' + str(i)].values[0] * 0.001
		analog_sensor_norm_real[i] = df['sensor_' + str(i)].values[0] * 0.001
		
	for i in range(6):
		if (matrix[i][0] == 1):
			b_dot_bin = (1 - neuron_avg) / tau_b
		else:
			b_dot_bin = - neuron_avg / tau_b
		
		bias_vert_old[i] = legpairs[gait_num] * weight * (matrix[i][0] + 1) / 2 - b_dot_bin * (ics_tau + dt)

		neuron_vert[i] = 1 / (1 + np.exp(gain_vert * (bias_vert[i] - membrane[i])))

		bias_vert_diff_old[i] = bias_vert[i] - bias_vert_old[i]
		bias_vert_min[i] = 0
		bias_vert_max[i] = legpairs[gait_num] * weight
		bias_vert_avg[i] = (bias_vert_max[i] + bias_vert_min[i]) / 2
		bias_vert_sens_horiz[i] = bias_vert[i]
		gain_horiz[i] = direction * 2 * np.log(1 / neuron_horiz_max - 1) / (bias_vert_min[i] - bias_vert_max[i])
		neuron_horiz[i] = 1 / (1 + np.exp(gain_horiz[i] * (bias_vert_avg[i] - bias_vert[i])))
		
		for j in range(6):
			feedback[i] += matrix[i][j] * neuron_vert[j]
		
		analog_sensor_norm[i] = analog_sensor_norm_real[i]
		
		if cut:	
			if analog_sensor_norm_real[i] > 1:
				analog_sensor_norm[i] = 1
			elif analog_sensor_norm_real[i] < 0:
				analog_sensor_norm[i] = 0			
				
		df['sensor_cut_' + str(i)].values[0] = np.around(analog_sensor_norm[i] * 1000, 0)
		df['y_v_' + str(i)].values[0] = np.around(neuron_vert[i] * 1000, 0)
		df['y_h_' + str(i)].values[0] = np.around(neuron_horiz[i] * 1000, 0)
		df['b_v_sens_h_' + str(i)].values[0] = np.around(bias_vert_sens_horiz[i] * 1000, 0)
		df['b_v_avg_' + str(i)].values[0] = np.around(bias_vert_avg[i] * 1000, 0)
		df['fb_' + str(i)].values[0] = np.around(feedback[i] * 1000, 0)
		df['b_v_max_' + str(i)].values[0] = np.around(bias_vert_max[i] * 1000, 0)
		df['b_v_min_' + str(i)].values[0] = np.around(bias_vert_min[i] * 1000, 0)
		df['a_h_' + str(i)].values[0] = np.around(gain_horiz[i] * 1000, 0)

	for k in range(1, df['n'].count()):
		step_count = df['n'][k]
		
		for i in range(6):
			analog_sensor_norm_real[i] = df['sensor_' + str(i)].values[k] * 0.001		
			analog_sensor_norm[i] = analog_sensor_norm_real[i]
			
			if cut:
				if analog_sensor_norm_real[i] > 1:
					analog_sensor_norm[i] = 1
				elif analog_sensor_norm_real[i] < 0:
					analog_sensor_norm[i] = 0
				

		if ((sensors == 0) and (feedback_loop == 1) and (step_count > sensor_start)):
			for i in range(6):
				feedback[i] = 0
				for j in range(6):
					feedback[i] += matrix[i][j] * analog_sensor_norm[j]
		else:
			for i in range(6):
				feedback[i] = 0
				for j in range(6):
					feedback[i] += matrix[i][j] * neuron_vert[j]

		for i in range(6):
			membrane[i] = df['x_' + str(i)].values[k] * 0.001

		if ((sensors == 1) and (feedback_loop == 1) and (step_count > sensor_start)):
			for i in range(6):
				bias_vert_sens_horiz[i] = bias_vert_avg[i] + direction * ((1 - analog_sensor_norm[i]) * (bias_vert_min[i] - bias_vert_avg[i]) + analog_sensor_norm[i] * (bias_vert_max[i] - bias_vert_avg[i]))
				neuron_vert[i] = 1 / (1 + np.exp(gain_vert * (bias_vert_sens_horiz[i] - membrane[i])))
		else:
			for i in range(6):
				neuron_vert[i] = 1 / (1 + np.exp(gain_vert * (bias_vert[i] - membrane[i])))
				

		for i in range(6):
			bias_vert[i] = df['b_v_' + str(i)].values[k] * 0.001

			if (((bias_vert[i] - bias_vert_old[i]) < 0) and ((bias_vert[i] - bias_vert_old[i]) * bias_vert_diff_old[i] < 0)):
				bias_vert_max[i] = bias_vert_old[i]
			elif (((bias_vert[i] - bias_vert_old[i]) > 0) and ((bias_vert[i] - bias_vert_old[i]) * bias_vert_diff_old[i] < 0)):
				bias_vert_min[i] = bias_vert_old[i]

			if (bias_vert[i] > bias_vert_max[i]):
				bias_vert_max[i] = bias_vert[i]
			elif (bias_vert[i] < bias_vert_min[i]):
				bias_vert_min[i] = bias_vert[i]

			bias_vert_diff_old[i] = bias_vert[i] - bias_vert_old[i]
			bias_vert_old[i] = bias_vert[i]

			bias_vert_avg[i] = (bias_vert_max[i] + bias_vert_min[i]) / 2
			
			gain_horiz[i] = direction * 2 * np.log(1 / neuron_horiz_max - 1)/(bias_vert_min[i] - bias_vert_max[i])
			
			neuron_horiz[i] = 1 / (1 + np.exp(gain_horiz[i] * (bias_vert_avg[i] - bias_vert[i])))

		for i in range(6):
			df['sensor_cut_' + str(i)].values[k] = np.around(analog_sensor_norm[i] * 1000, 0)
			df['y_v_' + str(i)].values[k] = np.around(neuron_vert[i] * 1000, 0)
			df['y_h_' + str(i)].values[k] = np.around(neuron_horiz[i] * 1000, 0)
			df['b_v_sens_h_' + str(i)].values[k] = np.around(bias_vert_sens_horiz[i] * 1000, 0)
			df['b_v_avg_' + str(i)].values[k] = np.around(bias_vert_avg[i] * 1000, 0)
			df['fb_' + str(i)].values[k] = np.around(feedback[i] * 1000, 0)
			df['b_v_max_' + str(i)].values[k] = np.around(bias_vert_max[i] * 1000, 0)
			df['b_v_min_' + str(i)].values[k] = np.around(bias_vert_min[i] * 1000, 0)
			df['a_h_' + str(i)].values[k] = np.around(gain_horiz[i] * 1000, 0)
	return df
		
def to_float(data):
	df = data.copy()
	df.loc[(df['t'] < 0) & (df['n'] > 0), 't'] = 32767 + 32769 + df['t']
	df.iloc[:, 1:] = df.iloc[:, 1:].astype(float) / 1000
	return df
		
def write_csv(header, df, path, filename):
	for i in range(100):
		try:
			result_path = path + filename + '_mod' + str(i) + '.csv'
			f = open(result_path, mode='x')
		except:
			pass
		else:
			break

	for comment in header:
		f.write(comment)
	
	df.to_csv(f, index=True, mode='a')
	f.close()

modify_existing = False
if modify_existing:
	path = ''
	filename = ''
	file_path = path + filename + '.csv'
	header = read_header(file_path)
	df = read_to_pandas(file_path)
	df = rename_cols(df)
	df = insert_missing(df)
	df = calculate(df=df, gait_num=0, feedback_loop=1, sensors=1, weight=1.7, neuron_avg=0.4, gain_vert=5, tau_b=0.2, tau_x=0.13, neuron_horiz_max=0.95, sensor_start=5, direction=1, ics_tau=0.1, dt=105, cut=1)
	df = to_float(df)
	write_csv(header, df, path, filename)