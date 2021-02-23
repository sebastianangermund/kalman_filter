import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from kalman_filter_system_params import KalmanFilter


## IMPORT 1 MINUTE GRANULAR BITCOIN DATA
df = pd.read_csv("../bitcoin/bitcoin.csv")
df.timestamp_readable = [ts[:-3] for ts in df.timestamp_readable]	# Remove seconds from time format

interval_duration = (df["timestamp"].iloc[-1] - df["timestamp"].iloc[0]) / 60 # minutes
number_of_steps = df.shape[0]


## SET DISCRETE STEP LENGTH
#
# Experiment with different granularities for different data sets
#
#granularity = 1	# No filtering
#granularity = 6	# Finest granularity possible with noticable effect. For analysis in timeranges < 1h
#granularity = 30	# For analysing trends in timeranges ~ 5h
#granularity = 60	# For analysing trends in timeranges ~ 10h
#granularity = 120	# For analysing trends in timeranges > 20h (Bitcoin's variance is pretty narrow when zooming out more than ~24 h)

for granularity in [(120, 0)]: #[(6, 18), (30, 16), (60, 12), (120, 0)]:
	discrete_step = (interval_duration/number_of_steps) * (1/granularity[0])


	## INITIAL VALUES
	state_array = np.zeros(number_of_steps)
	state_array_der = np.zeros(number_of_steps)
	x_0 = np.array([[df["value"].iloc[0]], [0]], dtype=float)
	P_0 = np.eye(2)
	x_k = x_0
	P_k = P_0


	## GUESS STANDARD DEVIATION AND INITIATE FILTER
	standard_deviation = 1
	filter = KalmanFilter(P_k, x_k, standard_deviation, discrete_step)


	## FILTER DATA
	for index, row in df.iterrows():
		filter.update(row['value'])
		state_array[index] = filter.x_k[0]
		state_array_der[index] = filter.x_k[1]

	df["filtered"] = state_array


	## CALCULATE DERIVATIVES
	""" df["derivative_1"] = df["filtered"].diff() / df["timestamp"].diff()
	df["derivative_2"] = (df["filtered"] - 2*df["filtered"].shift(1) + df["filtered"].shift(2)) / df["timestamp"].diff() """


	## PLOT

	minutes = df["timestamp"] - df["timestamp"].iloc[0]
	minute_array = minutes.tolist()

	# PLOT SETTINGS
	start = int(number_of_steps * (granularity[1]/20))
	end = int(number_of_steps * (20/20))
	set_ticks = int(len(minute_array[start:end])/5)

	# PLOT NICE FORMAT
	fig, axes = plt.subplots(1, 1, figsize=(14,8))
	axes.plot(minute_array[start:end], df["value"].tolist()[start:end], 'r')
	axes.plot(minute_array[start:end], state_array[start:end], 'b')
	axes.set_title(f'One minute resolution SEK/Bitcoin')
	axes.legend(['Observed', 'Filtered'], loc='lower left')
	axes.grid(True)
	axes.set_xticks(minute_array[start:end:set_ticks])
	axes.set_xticklabels(df["timestamp_readable"].tolist()[start:end:set_ticks])

	plt.savefig(f'bitcoin_{granularity[0]}.png', bbox_inches='tight')
