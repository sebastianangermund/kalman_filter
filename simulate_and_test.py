import math

import numpy as np
import matplotlib.pyplot as plt

from kalman_filter_system_params import KalmanFilter


def simulate_values(k):
	return (2*np.cos(np.pi*k/12)-0.9*np.cos(np.pi*k/7)+1);


## NOISE PARAMETERS
mean = 0
standard_dev = 2
variance = math.pow(standard_dev, 2)


## TIME PARAMETERS

# interval_duration = 24*20;   % hours
# number_of_steps = interval_duration*60/5;    % 5 minute resolution
# h = interval_duration/number_of_steps;    % time discretization step length


interval_duration = 24*2  # hours
resolution = 10  # minutes
number_of_steps = int(interval_duration*60/resolution)
discrete_step = interval_duration/number_of_steps
time_array = np.linspace(0, interval_duration, number_of_steps)


## VALUE ARRAYS
state_array = np.zeros(number_of_steps)
state_array_der = np.zeros(number_of_steps)
y_true = simulate_values(time_array)
noise = np.random.normal(mean, standard_dev, time_array.size)
y_observed = y_true + noise


## INITIAL VALUES
x_0 = np.array([[0], [0]], dtype=float)
P_0 = np.eye(2)
x_k = x_0
P_k = P_0


## GUESS STANDARD DEVIATION AND FILTER OBSERVED
st_dev = 2
filter = KalmanFilter(P_k, x_k, st_dev, discrete_step)

for k in range(0, time_array.size):
	filter.update(y_observed[k])
	state_array[k] = filter.x_k[0]
	state_array_der[k] = filter.x_k[1]


## PLOT
plt.figure(figsize=(10, 4))
plt.plot(time_array, y_observed, 'y');
plt.plot(time_array, y_true, 'r')
plt.plot(time_array, state_array, 'b');
plt.title(f'{resolution} minute resolution')
plt.xlabel('hours')
plt.legend(['Observed', 'True', 'Filtered'], loc='lower right')

plt.savefig(f'res{resolution}.png', bbox_inches='tight')
