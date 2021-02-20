%% KALMAN FILTER. Tracking 1D movement.
%
% x''(t) = f(t) is the DE for the exact movement. x(t) is what we are 
% measuring and f(t) is what drives its state change.
%
% f(t) will be modeled as a white noise sequence with mean 0 and
% variance sigma^2, where sigma^2 is choosen to reflect the physical
% properties of the object being tracked.
%
% OBS NOTE: sigma is set to a value below. And this value is the only thing
% that you will tamper with. You will have a trade off:
%
%   A lower value will generate a smooth curve, but will also 
%   generate lag and a damping effect.
%
%   A higher value will be more responsive but will not filter noise
%   as well.
%
% Play around with sigma using the simulated data (which you can
% also tamper with of course) to find a suiteable value.
%

%% Simulate exact data and add noise
%
% Note that we now add normally distributed numbers to simulate noise.
% The parameter "d" will determine the amplitude of the noise.
%
clear all

d = 0.9;

interval_duration = 24*20;   % hours

number_of_steps = interval_duration*60/5;    % 5 minute resolution

h = interval_duration/number_of_steps;    % time discretization step length

x_simulate_function = @(k) (2*cos(pi*k/12)-0.9*cos(pi*k/7)+1);

K = 0:h:(interval_duration-h);    % simulation interval

x_simulated = x_simulate_function(K)';

measure_points = x_simulated + d * randn(1, length(x_simulated))';   % add noise

plot(K, measure_points, 'g')
hold on
plot(K, x_simulated, 'r')
%% Initiate system parameters
%
% OBS: The only parameters that you should change are sigma and
% maybe the initial state x_0. x_0 however will not affect convergence,
% and can therefore be set arbitraty.
%
% The system models a one dimensional movement in discrete time with
% step length h (e.g. temperature over time). The system is derived in the
% Mathematical Systems Theory course notes Example 2.3.1
%

sigma = 7;

x_0 = [0;0];
A = [1 h;0 1];
B = [h^2/2, h]*sigma;
C = [1 0];
D = sqrt(d);
Q = C'*C;
R = 1;
P_0 = eye(2);

%% Discrete Time Riccati Equation

DTRE_1 = @(P_k) A*P_k*(A') + B*Q*(B');

DTRE_2 = @(P_k) A*P_k*(C')*(inv(C*P_k*(C') + D*R*(D')))*C*P_k*(A');

DTRE = @(P_k) DTRE_1(P_k) - DTRE_2(P_k);

%% Kalman Gain

kalman_gain = @(P_k) P_k*C'*(inv(C*P_k*C' + D*R*D'));

%% Recursive formula for state approximation

state_update = @(K_k, x_k, y_k) (A - A*K_k*C)*x_k + A*K_k*y_k;

P_k = P_0;
x_k = x_0;
state = zeros(2, length(measure_points));   % allocate memory for state data
for i = 1:length(measure_points)
    P_k = DTRE(P_k);
    K_k = kalman_gain(P_k);
    x_k = state_update(K_k, x_k, measure_points(i));
    state(:,i) = x_k;
end

%% Plot
% exact in blue, noisy data in green and estimate in red.
%

plot(K, measure_points, 'g')
hold on
plot(K, x_simulated, 'b')
plot(K, state(1,:), 'r')
