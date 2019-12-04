% Configuration Data:
% Program:Control_1D_LQR_V2
% Author: Sarvesha Kumar Kombaiah Seetha
% Version: 2.0
% Date: 4/21/18
% Description: Solves the LQR Problem for
% X(t+1) = a X(t) + b1 u(t) in 1D.
% Set up
clear;
clc;
clf;
% Coefficient Values
a = 1.5;    % Dynamic State Transform
b = 1;    % Control Transform for X
Q = 0.1;      % Cost of output error
R = 1;      % Cost of control
N = 10;     % Number of Time Steps
% Initialize Vectors
X = zeros(1,N);     % Controlled Output/State
X(1) = 0.2;       % Initial displacement
P = zeros(1,N);     % Riccati Metric
P(1) = Q;L = zeros(1,N);     % Feedback Gain
t = zeros(1,N);     % time
t(1) =0;
% Calculations: Solving Control Problem
% Calcuating the Ricatti Metrics and the Feedback Gain
P(N) = Q;
for i = 1:1:N-1
    P(N-i) = Q + a^2*P(N-i+1) - a^2*b^2*P(N-i+1)^2/(R+b^2*P(N-i+1));
    L(N-i)= a*b*P(N-i+1)/(R+b^2*P(N-i+1));
end
% Calcuating the States
for i = 1:1:N-1t(i+1) = t(i) +1;
    % Control Input
    u(i) = -L(i) * X(i);
    % Calculated Controlled State
    X(i+1) = a*X(i) + b*u(i);
end
u(N-1)
u(N) = 0;
X(N)
% Calculations: Solve Kalman Filter
plot (t,u, t,X)
legend('u(t)', 'X(t)', 'Location', 'northwest')
axis([0,N,-0.5,0.5])