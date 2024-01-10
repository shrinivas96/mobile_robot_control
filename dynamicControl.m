%% reset
clc; 
clearvars;

%% constants config

% defining constants for workspace
b = 0.3/2;
D = 2*b; 
T = 0.01;    % Time constant of the steering system
m = 0.5;    % mass of the vehicle
r = 0.05;   % Rear wheel radius
J = 0.03;   % mass moment of intertia TBD
K = 1;      % Constant coefficient 

%% state and control config 
x_G = 0;
y_G = 0;
phi = pi/6;
v_xg = 0.05;
psi = pi/10;

tau_d = 0;
u_s = 0;

constants = [b, D, T, m, r, J, K];
initState = [x_G, y_G, phi, v_xg, psi];
initControl = [tau_d, u_s];

% Initial state
x0 = initState;
u0 = initControl;

%% lqr config
% fairly a dummy matrix, could use some tune up
Q = [0.5, 0, 0, 0, 0;           % higher penalty on convergence \
     0, 0.5, 0, 0, 0;           % of position and \
     0, 0, 0.5, 0, 0;           % orientation variables.
     0, 0, 0, 0.01, 0;          % do not want velocity to converge too quickly
     0, 0, 0, 0, 0.1];

R = [1, 0;
     0, 1];

N = zeros(5, 2);

%% simulation config 
% Set simulation parameters
t_span = 0:0.1:10; % Time span for simulation

% Function to calculate A and B matrices based on the current state x
% calculateAB = @(x, u) returnLinearisedPlant(x, u, constants);

%% result config
% Initialize arrays to store results
t = zeros(length(t_span), 1);
x = zeros(length(t_span), length(x0));
u = zeros(length(t_span), length(u0));

% Set initial values
t(1) = t_span(1);
x(1, :) = x0;
u(1, :) = u0;

% Define reference state
x_ref = [0.5, 0.5, 0, pi/2, 0];

%% simulate system
% Simulate the closed-loop system
for i = 2:5
    % Calculate A and B based on the current state x
    [A, B] = returnLinearisedPlant(x(i-1, :), u(i-1, :), constants)

    set(carModelLSS, 'A', A);
    set(carModelLSS, 'B', B);

    [K, ~, ~] = lqr(A, B, Q, R, N);

    % Implement the feedback control law: u = -Kx
    u = K * (x_ref - x(i-1, :))';

    % Update state using Euler integration
    x_dot = (A - B * K) * x(i-1, :)' + B * K * x_ref';

    x(i, :) = x(i-1, :) + x_dot' * (t_span(i) - t_span(i-1));

    % Update time
    t(i) = t_span(i);
end

%% Plot results
% plot(t, x);
% xlabel('Time');
% ylabel('State Variables');
% legend('x1', 'x2', 'x3', 'x4', 'x5');
