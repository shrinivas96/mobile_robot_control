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
% initial state
x_G = 0;
y_G = 0;
phi = pi/6;
v_xg = 0.05;           % giving it velocity inital velocity so it can move out of the state
psi = pi/10;

% initial conrol
tau_d = 0;
u_s = 0;

constants = [b, D, T, m, r, J, K];
initState = [x_G, y_G, phi, v_xg, psi]';
initControl = [tau_d, u_s]';

refState = [2, 3, pi/4, 0.05, 0.01]';

% Initial state and control
x0 = initState;
u0 = initControl;

%% simulaton set up
tspan = [0:0.01:10];
tspan = [0:0.01:10];


[t, x] = ode45(@(t, x) plantModel(t, x, refState, initControl, constants), tspan, initState);
% [t, x, u] = ode45(@plantModel, tspan, refState, initState, initControl, constants);
% plot(t, x);

%% Plot results
figure
plot(t, x(:,1))
hold on
plot(t, x(:, 2))
plot(t, x(:, 3))
plot(t, x(:, 4))
plot(t, x(:, 5))
hold off

legend('x_G', 'y_G', 'phi', 'v_xg', 'psi')

figure
plot(x(:, 1), x(:, 2))

%% second simulate
% newInitState = x(end, :);
% secondRefState = [6, 6, pi/4, 0.05, 0.01]';
% 
% [t2, x2] = ode45(@(t, x) plantModel(t, x, secondRefState, initControl, constants), tspan, newInitState);
% 
% 
% %% Plot results
% figure
% plot(t2, x2(:,1))
% hold on
% plot(t2, x2(:, 2))
% plot(t2, x2(:, 3))
% plot(t2, x2(:, 4))
% plot(t2, x2(:, 5))
% hold off
% 
% legend('x_G', 'y_G', 'phi', 'v_xg', 'psi')