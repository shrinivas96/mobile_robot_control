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

refState = [2, 2, pi/4, 0.05, pi/10]';


% Initial state and control
x0 = initState;
u0 = initControl;

%% self simulation for testing

tspan = [0:0.1:1];

t = zeros(length(tspan), 1);
x = zeros(length(x0), length(tspan));
u = zeros(length(u0), length(tspan));

%%
% Set initial values
t(1) = tspan(1);
x(:, 1) = x0;
u(:, 1) = u0;

%% 
for i = 2:5
%     refAndCurrStates = {refState, x(i-1, :)};
    [xdot, u_next] = plantModel(t(i-1, :), refState, x(:, i-1), u(:, i-1), constants);

    % update state array
    x(:, i) = x(:, i-1) + xdot * (t(i-1, :) - t(i, :));

    % update controls array
    u(:, i) = u_next;
    
    % update time array
    t(i) = tspan(i);
end