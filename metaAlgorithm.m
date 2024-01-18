%% reset
clc;
close all;
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
v_xg = 0.05;            % giving it velocity inital velocity so it can move out of the state
psi = 0;                % pi/10;

% initial conrol
tau_d = 0;
u_s = 0;

constants = [b, D, T, m, r, J, K];
initState = [x_G, y_G, phi, v_xg, psi]';
initControl = [tau_d, u_s]';

% refState = [2, 2, pi/4, 0.05, 0.01]';

% lqr config 
% fairly a dummy matrix, could use some tune up
Q = [0.4, 0, 0, 0, 0;           % higher penalty on convergence \
     0, 0.4, 0, 0, 0;           % of position and \
     0, 0, 0.4, 0, 0;           % orientation variables.
     0, 0, 0, 0.01, 0;          % do not want velocity to converge too quickly
     0, 0, 0, 0, 0.01];

Q = Q / 10;

R = [0.5 0;
     0, 0.5];

N = zeros(5, 2);

%% simulating the system
% iteration counter: unused atm
k = 0;

% sampling time: this is the time steps the controller runs at?
h = 0.1;

% total simulation time
totalSimTime = 10;

simulation = k:h:totalSimTime;

% Initial state and control
x_curr = initState;
u_curr = initControl;

% array to save trajectory history: check if sizes are correct? a column vector being added as a row vector
stateHistory = zeros(length(simulation), length(x_curr));
controlHistory = zeros(length(simulation), length(u_curr));
timeHistory = zeros(length(simulation), 1);

for i = 1:length(simulation)

    % linearize system at the current state and control
    [A, B] = returnLinearisedPlant(x_curr, u_curr, constants);

    % find appropriate control for this ss model
    [K, ~, ~] = lqr(A, B, Q, R, N);

    % dummy function; at the moment only returns a static target state
    x_ref = returnNewReference();
    
    % TODO: confirm matrix sizes are correct
    % compute new control
    % UPDATE: new control to be computed inside plant model
    % u_new = K * (x_ref - x_curr);

    % integrate xdot on the time interval [kh, (k+1)h) \
    % using the fixed controller K
    tspan = [(i-1)*h (i)*h];
    [t_sim, x_sim] = ode45(@(t, x) plantModel(t, x, x_ref, K, constants), tspan, x_curr);

    % update for next iteration
    x_curr = x_sim(end, :)';
    u_curr = K * (x_ref - x_curr);

    stateHistory(i, :) = x_curr';
    controlHistory(i, :) = u_curr';
    timeHistory(i) = i;
end


%% plot
% plot all states
figure
plot(timeHistory, stateHistory(:, 1))
hold on
plot(timeHistory, stateHistory(:, 2))
plot(timeHistory, stateHistory(:, 3))
plot(timeHistory, stateHistory(:, 4))
plot(timeHistory, stateHistory(:, 5))
hold off

legend('x_G', 'y_G', 'phi', 'v_xg', 'psi')

% plot only position states
figure
plot(stateHistory(:,1), stateHistory(:, 2))


%{
while k*h <= totalSimTime
    % linearize system at the current state and control
    [A, B] = returnLinearisedPlant(x_curr, u_curr, constants);

    % find appropriate control for this ss model
    [K, ~, ~] = lqr(A, B, Q, R, N);

    % dummy function; at the moment only returns a static target state
    x_ref = returnNewReference();
    
    % TODO: confirm matrix sizes are correct
    % compute new control
    % UPDATE: new control to be computed inside plant model
    % u_new = K * (x_ref - x_curr);

    % integrate xdot on the time interval [kh, (k+1)h) \
    % using the fixed controller K
    tspan = [k*h (k+1)*h];
    [t_sim, x_sim] = ode45(@(t, x) plantModel(t, x, x_ref, K, constants), tspan, x_curr);

    % update for next iteration
    x_curr = x_sim(end, :)';
    u_curr = K * (x_ref - x_curr);
    k = k + 1;
end
%}

function refState = returnNewReference()
    % this funciton would be replaced by the part that returns a new reference from camera images
    refState = [1, 6, pi/4, 0.05, 0]';               %put the psi to 0 maybe
end