clc; clear all;

% defining constants for workspace
b = 0.3/2;
D = 2*b; 
T = 0.01;    % Time constant of the steering system
m = 0.5;    % mass of the vehicle
r = 0.05;   % Rear wheel radius
J = 0.03;   % mass moment of intertia TBD
K = 1;      % Constant coefficient 

x_G = 0;
y_G = 0;
phi = pi/6;
v_xg = 0.05;
psi = pi/10;

tau_d = 0;
u_s = 0;

constants = [b, D, T, m, r, J, K];
state = [x_G, y_G, phi, v_xg, psi];
control = [tau_d, u_s];

[A, B] = returnLinearisedPlant(state, control, constants);


% computing controllability matrix for kalman rank condition
controllablilityMatrix = ctrb(A, B);
sysRank = rank(controllablilityMatrix);

% Observation matrix assuming we have all the variables (y = cx)
C = eye(5); 

% Feed forward matrix 
Dff = 0;

% original system in state space and tf form
plant_ss = ss(A, B, C, Dff);
plant_tf = tf(plant_ss);

Q = [0.1, 0, 0, 0, 0;
     0, 0.1, 0, 0, 0;
     0, 0, 0.1, 0, 0;
     0, 0, 0, 0.01, 0;
     0, 0, 0, 0, 0.1];
R = eye(2);

N = zeros(5, 2);

[K, ~, ~] = lqr(plant_ss, Q, R, N);