clc
clearvars

% All variables are in SI units
b = 0.1;    % Distance from rear wheel axis mid point Q to Center of Gravity G 
D = 2*b;    % Distance between rear wheel axis mid point Q and front wheel axis mid point P
T = 0.1;    % Time constant of the steering system
m = 0.5;    % mass of the vehicle
r = 0.05;   % Rear wheel radius
K = 1;      % Constant coefficient 

ref_state = [5; 5; pi/4; 0; 0];

% State-transition (Process matrix) matrix
% A = [0, 0, 0, 1, 0;
%      0, 0, 0, 0, 0;
%      0, 0, 0, 0, 0;
%      0, 0, 0, 0, 0;
%      0, 0, 0, 0, -1/T];

A = [0, 0, -0.01, -0.162459848116453, -0.00552786404500042; 
     0, 0, -0.00162459848116453, 1.0, 3.38484050441575e-19; 
     0, 0, 0, 1.08306565410969, 0.0368524269666695;
     0, 0, 0, 9.43207242891546, 0.625825278488772; 
     0, 0, 0, 0, -100.0];


A = [0, 0, (-sin(phi) - b*cos(phi)*tan(psi)/D)*dx_g, cos(phi) - b*sin(phi)*tan(psi)/D, -b*(tan(psi)^2 + 1)*dx_g*sin(phi)/D; 
     0, 0, (cos(phi) - b*sin(phi)*tan(psi)/D)*dx_g, sin(phi) + b*cos(phi)*tan(psi)/D, b*(tan(psi)^2 + 1)*dx_g*cos(phi)/D; 
     0, 0, 0, tan(psi)/D, (tan(psi)^2 + 1)*dx_g/D;
     0, 0, 0, (-J - b^2*m)*(K*u_s/T - psi/T)*tan(psi)/((D^2*m + (J + b^2*m)*tan(psi)^2)*cos(psi)^2), -(-J - b^2*m)*(J + b^2*m)*(K*u_s/T - psi/T)*(2*tan(psi)^2 + 2)*dx_g*tan(psi)^2/((D^2*m + (J + b^2*m)*tan(psi)^2)^2*cos(psi)^2) + (-J - b^2*m)*(K*u_s/T - psi/T)*(tan(psi)^2 + 1)*dx_g/((D^2*m + (J + b^2*m)*tan(psi)^2)*cos(psi)^2) + 2*(-J - b^2*m)*(K*u_s/T - psi/T)*dx_g*sin(psi)*tan(psi)/((D^2*m + (J + b^2*m)*tan(psi)^2)*cos(psi)^3) - (-J - b^2*m)*dx_g*tan(psi)/(T*(D^2*m + (J + b^2*m)*tan(psi)^2)*cos(psi)^2);
     0, 0, 0, 0, -1/T];

% Control matrix
% B = [0, 0;
%     0, 0;
%     0, 0;
%     1/m*r, 0;
%     0, K/T];


B = [0, 0; 0 ,0; 0, 0; 36.4705589275325, 0; 0, 100.0];

% Observation (measurement matrix) matrix assuming We have all the variables (y = cx)
C = eye(5); 

% Feed forward matrix 
Df = 0;

% eignevalues of the uncontrolled system
sysEigens = eig(A);

% computing controllability matrix for kalman rank condition
controllablilityMatrix = ctrb(A, B);
sysRank = rank(controllablilityMatrix);

% original system in state space form
plant_ss = ss(A, B, C, Df);
plant_tf = tf(plant_ss);

Q = [0.1, 0, 0, 0.01, 0;
     0, 0.1, 0, 0, 0;
     0, 0, 0.1, 0, 0;
     0.01, 0, 0, 0.01, 0;
     0, 0, 0, 0, 0.1];
R = eye(2);

N = zeros(5, 2);

[K, S, P] = lqr(plant_ss, Q, R, N)

% Augment A matrix
% A_aug = [A, zeros(size(A, 1), size(C, 1));
%          -C, zeros(size(C, 1), size(C, 1))];

% Augment B matrix
% B_aug = [B; zeros(size(C, 1), size(B, 2))];

% Augment C matrix
% C_aug = [C, zeros(size(C))];

% Augment D matrix
% D_aug = Df;

% Q = 0.1*diag([1, 1, 1, 1, 1, 1, 1, 1, 1, 1]);  % Adjust weights based on performance requirements
% R = 0.1*eye(size(B_aug, 2));  % Adjust weights based on performance requirements

% [K, S, e] = lqr(A_aug, B_aug, Q, R);






% pzplot(sys);

% system in transfer function form
% systemTF = tf(sys);

% [num, denn] = tfdata(systemTF);

% desired pole; dummy value
% pole_val = [-1;-1;-1;-1;-10];

% caluclate feedback matrix K
% K = place(A,B,pole_val);

% maybe the reason why it throws the error is because the syste is not
% completely controllabel. thus, when I give the place() functin 5 poles to
% place, it cannot do that as it cannot control the entire system


% another assumption from just the error message is that because there are
% 4 same poles exactly at 0(i.e. multiplicity of the eigenvalue 0 is 4),
% thus it could have some effect on the placement, but i am not sure how to
% resolve it.