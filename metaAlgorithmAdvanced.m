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
% initial state if circle then x y 9 and 5 if sine then 0 and 0
x_G = 0;
y_G = 0;
phi = pi/4;
v_xg = 0.05;            % giving it velocity inital velocity so it can move out of the state
psi = 0;                % pi/10;

% initial conrol
tau_d = 0;
u_s = 0;

constants = [b, D, T, m, r, J, K];
initState = [x_G, y_G, phi, v_xg, psi]';
initControl = [tau_d, u_s]';

% lqr config 
% Q = [2.8, 0, 0, 0, 0;           % higher penalty on convergence \
%       0, 2.8, 0, 0, 0;           % of position and \
%       0, 0, 0.29, 0, 0;           % orientation variables.
%       0, 0, 0, 0.01, 0;          % do not want velocity to converge too quickly
%       0, 0, 0, 0, 0.01];

Q = [16, 0, 0, 0, 0;           % higher penalty on convergence \
     0, 16, 0, 0, 0;           % of position and \
     0, 0, 0.29, 0, 0;           % orientation variables.
     0, 0, 0, 0.05, 0;          % do not want velocity to converge too quickly
     0, 0, 0, 0, 0.1];

% Q = [0.4, 0, 0, 0, 0;           % higher penalty on convergence \
%      0, 0.4, 0, 0, 0;           % of position and \
%      0, 0, 0.4, 0, 0;           % orientation variables.
%      0, 0, 0, 0.05, 0;          % do not want velocity to converge too quickly
%      0, 0, 0, 0, 0.1];

% Q = Q / 2;

R = [4*0.5 0;
     0, 0.1];

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

refHistoryX = zeros(length(simulation), 1);
refHistoryY = zeros(length(simulation), 1);
refHistory = zeros(length(simulation), 2);

% only to have a moving sine reference state
movingSineRef = x_curr;

for i = 1:length(simulation)
    % linearize system at the current state and control
    [A, B] = returnLinearisedPlant(x_curr, u_curr, constants);

    % find appropriate control for this ss model
    [K, ~, ~] = lqr(A, B, Q, R, N);

    % sine reference function to track
    x_ref = returnSineReference(movingSineRef, h);
    refHistory(i, :) = x_ref(1:2)';
    refHistoryX(i) = x_ref(1);
    refHistoryY(i) = x_ref(2);
    movingSineRef = x_ref;
    
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
%{
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
    legend('x_G', 'y_G', 'phi', 'v_xg', 'psi')

    % plot the reference circle to visualise
    x_ref = returnCircleReference(x_curr, (i-1)*h); % run within a loop
    refHistoryX(i) = x_ref(1);
    refHistoryY(i) = x_ref(2);
    plot(refHistoryX, refHistoryY)
    
    % plot reference sine wave to visualise; needs to be done in a loop
    x_ref = returnSineReference(x_curr, h);
    refHistoryX(i) = x_ref(1);
    refHistoryY(i) = x_ref(2);
    x_curr = x_ref;
    plot(refHistoryX, refHistoryY)
%}
    
% plot only position states
figure(1)
plot(refHistoryX, refHistoryY, 'LineWidth', 2)
hold on
plot(refHistory(:, 1), refHistory(:, 2), 'LineWidth', 2)
plot(stateHistory(:, 1), stateHistory(:, 2), 'LineWidth', 2)
hold off
legend('Reference path', 'Ref2', 'Path taken')
% 
% figure(2)
% plot()

%% functions
function refState = returnSineReference(currentState, time)
    x0 = currentState(1);
    refState = zeros(size(currentState));
    scale = 1;
    freq = 1;
    % the new coorinates for reference are (x, sin(x)). the x is offset by time
    refState(1) = x0 + time;
    refState(2) = scale * sin(freq * refState(1));

    % random constant orientation, velocity and psi
    refState(3) = pi/4;
    refState(4) = 0.05;
    refState(5) = 0;
end

function refState = videoReference(currentState, obsDistaceToObj, pixelDistToCentre)
    % the ball is at a distance obsDistaceToObj away from the currentState
    % the ball is assumed to be approximately in the same direction \
    % as where the car is facing at the moment, i.e. currentState(3)
    % obsDistaceToObj should be in meters

    phi = currentState(3);
    
    refState = zeros(size(currentState));
    
    refState(1) = currentState(1) + obsDistaceToObj*cos(phi);
    refState(2) = currentState(2) + obsDistaceToObj*sin(phi);

    % here should be the mechanism to find what the new orientation angle should be \
    % based on the current orientation plus a little addition or subtraction of the value.
    
    % this is because we have already assumed that the set up is such that the ball is not \
    % too far away from the vertical center of the frame. 

    % ideally this calculation would be based on how many pixels away from vertical center is the ball.
    % then convert that into an approximate distance. this would create a triangle between:
    %  - the center of ball; location of camera; a point on the vertical center line
    % but, we just add or subtract 
    ratioBwPixelAndDist = (3.55/100) / 155;
    meterDistToCentre = pixelDistToCentre * ratioBwPixelAndDist;
    orientationChange = asin(meterDistToCentre / obsDistaceToObj);
    refState(3) = phi + orientationChange;

    % here could be a mechanism to decide on what the velocity should be
    refState(4) = 0.05;
    refState(5) = 0;

end