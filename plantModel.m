function [xdot, u_next] = plantModel(t, x_curr, x_ref, u_curr, constants)
    %% system dynamics for this time-step
    % function to calculate A and B matrices based on the current state
    [A, B] = returnLinearisedPlant(x_curr, u_curr, constants);

    %% lqr config
    % fairly a dummy matrix, could use some tune up
    Q = [0.5, 0, 0, 0, 0;           % higher penalty on convergence \
         0, 0.5, 0, 0, 0;           % of position and \
         0, 0, 0.5, 0, 0;           % orientation variables.
         0, 0, 0, 0.05, 0;          % do not want velocity to converge too quickly
         0, 0, 0, 0, 0.1];

    R = [1, 0;
         0, 1];

    N = zeros(5, 2);

    [K, ~, ~] = lqr(A, B, Q, R, N);

    u_next = K * (x_ref - x_curr);

    xdot = A*x_curr + B*u_next;
end