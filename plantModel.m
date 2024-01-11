function [xdot, u_next] = plantModel(t, x_curr, x_ref, u_curr, constants)
	%% system dynamics for this time-step
	% function to calculate A and B matrices based on the current state
	[A, B] = returnLinearisedPlant(x_curr, u_curr, constants);

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

	[K, ~, ~] = lqr(A, B, Q, R, N);

	%% derive new control
	u_next = K * (x_ref - x_curr);

	tau_d = u_next(1);
	u_s = u_next(2);

	%% non-linear system
	% unpack required constants
	T = constants(3);    	% Time constant of the steering system
	r = constants(5);    			% Rear wheel radius

	% rear wheel driving force
	F_d = tau_d / r;
	
	phi = x_curr(3);
	v_xg = x_curr(4);
	psi = x_curr(5);
	
	% recurring term
	gamma = (cos(psi)^2) * (m*D^2 + (m*b^2 + J* tan(psi)^2));

	% non linear set of equations
	dot_x_g = (cos(phi) - (b * tan(psi) * sin(phi))/D) * v_xg;
	dot_y_g = (sin(phi) + (b * tan(psi) * cos(phi))/D) * v_xg;
	dot_phi = (tan(psi)/D) * v_xg;
	dot_psi = (K*u_s - psi)/T;																				% control input 2 is applied here
	dot_v_xg = (v_xg * (m * b^2 + J) * tan(psi) * psi_dot)/gamma + (D^2 * cos(psi)^2 * F_d)/gamma;			% control input 1 is applied here through the force term F_d

	xdot = [dot_x_g;
			dot_y_g;
			dot_phi;
			dot_v_xg;
			dot_psi];
end