function xdot = plantModel(t, x_curr, x_ref, feedbackControl, constants)
	
	% line 72 thing should happen here 

	% variable that i hand over should be K not u_curr
	
	% compute control based on the new state that ode45 passes here while simulating
	u_curr = feedbackControl * (x_ref - x_curr);

	% unpack controls
	tau_d = u_curr(1);
	u_s = u_curr(2);

	% unpack required constants
	b = constants(1);    % Distance from rear wheel axis mid point Q to Center of Gravity G 
    D = constants(2);    % Distance between rear wheel axis mid point Q and front wheel axis mid point P
    T = constants(3);    % Time constant of the steering system
    m = constants(4);    % mass of the vehicle
    r = constants(5);    % Rear wheel radius
    J = constants(6);
    K = constants(7);

	% rear wheel driving force: uses control input 1
	F_d = tau_d / r;
	
	% states that are needed
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
	dot_v_xg = (v_xg * (m * b^2 + J) * tan(psi) * dot_psi)/gamma + (D^2 * cos(psi)^2 * F_d)/gamma;			% control input 1 is applied here through the force term F_d

	xdot = [dot_x_g;
			dot_y_g;
			dot_phi;
			dot_v_xg;
			dot_psi];
end