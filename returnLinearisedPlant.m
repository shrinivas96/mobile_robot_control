function [A, B] = returnLinearisedPlant(x, u, c)
    dx_g = x(3);
    phi = x(4);
    psi = x(5);

    u_s = u(2);

    b = c(1);    % Distance from rear wheel axis mid point Q to Center of Gravity G 
    D = c(2);    % Distance between rear wheel axis mid point Q and front wheel axis mid point P
    T = c(3);    % Time constant of the steering system
    m = c(4);    % mass of the vehicle
    r = c(5);    % Rear wheel radius
    J = c(6);
    K = c(7);

    A = [0, 0, (-sin(phi) - b*cos(phi)*tan(psi)/D)*dx_g, cos(phi) - b*sin(phi)*tan(psi)/D, -b*(tan(psi)^2 + 1)*dx_g*sin(phi)/D; 
         0, 0, (cos(phi) - b*sin(phi)*tan(psi)/D)*dx_g, sin(phi) + b*cos(phi)*tan(psi)/D, b*(tan(psi)^2 + 1)*dx_g*cos(phi)/D; 
         0, 0, 0, tan(psi)/D, (tan(psi)^2 + 1)*dx_g/D;
         0, 0, 0, (-J - b^2*m)*(K*u_s/T - psi/T)*tan(psi)/((D^2*m + (J + b^2*m)*tan(psi)^2)*cos(psi)^2), -(-J - b^2*m)*(J + b^2*m)*(K*u_s/T - psi/T)*(2*tan(psi)^2 + 2)*dx_g*tan(psi)^2/((D^2*m + (J + b^2*m)*tan(psi)^2)^2*cos(psi)^2) + (-J - b^2*m)*(K*u_s/T - psi/T)*(tan(psi)^2 + 1)*dx_g/((D^2*m + (J + b^2*m)*tan(psi)^2)*cos(psi)^2) + 2*(-J - b^2*m)*(K*u_s/T - psi/T)*dx_g*sin(psi)*tan(psi)/((D^2*m + (J + b^2*m)*tan(psi)^2)*cos(psi)^3) - (-J - b^2*m)*dx_g*tan(psi)/(T*(D^2*m + (J + b^2*m)*tan(psi)^2)*cos(psi)^2);
         0, 0, 0, 0, -1/T];

    B = [0, 0;
         0, 0;
         0, 0;
         D^2/(r*(D^2*m + (J + b^2*m)*tan(psi)^2)), (K*(-J - b^2*m)*dx_g*tan(psi))/(T*(D^2*m + (J + b^2*m)*tan(psi)^2)*cos(psi)^2); 
         0, K/T];
end