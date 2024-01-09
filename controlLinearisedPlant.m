

% Set simulation parameters
t_span = 0:0.1:10; % Time span for simulation
x0 = ...; % Initial state

% Implement the feedback control law
u_func = @(t, x) -K * (x - x_ref);

% Simulate the closed-loop system
[t, x] = ode45(@(t, x) (A - B * K) * x, t_span, x0);

% Plot results
plot(t, x);
xlabel('Time');
ylabel('State Variables');
legend('x1', 'x2', 'x3', 'x4', 'x5');
