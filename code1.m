function main

% Initial conditions
p0 = -70;
a0 = 20;
b0 = 0;

% Target conditions
pt = 9;
at = 5;
bt = 0;

% Time interval
T = 5;
dt = 0.01;
t = 0:dt:T;

% Control parameters
kp = 3;
ka = 8;
kb = -1.5;

% ODE solution
x0 = [p0, a0, b0];
[t, x] = ode45(@robot_dynamics, t, x0);
p = x(:,1);
a = x(:,2);
b = x(:,3);

% Plot results
figure
plot(p,a)
xlabel('p')
ylabel('a')
title('Mobile Robot Trajectory')

% ODE function

function xdot = robot_dynamics(t, x)
p = x(1);
a = x(2);
b = x(3);
v = kp * (pt - p);
w = ka * (at - a) + kb * b;
xdot = [v; w; 0];
end

end
