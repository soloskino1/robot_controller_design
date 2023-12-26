function main

% Initial conditions for 7 robots
p0 = [-50, 60, 70, -80, -45, -75];
a0 = [-35, 25, 15, -5, -30, -20];
b0 = zeros(1,6);

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

% ODE solution for 7 robots
x0 = [p0; a0; b0];
[t, x] = ode45(@robot_dynamics, t, x0);
p = x(:,1:6);
a = x(:,7:12);
b = x(:,13:18);

% Plot results
figure
for i = 1:6
    plot(p(:,i),a(:,i))
    hold on
end
xlabel('p')
ylabel('a')
title('Mobile Robot Trajectories')
legend('Robot 1', 'Robot 2', 'Robot 3', 'Robot 4', 'Robot 5', 'Robot 6')

% ODE function
function xdot = robot_dynamics(t, x)
p = x(1:6);
a = x(7:12);
b = x(13:18);
v = kp * (pt - p);
w = ka * (at - a) + kb * b;
xdot = [v; w; zeros(6,1)];
end


end
