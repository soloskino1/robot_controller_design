function main

% Initial conditions
p0 = [-50; 60; 70; -80; -45; -75];
a0 = [-35; 25; 15; -5; -30; -20];
b0 = zeros(6,1);

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
x0 = [p0; a0; b0];
[t, x] = ode45(@robot_dynamics, t, x0);
p = x(:,1:6);
a = x(:,7:12);
b = x(:,13:18);   

% Plot results
figure
plot(p(:,1),a(:,1),'r',p(:,2),a(:,2),'b',p(:,3),a(:,3),'g',p(:,4),a(:,4),'k',p(:,5),a(:,5),'m',p(:,6),a(:,6),'c')
xlabel('p')
ylabel('a')
title('Trajectories of Mobile Robots')
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
