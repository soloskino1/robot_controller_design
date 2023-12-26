function main

% Initial conditions
p0 = [-50; 60; 70; -80; -45; -75];
a0 = [-35; 25; 15; -5; -30; -20];
b0 = zeros(6,1);

% First target conditions
pt1 = [9; 9; 9; 9; 9; 9];
at1 = [5; 5; 5; 5; 5; 5];
bt1 = zeros(6,1);

% Second target conditions
pt2 = [60; -10; 50; -5; 25; -90];
at2 = [40; 20; 45; 25; -10; 5];
bt2 = zeros(6,1);

% Time interval
T = 5;
dt = 0.001;
t = 0:dt:T;

% Control parameters
kp = 3;
ka = 10;
kb = -1.5;

% ODE solution for movement from initial position to first target
x0 = [p0; a0; b0];
[t1, x1] = ode45(@robot_dynamics, t, x0, [], kp, ka, kb, pt1, at1, bt1);

p1 = x1(:,1:6);
a1 = x1(:,7:12);
b1 = x1(:,13:18); 

% ODE solution for movement from first target to second target
x0 = [pt1; at1; bt1];
[t2, x2] = ode45(@robot_dynamics, t, x0, [], kp, ka, kb, pt2, at2, bt2);

p2 = x2(:,1:6);
a2 = x2(:,7:12);
b2 = x2(:,13:18); 

% Concatenate the results
p = [p1;p2];
a = [a1;a2];
b = [b1;b2];

% Plot results
figure
plot(p(:,1),a(:,1),'r',p(:,2),a(:,2),'b',p(:,3),a(:,3),'g',p(:,4),a(:,4),'k',p(:,5),a(:,5),'m',p(:,6),a(:,6),'c')
xlabel('p')
ylabel('a')
title('Trajectories of Mobile Robots')
legend('Robot 1', 'Robot 2', 'Robot 3', 'Robot 4', 'Robot 5', 'Robot 6')


end

% ODE function
function xdot = robot_dynamics(t, x, kp, ka, kb, pt, at, bt)
p = x(1:6);
a = x(7:12);
b = x(13:18);
v = kp * (pt(1:6) - p);
% Limit the velocity of each wheel to a maximum and minimum value
max_vel = 50;
min_vel = -50;
v = min(v, max_vel);
v = max(v, min_vel);
w = ka * (at(1:6) - a) + kb * bt(1:6);
xdot = [v; w; zeros(6,1)];
end