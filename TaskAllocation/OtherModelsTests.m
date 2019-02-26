%% Holonomic Single integrator
tspan = [0 5];
p0 = [0,0]; % x and y
u = [1,2];
[~,p] = ode45(@(t,p) [u(1);u(2)], tspan, p0);
comet(p(:,1),p(:,2))
%% bicicle
tspan = [0 5];
p0 = [0, 0, pi/4]; % x, y and theta
u = [1,pi/2];
[~,p] = ode45(@(t,p) [u(1)*cos(p(3));u(1)*sin(p(3));u(2)], tspan, p0);
comet(p(:,1),p(:,2))