clc;
clear all

% this is for integration
syms c(t) cd(t) p h g T
cdd  = (c-p)/h * (2*h/(h^2 + (c-p)^2) * cd^2   + g)

% integrate this from 0 to t
int(T^2,T, 0, 3)


dydt = lip_ode(0.0, [0;1])


y0 = [0;1]; % initial position and velocity


% calculate capture point
p_capture_pratt = y0(2) * sqrt(0.58/9.81)
dt = [0 1.0];
[t,y] = ode45(@lip_ode,dt,y0);

plot(t,y(:,1),'-o',t,y(:,2),'-o')
title('Pendulum falling motion');
xlabel('Time t');
ylabel('Position/Velocity of CoM');
legend('position', 'velocity');


%% Solving a boundary value problem to determine capture point
solinit = bvpinit([0:0.01:0.5],[0 1], 0.2);
solinit
sol = bvp4c(@lip_ode,@bc_pendulum, solinit)

sol.parameters


