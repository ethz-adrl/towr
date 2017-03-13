clc;
clear all;

% Builds the polynomial for the swingleg trajectory

% say we have a 5th order polynomial
syms t a b c d e f t_step_ z_max_
zp = a*t^5 + b*t^4 + c*t^3 + d*t^2 + e*t + f;
zv = diff(zp, t, 1)
za = diff(zp, t, 2)

%t_max = 1;

% initial position and velocity are zero
t = 0;
eq_init_p = subs(zp) == 0
eq_init_v = subs(zv) == 0

% the intermediate height is at 1 and velocity zero
fprintf('constraints wrt [a,b,c,d]:\n');
t = t_step_/2;
eq_mid_p = subs(zp) == z_max_
eq_mid_v = subs(zv) == 0

% final position and velocity are zero (at time t=1)
t = t_step_;
eq_final_p = subs(zp) == 0
eq_final_v = subs(zv) == 0


% solve the system of equations for the polynomial coefficients
S = solve([eq_init_p, eq_init_v,
           eq_mid_p,  eq_mid_v,
           eq_final_p, eq_final_v], [a,b,c,d,e,f]);
      
a = S.a
b = S.b
c = S.c
d = S.d
e = S.e
f = S.f

syms t


jo(t) = subs(zp)

fplot(jo, [0 1])
% plot(t,y(:,1),'-o',t,y(:,2),'-o')
% title('Pendulum falling motion');
% xlabel('Time t');
% ylabel('Position/Velocity of CoM');
% legend('position', 'velocity');



