clc;
clear all;

% Builds the polynomial for the swingleg trajectory

% say we have a 5th order polynomial
syms t a b c d e f t_step h z_start z_end
zp = a*t^5 + b*t^4 + c*t^3 + d*t^2 + e*t + f;
zv = diff(zp, t, 1)
za = diff(zp, t, 2)

%t_max = 1;

% initial position and velocity are zero
t = 0;
eq_init_p = subs(zp) == z_start
eq_init_v = subs(zv) == 0

% % the intermediate height is at 1 and velocity zero
% fprintf('constraints wrt [a,b,c,d]:\n');
% t = t_step/2;
% eq_mid_p = subs(zp) == (z_start+z_end)/2 + h %average between footholds raised by 2
% eq_mid_v = subs(zv) == 0

% the intermediate height is at 1 and velocity zero
fprintf('constraints wrt [a,b,c,d]:\n');
syms n;
t = t_step*1/n;
eq_mid_p = subs(zp) == z_start+ h %initial foothold raised by h

t = t_step*(n-1)/n;
eq_mid_v = subs(zp) == z_end + h %final foothold raised by h



% final position and velocity are zero (at time t=1)
t = t_step;
eq_final_p = subs(zp) == z_end
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


% now get the derivatives w.r.t. the start and end point
jac_x = jacobian(zp, [z_start z_end])









syms t
h      = 0.03; %m
n = 10; % reach hight after 1/5 and at 4/5

t_step = 0.4; %s
z_start = 0.3; %m
z_end = 0.3; %m

% CAREFUL: This is copied, replace when changing the top code
subs(zp)
%jo(t) = (16*h*t^2)/t_step^2 - (32*h*t^3)/t_step^3 + (16*h*t^4)/t_step^4;
jo(t) = - (2*(2*n^2*z_end - 3*n^3*z_end - 2*n^2*z_start + 3*n^3*z_start)*t^5)/(t_step^5*(n - 2)*(n^2 - 2*n + 1)) - ((2*h*n^4 - h*n^5 - 10*n^2*z_end + 15*n^3*z_end + 10*n^2*z_start - 15*n^3*z_start)*t^4)/(t_step^4*(n - 2)*(n^2 - 2*n + 1)) + (2*(2*z_end - 2*z_start - 5*n*z_end + 5*n*z_start + 2*h*n^4 - h*n^5 + 5*n^3*z_end - 5*n^3*z_start)*t^3)/((n - 2)*(n^2*t_step^3 - 2*n*t_step^3 + t_step^3)) + ((6*z_end - 6*z_start - 15*n*z_end + 15*n*z_start + 2*h*n^4 - h*n^5 + 10*n^2*z_end - 10*n^2*z_start)*t^2)/(- n^3*t_step^2 + 4*n^2*t_step^2 - 5*n*t_step^2 + 2*t_step^2) + z_start;
fplot(jo, [0,t_step]);

% plot(t,y(:,1),'-o',t,y(:,2),'-o')
% title('Pendulum falling motion');
% xlabel('Time t');
% ylabel('Position/Velocity of CoM');
% legend('position', 'velocity');



