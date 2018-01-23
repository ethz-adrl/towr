%% Provides derivatives of a third-order polynomial w.r.t. nodes
%
% These results are used in class CubicHermitePolynomial to code the
% analytical derivatives.
%
% Author: Alexander Winkler

clc;
clear all;


%% Define the polynomial through its coefficients
syms a b c d % the polynomial coefficients
syms p0 v0 p1 v1 % the initial and final position and velocity ("nodes")
syms t  % the current time 
syms T  % The total duration of the polynomial

% a third-order polynomial and it's derivatives
pos  = d*t^3 + c*t^2 + b*t + a;
vel  = diff(pos, t);
acc  = diff(pos, t, 2);


%% Express polynomial coefficients as functions of node values/duration
% This fully defines the 3rd-order polynomial and is called Hermite
% Parameterization.

% initial position and velocity are zero
t = 0;
eq_init_p = subs(pos) == p0;
eq_init_v = subs(vel) == v0;


% final position and velocity are zero (at time t=1)
t = T;
eq_final_p = subs(pos) == p1;
eq_final_v = subs(vel) == v1;


% solve the system of equations for the polynomial coefficients
S = solve([eq_init_p, eq_init_v, eq_final_p, eq_final_v], [a,b,c,d]);
      
a = S.a;
b = S.b;
c = S.c;
d = S.d;

syms t;


%% Derivative of pos, vel and acc w.r.t the node values and duration
% position
jac_p = jacobian(subs(pos), [p0 v0 p1 v1 T]);

dp_dp0 = jac_p(1)
dp_dv0 = jac_p(2)
dp_dp1 = jac_p(3)
dp_dv1 = jac_p(4)
dp_dT = jac_p(5)

% velocity
jac_v = jacobian(subs(vel), [p0 v0 p1 v1 T])

dpv_dp0 = jac_v(1)
dpv_dv0 = jac_v(2)
dpv_dp1 = jac_v(3)
dpv_dv1 = jac_v(4)
dpv_dT  = jac_v(5)

% acceleration
jac_a = jacobian(subs(acc), [p0 v0 p1 v1 T])

dpa_dp0 = jac_a(1)
dpa_dv0 = jac_a(2)
dpa_dp1 = jac_a(3)
dpa_dv1 = jac_a(4)
dpa_dT = jac_a(5)