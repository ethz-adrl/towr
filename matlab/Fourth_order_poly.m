clc;
clear all;

% 4rd order poly
syms a b c d e t p0 p1 v0 v1 a0 T
p   = a + b*t + c*t^2 + d*t^3 + e*t^4;
pv  = diff(p, t);
pa  = diff(p, t, 2);


% initial position and velocity are zero
t = 0;
eq_init_p = subs(p)  == p0
eq_init_v = subs(pv) == v0
eq_init_a = subs(pa) == a0


% final position and velocity are zero (at time t=1)
t = T;
eq_final_p = subs(p) == p1
eq_final_v = subs(pv) == v1


% solve the system of equations for the polynomial coefficients
S = solve([eq_init_p, eq_init_v, eq_init_a, eq_final_p, eq_final_v], [a,b,c,d,e]);
      
a = S.a
b = S.b
c = S.c
d = S.d
e = S.e

syms t;
pretty(subs(p))


% Get the derivative w.r.t the initial and final position
jac_p = jacobian(subs(p), [p0 p1])