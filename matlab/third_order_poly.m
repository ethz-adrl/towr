clc;
clear all;

% 3rd order poly
syms a b c d t p0 v0 p1 v1 T
p   = d*t^3 + c*t^2 + b*t + a;
pv  = diff(p, t);
pa  = diff(p, t, 2);


% initial position and velocity are zero
t = 0;
eq_init_p = subs(p) == p0
eq_init_v = subs(pv) == v0


% final position and velocity are zero (at time t=1)
t = T;
eq_final_p = subs(p) == p1
eq_final_v = subs(pv) == v1


% solve the system of equations for the polynomial coefficients
S = solve([eq_init_p, eq_init_v, eq_final_p, eq_final_v], [a,b,c,d]);
      
a = S.a
b = S.b
c = S.c
d = S.d

syms t;
pretty(subs(p))


% Get the derivative w.r.t the initial and final position
jac_p = jacobian(subs(p), [p0 v0 p1 v1 T])

dp_dp0 = jac_p(1)
dp_dv0 = jac_p(2)
dp_dp1 = jac_p(3)
dp_dv1 = jac_p(4)
dp_dT = jac_p(5)

% Get the derivative w.r.t the initial and final position
jac_v = jacobian(subs(pv), [p0 v0 p1 v1 T])

dpv_dp0 = jac_v(1)
dpv_dv0 = jac_v(2)
dpv_dp1 = jac_v(3)
dpv_dv1 = jac_v(4)
dpv_dT  = jac_v(5)


jac_a = jacobian(subs(pa), [p0 v0 p1 v1 T])
dpa_dp0 = jac_a(1)
dpa_dv0 = jac_a(2)
dpa_dp1 = jac_a(3)
dpa_dv1 = jac_a(4)
dpa_dT = jac_a(5)