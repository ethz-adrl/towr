clc;
clear all;
close all;

syms th(t) t h 
c   = h*tan(th);
cd  = diff(c, t);
cdd = diff(c, t, 2);


disp(simplify(c));
disp(simplify(cd));
disp(simplify(cdd));

syms x
f = 1/(1+(tan(x))^2)^2 * tan(x);
S = simplify(f)
pretty(S)
S = simplify(f, 'IgnoreAnalyticConstraints', true, 'Steps', 50)



syms y
g = sin(y)*cos(y)*(1+(tan(y))^2);
simplify(g)
G = simplify(g, 'IgnoreAnalyticConstraints', true, 'Steps', 50)
pretty(G)



%%
clc
clear all
close all
syms c clear
syms p clear
syms g clear
syms h clear
syms cd clear
cdd  = 2*h/(h^2 + (c-p)^2) * cd^2   + g
% solve an equation
eqn = cdd == 0;
P = solve(eqn,p)
pretty(P)



