clc;
clear all;

% 3rd order poly
syms x a b c

z   = a*x^2 + b*x + c;
%dz  = diff(z, x);
%dzz = diff(z, x, 2);

syms h w xc
% initial position and velocity are zero
x = xc;
height_center = subs(z) == -h;


% position at endge of gap should be zero
x = xc-w/2;
height_start = subs(z) == 0;
x = xc+w/2;
height_end = subs(z) == 0;


% solve the system of equations for the polynomial coefficients
S = solve([height_center, height_start, height_end], [a,b,c]);
      
a = S.a
b = S.b
c = S.c

syms x;
pretty(subs(z))
subs(z)

%%
clc;
clear all;

h  = 5.0;
w  = 0.25;
xc = 1.2;
x = [0.9:0.1:1.5];

% CAREFUL: copied from top solution
height = (4*h*x.^2)/w^2 - (8*h*x*xc)/w^2 - (h*(w - 2*xc)*(w + 2*xc))/w^2;

plot(x, height);