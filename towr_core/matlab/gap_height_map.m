%% Modelling of a gap in the terrain by a curved polynomial.
%
% Used in file height_map_examples.h to describe a gap terrain.
%
% Author: Alexander Winkler

clc;
clear all;

% 3rd order poly in the shape of a terrain gap
syms x a b c
z   = a*x^2 + b*x + c;


syms h w xc
% initial position (= terrain height) and velocity are zero
x = xc;
height_center = subs(z) == -h;


% position at endge of gap should be zero (same height as started)
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


%% Plot the shape of the gap just to double check
clc;
clear all;

h  = 5.0;
w  = 0.25;
xc = 1.2;
x = [0.9:0.1:1.5];

% CAREFUL: copied from solution of first part
height = (4*h*x.^2)/w^2 - (8*h*x*xc)/w^2 - (h*(w - 2*xc)*(w + 2*xc))/w^2;

plot(x, height);