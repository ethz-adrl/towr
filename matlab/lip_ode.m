function [ dydt ] = lip_ode( t, y )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

h = 0.58;
g = 9.81;
p = 0.2;

c  = y(1);
cd = y(2);

% how do i get this analytically?
p = 0.2265;


dydt  = [cd;
        (c-p)/h * (2*h/(h^2 + (c-p)^2) * cd^2   + g)];
    





end

