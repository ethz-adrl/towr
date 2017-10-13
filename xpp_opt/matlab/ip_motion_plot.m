% plot IP motion
clc;
close all;
clear all;


g = 9.81;
h = 0.58;
a = sqrt(g/h); 

c0 = 0;
cd0 = 1;


% capture point
u = c0 + cd0/a



b1 = 1/2*(c0 + cd0/a - u)
b2 = 1/2*(c0 - cd0/a - u)



t = [0:0.01:5];


c = b1*exp(a*t) + b2*exp(-a*t) + u;
cd = a*b1*exp(a*t) - a*b2*exp(-a*t); 


plot(t,c);
hold on;
plot(t,cd);

%plot(t,log(t))
