clear all;
close all;
clc;

% angle of the pendulum
%th    = 5/180*pi  % rad
%thd  = 0.0;        % rad/s
%thdd = 0.0;        % rad/s^2

h = 0.58;
g = 9.81;

% converting linear offset to tilt angle
c = 0.2;
p = [0.0, 0.7];
th = atan((c-p)/h)

%c = p + tan(th)*h


% converting linear velocity to angular
cd = 2; % m/s
thd = 1./(1+(tan(th)).^2) * 1/h * cd


% lower bound by ZMP constraint (balance of moments)
fprintf('acceleration neccessary to stay standing:\n');
cdd_zmp = g/h*(c-p)


% acceleration of pendulum in this position
fprintf('acceleration for pendulum dropped at this angle:\n');
cdd        = h*(1+(tan(th)).^2) .* (2*tan(th).*thd.^2 + g/h*cos(th).*sin(th))
cdd_simp   = tan(th) .* (2/h*cos(th).^2*cd.^2  +  g)
cdd_simp2  = (c-p) .* (2./(h^2 + (c-p).^2) .* cd.^2   + g/h)
cdd_simp2_a= (c-p)/h .* (2./h                .* cd.^2   + g)
cdd_approx = h*(1       +th.^2) .* (2*    th .*thd.^2 + g/h*th) % small angle approximation


%% This for plotting as a function


th = [0:0.1:pi/2];
%th_d = zeros(1,size(th,2))


figure;
plot(th, cdd); 
hold on;

plot(th, cdd_approx); 
legend('cdd','cdd_{approx}');


