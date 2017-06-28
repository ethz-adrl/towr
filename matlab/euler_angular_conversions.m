clc;
clear all;

%syms z y x; %euler angles
syms t u

syms z(t,u) y(t,u) x(t,u)

M = [0, -sin(z),  cos(y)*cos(z);
     0,  cos(z),  cos(y)*sin(z);
     1,       0, -sin(y)  ]
 
dM_du = diff(M,u)


%z = sym('z(t)') % yaw angle
%y = sym('y(t)') % pitch angle
%x = sym('x(t)') % roll angle

% 4rd order poly
%syms a b c d e

%z = @(t)3*t;
%z(t) = a + b*t + c*t^2 + d*t^3 + e*t^4;
%zv  = diff(z, t);
%za  = diff(z, t, 2);


%M(0)


 
%% find derivative of M_dot
 
Md = diff(M,t)

dMd_du = diff(Md,u)

% syms a b
% y = z + b
% subs(z(t), 3);
% subs(y)
%subs(a + b, a, 4)


% angular velocities
% z(t) = 3;
% subs(z,0);
% subs(y,0);
% subs(x,0);
% subs(M);
% % z = 0;
% % y = 0;
% % x = 0;

% zd = 1;
% yd = 1;
% xd = 1;
% w = M*[zd;yd;xd]
 
%% derivative of rotation matrix
% from kindr cheet sheet, using convention zyx

R = [cos(y)*cos(z), cos(z)*sin(x)*sin(y) - cos(x)*sin(z), sin(x)*sin(z) + cos(x)*cos(z)*sin(y);
     cos(y)*sin(z), cos(x)*cos(z) + sin(x)*sin(y)*sin(z), cos(x)*sin(y)*sin(z) - cos(z)*sin(x);
           -sin(y),                        cos(y)*sin(x), cos(x)*cos(y)]
       
       
dR_du = diff(R,u)

















 
     