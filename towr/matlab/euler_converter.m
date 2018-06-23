%% Derivatives of a Euler->Angular velocity mappings w.r.t euler angles.
%
% Used in class EulerConverter to specifcy analytical derivatives.
%
% Author: Alexander Winkler

clc;
clear all;

syms t u % time and polynomial coefficients defining euler angles
syms z(t,u) y(t,u) x(t,u) %euler angles yaw, pitch, roll 

% matrix that maps euler rates to angular velocities.
% see https://docs.leggedrobotics.com/kindr/cheatsheet_latest.pdf
M = [0, -sin(z),  cos(y)*cos(z);
     0,  cos(z),  cos(y)*sin(z);
     1,       0,         -sin(y)]
 
dM_du = diff(M,u)


 
%% Derivative of M_dot

% Derivative of M w.r.t time
Md = diff(M,t)

% Derivative of M_dot w.r.t polynomial coefficients u defining euler angles
dMd_du = diff(Md,u)

 

%% derivative of rotation matrix defined by euler angles
% from kindr cheet sheet, using convention zyx
% see https://docs.leggedrobotics.com/kindr/cheatsheet_latest.pdf

R = [cos(y)*cos(z), cos(z)*sin(x)*sin(y) - cos(x)*sin(z), sin(x)*sin(z) + cos(x)*cos(z)*sin(y);
     cos(y)*sin(z), cos(x)*cos(z) + sin(x)*sin(y)*sin(z), cos(x)*sin(y)*sin(z) - cos(z)*sin(x);
           -sin(y),                        cos(y)*sin(x), cos(x)*cos(y)]
         
       
dR_du = diff(R,u)















 
     