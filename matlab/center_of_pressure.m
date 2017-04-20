%% Center of Pressure derivatives
%
% This file calculates the dependency between the CoP
% and the load of the endeffectors

clc;
clear all;
close all;

syms l1 l2 l3 l4 p1 p2 p3 p4
cop   = (l1*p1 + l2*p2 + l3*p3 + l4*p4)/(l1+l2+l3+l4);

cop_wrt_load  = diff(cop, l1)
cop_wrt_ee  = diff(cop, p1)