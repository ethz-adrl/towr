%% Derivatives of a 5th-order polynomial with respect to the coefficients
% Author: Alexander Winkler

clc;
clear all;
close all;

syms a b c d e f t
x   = a*t^5 + b*t^4 + c*t^3 + d*t^2 + e*t + f;
xd  = diff(x, t);
xdd = diff(x, t, 2);

% Jacobian of these functions
jac_x = jacobian(x, [a b c d e f]);
jac_xd = jacobian(xd, [a b c d e f]);
jac_xdd = jacobian(xdd, [a b c d e f]);

% Jacobian of combined functions
xd2 = xd^2;
jac_xd2 = jacobian(xd^2, [a b c d e f]);
fprintf('jacobian xd^2 wrt [a,b,c,d,e,f]:\n');
disp(simplify(jac_xd2));

jac_xd2 = jacobian(x*xd^2, [a b c d e f]);
fprintf('jacobian x*xd^2 wrt [a,b,c,d,e,f]:\n');
disp(simplify(jac_xd2));
