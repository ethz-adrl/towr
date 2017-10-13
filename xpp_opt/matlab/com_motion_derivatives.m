% com motion derivatives

clc;
clear all;
close all;

syms a b c d e f t
x   = a*t^5 + b*t^4 + c*t^3 + d*t^2 + e*t + f;
xd  = diff(x, t);
xdd = diff(x, t, 2);

%disp(simplify(x));
%disp(simplify(xd));
%disp(simplify(xdd));


% now jacobian of these functions
jac_x = jacobian(x, [a b c d e f]);
jac_xd = jacobian(xd, [a b c d e f]);
jac_xdd = jacobian(xdd, [a b c d e f]);



% now jacobian of combined functions
xd2 = xd^2;
jac_xd2 = jacobian(xd^2, [a b c d e f]);
fprintf('jacobian xd^2 wrt [a,b,c,d,e,f]:\n');
disp(simplify(jac_xd2));


jac_xd2 = jacobian(x*xd^2, [a b c d e f]);
fprintf('jacobian x*xd^2 wrt [a,b,c,d,e,f]:\n');
disp(simplify(jac_xd2));



%% now using the real function
syms p h g
cx = (x-p)/h * (2*h/(h^2 + (x-p)^2) * xd^2   + g);
disp(simplify(cx, 'IgnoreAnalyticConstraints', true, 'Steps', 50));

jac_cx = jacobian(cx, [a b c d e f]);
disp(simplify(jac_cx, 'IgnoreAnalyticConstraints', true, 'Steps', 50));
