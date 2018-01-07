clear all;
close all;
clc;

syms x0 y0 x1 y1;

norm(x0, y0, x1, y1) = sqrt( (x0-x1)^2 + (y0-y1)^2 );
p(x0, y0, x1, y1) = (y0 - y1)/norm(x0, y0, x1, y1);
q(x0, y0, x1, y1) = (x1 - x0)/norm(x0, y0, x1, y1);
r(x0, y0, x1, y1) = (x0*y1 - x1*y0)/norm(x0, y0, x1, y1);


fprintf('derivative w.r.t x0:\n');
disp(simplify(diff(p,x0,1), 'IgnoreAnalyticConstraints', true, 'Steps', 50));
disp(simplify(diff(q,x0,1), 'IgnoreAnalyticConstraints', true, 'Steps', 50));
disp(simplify(diff(r,x0,1), 'IgnoreAnalyticConstraints', true, 'Steps', 50));

fprintf('\n\nderivative w.r.t y0:\n');
disp(simplify(diff(p,y0,1), 'IgnoreAnalyticConstraints', true, 'Steps', 50));
disp(simplify(diff(q,y0,1), 'IgnoreAnalyticConstraints', true, 'Steps', 50));
disp(simplify(diff(r,y0,1), 'IgnoreAnalyticConstraints', true, 'Steps', 50));

fprintf('\n\nderivative w.r.t x1:\n');
disp(simplify(diff(p,x1,1), 'IgnoreAnalyticConstraints', true, 'Steps', 50));
disp(simplify(diff(q,x1,1), 'IgnoreAnalyticConstraints', true, 'Steps', 50));
disp(simplify(diff(r,x1,1), 'IgnoreAnalyticConstraints', true, 'Steps', 50));

fprintf('\n\nderivative w.r.t y1:\n');
disp(simplify(diff(p,y1,1), 'IgnoreAnalyticConstraints', true, 'Steps', 50));
disp(simplify(diff(q,y1,1), 'IgnoreAnalyticConstraints', true, 'Steps', 50));
disp(simplify(diff(r,y1,1), 'IgnoreAnalyticConstraints', true, 'Steps', 50));






% function for derivative of line coefficients





