function [ x, y, x_dd, y_dd, x_zmp, y_zmp ] = PositionAtT( p, t )
%Returns the (x,y) positions of a spline for all times in t
%
%Usage: [ x, y ] = PositionAtT(p,t)
%p = 12 spline parameters (x1,..x6, y1,...y6) of the spline
%t = vector of times in [s] of spline at which position is desired
%x,y = vectors of positions corresponding to each time in t

ax = p(1); ay = p(7); 
bx = p(2); by = p(8); 
cx = p(3); cy = p(9);
dx = p(4); dy = p(10);
ex = p(5); ey = p(11);
fx = p(6); fy = p(12);


x = ax*t.^5 + bx*t.^4 + cx*t.^3 + dx*t.^2 + ex*t + fx;
y = ay*t.^5 + by*t.^4 + cy*t.^3 + dy*t.^2 + ey*t + fy;

% x_d = 5*ax*t.^4 + 4*bx*t.^3 + 3*cx*t.^2 + 2*dx*t + ex;
% x_d = 5*ay*t.^4 + 4*by*t.^3 + 3*cy*t.^2 + 2*dy*t + ex;

x_dd = 20*ax*t.^3 + 12*bx*t.^2 + 6*cx*t + 2*dx;
y_dd = 20*ay*t.^3 + 12*by*t.^2 + 6*cy*t + 2*dy;
    
x_zmp = x - 0.58/9.81*x_dd;
y_zmp = y - 0.58/9.81*y_dd;

end

