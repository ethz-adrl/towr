function [] = drawTriangles( number_triangles, x, y, fig_handle)

% Draws the specified number of support triangles
% x: x-values of the triangle (3 per triangle)
% y: y-values corresponding y values
% fig_handle: in which figure these triangles should be drawn

figure(fig_handle);
c = nextColor('yellow'); % next color after yellow is green
h = 0;
for i = 1:number_triangles;
    
    % index of points 1,2 & 3 of current triangle
    i1 = 3*(i-1) + 1;
    i2 = 3*(i-1) + 2;
    i3 = 3*(i-1) + 3;
    
    % draw 3 lines connecting each of these points
    h(i) = line( [x(i1) x(i2)], [y(i1) y(i2)], 'color', c, 'LineWidth', 2);
    line( [x(i2) x(i3)], [y(i2) y(i3)], 'color', c, 'LineWidth', 2);
    line( [x(i3) x(i1)], [y(i3) y(i1)], 'color', c, 'LineWidth', 2);
    
    c = nextColor(c);
    
end

legend(h, ['LH'; 'LF'; 'RH'; 'RF']);
end