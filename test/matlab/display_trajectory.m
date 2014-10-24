% Plots the optimized CoG trajectory defined by the spline parameters.
% Input file: ./matlab/trajectory_coefficients.log 

clear all;
close all;
clc;

% read in the textfile formatted by log4cxx
zmp_env = getenv('ALEX_LIB_ROOT');
zmp_log_file = strcat(zmp_env, '/test/matlab/trajectory_coefficients.log'); 
fid = fopen(zmp_log_file);

C = textscan(fid, '%s','delimiter', '\n');
n_steps      = str2num(C{1}{1});
spline_times = str2num(C{1}{2});
triangle_x   = str2num(C{1}{3});
triangle_y   = str2num(C{1}{4});
p            = str2num(C{1}{5});

% calculate some basic values from these read in values
t_max_spline = spline_times(1); 
t_max_4ls = spline_times(2);

params = (reshape(p',12,[]))'; %% saves each set of 12 params in new row
n_splines = size(params,1); % 1 = want first dimension (rows) of matrix
n_splines_per_step = floor(n_splines/n_steps);

% create the figure window
h_xy = mouse_figure;
set(h_xy,'name','x-y- plot of cog positions','numbertitle','off');
iptsetpref('ImshowBorder','tight');
h_ti = get(gca,'TightInset');
set(gca,'Position',[h_ti(1) h_ti(2) 1-h_ti(3)-h_ti(1) 1-h_ti(4)-h_ti(2)]);
grid on;
axis equal;
xlabel('x (m)'); ylabel('y (m)');
% axis([-0.3 1 -0.6 0.6]);

c_prev = nextColor('yellow'); % so c_prev is yellow and first color is green
s_count = 0; % counts current 

dt = 0.01;

for s=1:n_splines
       
    % splines 1,..,4,...7,...
    four_leg_supp_spline = (mod(s,2*n_splines_per_step+1) == 0);
    
    if four_leg_supp_spline
        t = 0:dt:t_max_4ls;
        c = 'r'; %$[0.3 0.3 0.3]; % gray
    else
        t = 0:dt:t_max_spline;
        
        if s_count == n_splines_per_step
            c_prev = nextColor(c_prev);
            s_count = 0;
        end
        
        s_count = s_count + 1;
        c = c_prev;
    end
        
    [x_, y_, xdd_, ydd_, x_zmp_, y_zmp_] = PositionAtT( params(s,:), t );
    
    % plot spline s
    figure(h_xy); hold on;
    scatter(x_, y_, 50, c, 'x');
    %scatter(x_zmp_, y_zmp_, 40, c, 'filled');
    legend('CoG', 'ZMP');
    
    % save temporary x,y-states for viewing
    n = length(x_); % number of points of this spline
    x(s,1:n) = x_;         y(s,1:n) = y_;
    xdd(s,1:n) = xdd_;     ydd(s,1:n) = ydd_;
    x_zmp(s,1:n) = x_zmp_; y_zmp(s,1:n) = y_zmp_;             
end

% plot suppport triangles
drawTriangles( n_steps, triangle_x, triangle_y, h_xy);




