%% Plot towr rosbags in matlab
%
% The bags can be generated using
% towr_ros/src/exe/rosbag_geom_msg_extractor.cc.
%
% Author: Alexander Winkler

clc;
clear all;

%% Extract the desired 3D vectors from the bag
filePath = '~/Desktop/matlab_rdy.bag';
bag_all = rosbag(filePath);

t0 = 0.0; %bag_all.StartTime;
T  = 3.4; %bag_all.EndTime;

selectOptions = {'Time', [t0 T] };
bag = select(bag_all, selectOptions{:});

% base motion
bag_base_pose = select(bag, 'Topic', 'base_pose');
ts_base_pos = timeseries(bag_base_pose, 'Position.X', 'Position.Y', 'Position.Z');

bag_base_acc  = select(bag, 'Topic', 'base_acc');
ts_base_acc = timeseries(bag_base_acc, 'Z');

% endeffector motion
bag_foot_0 = select(bag, 'Topic', 'foot_pos_0');
ts_foot_0 = timeseries(bag_foot_0, 'Z');

bag_foot_1 = select(bag, 'Topic', 'foot_pos_1');
ts_foot_1 = timeseries(bag_foot_1, 'Z');

% endeffector forces
bag_force_0 = select(bag, 'Topic', 'foot_force_0');
ts_force_0 = timeseries(bag_force_0, 'Z');

bag_force_1 = select(bag, 'Topic', 'foot_force_1');
ts_force_1  = timeseries(bag_force_1, 'Z');


%% define the plotting range and other additional quantities
t = ts_base_pos.Time; 

% base motion
base_x   = ts_base_pos.Data(:,1);
base_y   = ts_base_pos.Data(:,2);
base_z   = ts_base_pos.Data(:,3);

% base acceleration
base_zdd = ts_base_acc.Data(:,1);

% foot motion
foot_0_z = ts_foot_0.Data(:,1);
foot_1_z = ts_foot_1.Data(:,1);

% foot force
force_0_z = ts_force_0.Data(:,1);
force_1_z = ts_force_1.Data(:,1);


% calculate RMSE between base-z acceleration and what should be the 
% acceleration based on the forces and gravity.
m = 20;   % weight of the robot
g = 9.81; % gravity acceleration
F_ext = force_0_z + force_1_z;
base_zdd_dynamics = 1/m*F_ext - g;
% calculate Root mean square error
base_zdd_error = base_zdd_dynamics - base_zdd;
norm_sqare = norm(base_zdd_error)^2;
n = size(t,1); % number of sampled points
RMSE = sqrt(norm_sqare/n) 

% intervall at which dynamic constraint is enforced
dt_dyn = 0.2;

spline_durations_f0 = [0.25 0.45 0.34 0.35 0.63 0.13 0.50 0.12 0.56 0.40];
spline_durations_f1 = [0.62 0.41 0.59 0.14 0.39 0.13 0.78 0.36 0.23 0.46];

% create vector with absolute timings
num_phases   = size(spline_durations_f0, 2);
dt_foot_0    = spline_durations_f0;
dt_foot_1    = spline_durations_f1;

for c = 2:num_phases
  dt_foot_0(1,c) = dt_foot_0(c-1) + spline_durations_f0(c);
  dt_foot_1(1,c) = dt_foot_1(c-1) + spline_durations_f1(c);
end


%% plot the values
fh = figure(1);

sp1 = subplot(3,1,1);
plot(t,base_zdd, 'k'); hold on; 
plot(t,base_zdd_dynamics, 'k--'); hold on;
xlim([t0 T]);
sp1.XGrid = 'on';
sp1.XTick = [t0:dt_dyn:T];


sp2 = subplot(3,1,2);
[he0t, he0z, he0f] = plotyy(t, foot_0_z, t, force_0_z);
set(he0t(1),'XLim',[t0 T]);
set(he0t(2),'XLim',[t0 T]);
set(he0t(2),'YLim',[-80 600]);
set(he0t(2),'YTick',[-200:200:600]);
sp2.XGrid = 'on';
sp2.XTick = dt_foot_0;


sp3 = subplot(3,1,3);
[he1t, he1z, he1f] = plotyy(t, foot_1_z, t, force_1_z);
set(he1t(1),'XLim',[t0 T]);
set(he1t(2),'XLim',[t0 T]);
set(he1t(2),'YLim',[-80 600]);
set(he1t(2),'YTick',[-200:200:600]);
sp3.XGrid = 'on';
sp3.XTick = dt_foot_1;



%%  Generate pdf from the figure for paper
width  = 20;
height = 15;

fh.Units = 'centimeters';
fh.PaperUnits = 'centimeters';
fh.Position = [0, 0, width, height];
fh.PaperSize = [width, height];
fh.PaperPositionMode = 'auto';
fn = 'side_stepping';

saveas(fh, fn, 'pdf')
system(['pdfcrop ' fn ' ' fn]);















