%% file to plot rosbags in matlab

clc;
clear all;


filePath = '~/Desktop/matlab_rdy.bag';
bag_all = rosbag(filePath);


T = 2;
selectOptions = {'Time', [bag_all.StartTime bag_all.StartTime + T] };
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
ts_force_1 = timeseries(bag_force_1, 'Z');







%% 
t = ts_base_pos.Time; % was one point too many
n = size(t,1); % number of sampled points

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


% calculate actual base acceleration from plan
m = 20; %36.5;
g = 9.81;
F_ext = force_0_z + force_1_z;
base_zdd_dynamics = 1/m*F_ext - g;
% calculate Root mean square error
base_zdd_error = base_zdd_dynamics - base_zdd;
norm_sqare = norm(base_zdd_error)^2;
RMSE = sqrt(norm_sqare/n) % make sure to mention on how many data points.


% intervall at which dynamic constraint is enforced
dt_dyn = 0.2;
dt_foot_0 = [0, 0.8, 1.3];
dt_foot_1 = [1.2, 1.5];
dt_foot = sort([dt_foot_0 dt_foot_1]);



%set(gca, 'ColorOrder', [0.5 0.5 0.5; 0.2 0.2 0.2; 0.8 0.8 0.8],'NextPlot', 'replacechildren');


figure(1);
sp1 = subplot(3,1,1);
plot(t,base_zdd, 'g'); hold on; 
plot(t,base_zdd_dynamics, 'g--'); hold on;
%set(sp1,'xtick',[0:0.2:T]);
sp1.XGrid = 'on';
sp1.XTick = [0:dt_dyn:T];

sp2 = subplot(3,1,2);
plot(t, base_z); hold on;
plot(t, foot_0_z, 'r'); hold on;
plot(t, foot_1_z, 'b'); hold on;
sp2.XGrid = 'on';
sp2.XTick = dt_foot;

sp3 = subplot(3,1,3);
plot(t, force_0_z, 'r'); hold on;
plot(t, force_1_z, 'b');
sp3.XGrid = 'on';
sp3.XTick = dt_foot;















