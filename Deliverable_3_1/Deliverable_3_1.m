addpath(fullfile('..', 'src'));
% TODO: This file should produce all the plots for the deliverable
clear
close all
clc
%% lineraize-divide 

Ts = 1/20; % Sample time
rocket = Rocket(Ts);
Tf = 10.0; % s


% get steady state values
[xs, us] = rocket.trim();
% linearize
sys = rocket.linearize(xs, us);
% create sub system
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);


H = 3; % Horizon length in secondsH = 3; % Horizon length in seconds

%% Design MPC controller for subsystem X

mpc_x = MpcControl_x(sys_x, Ts, H);

% create input state for the subsystem_x
% x = [wy, beta, vx, x]'
x0_x = [0, 0, 0, 4];

% Create controller to go from x state to steadyState
[u_x, T_opt_x, X_opt_x, U_opt_x] = mpc_x.get_u(x0_x');

%Evaluate once and plot optimal openloop trajectory,
U_opt_x(:,end+1) = nan;
ph_x = rocket.plotvis_sub(T_opt_x, X_opt_x, U_opt_x, sys_x, xs, us); % Plot as usual
%Change open-loop plot color
for j = 2:length(ph_x.fig.Children)
for i = 1:length(ph_x.fig.Children(j).Children)
ph_x.fig.Children(j).Children(i).Color = "red";
end
end

%Simulate closed-loop and plot
[T_x, X_sub_x, U_sub_x] = rocket.simulate_f(sys_x, x0_x, Tf, @mpc_x.get_u, 0); 
ph_xc = rocket.plotvis_sub(T_x, X_sub_x, U_sub_x, sys_x, xs, us);

for i = 1:length(ph_x.fig.Children)
copyobj(ph_x.fig.Children(i).Children,ph_xc.fig.Children(i));
end
% saveas(ph_xc.fig,'cl_x_H3.png');

%% Design MPC controller for subsystem Y

mpc_y = MpcControl_y(sys_y, Ts, H);

% create input state for the subsystem_y
% y = [wx, alpha, vy, y]'
x0_y = [0, 0, 0, 4];

% Create controller to go from y state to steadyState 
[u_y, T_opt_y, X_opt_y, U_opt_y] = mpc_y.get_u(x0_y');

% Evaluate once and plot optimal openloop trajectory,
U_opt_y(:,end+1) = nan;
ph_y = rocket.plotvis_sub(T_opt_y, X_opt_y, U_opt_y, sys_y, xs, us); % Plot as usual
%Change open-loop plot color
for j = 2:length(ph_y.fig.Children)
for i = 1:length(ph_y.fig.Children(j).Children)
ph_y.fig.Children(j).Children(i).Color = "red";
end
end
%Simulate closed-loop and plot
[T_y, X_sub_y, U_sub_y] = rocket.simulate_f(sys_y, x0_y, Tf, @mpc_y.get_u, 0); 
ph_yc = rocket.plotvis_sub(T_y, X_sub_y, U_sub_y, sys_y, xs, us);

for i = 1:length(ph_y.fig.Children)
copyobj(ph_y.fig.Children(i).Children,ph_yc.fig.Children(i));
end
% saveas(ph_yc.fig,'cl_y_H3.png');
%% Design MPC controller for subsystem Z
mpc_z = MpcControl_z(sys_z, Ts, H);

% create input state for the subsystem_z
% z = [vz, z]'
x0_z = [0,4];

% Create controller to go from z state to steadyState 
[u_z, T_opt_z, X_opt_z, U_opt_z] = mpc_z.get_u(x0_z');

% Evaluate once and plot optimal openloop trajectory,
U_opt_z(:,end+1) = nan;
ph_z = rocket.plotvis_sub(T_opt_z, X_opt_z, U_opt_z + us(3), sys_z, xs, us); % Plot as usual
%Change open-loop plot color
for j = 2:length(ph_z.fig.Children)
for i = 1:length(ph_z.fig.Children(j).Children)
ph_z.fig.Children(j).Children(i).Color = "red";
end
end

%Simulate closed-loop and plot
[T_z, X_sub_z, U_sub_z] = rocket.simulate_f(sys_z, x0_z, Tf, @mpc_z.get_u, 0); 
ph_zc = rocket.plotvis_sub(T_z, X_sub_z, U_sub_z, sys_z, xs, us);

for i = 1:length(ph_z.fig.Children)
copyobj(ph_z.fig.Children(i).Children,ph_zc.fig.Children(i));
end

% saveas(ph_zc.fig,'cl_z_H3.png');
%% Design MPC controller for subsystem Roll
mpc_roll = MpcControl_roll(sys_roll, Ts, H);

% create input state for the subsystem_roll
% roll = [wz, gamma]'
x0_roll = [0,0.61086]; % 35° roll angle

% Create controller to go from roll state to steadyState
[u_roll, T_opt_roll, X_opt_roll, U_opt_roll] = mpc_roll.get_u(x0_roll');

% Evaluate once and plot optimal openloop trajectory,
U_opt_roll(:,end+1) = nan;
ph_roll = rocket.plotvis_sub(T_opt_roll, X_opt_roll, U_opt_roll, sys_roll, xs, us); % Plot as usual
%Change open-loop plot color
for j = 2:length(ph_roll.fig.Children)
for i = 1:length(ph_roll.fig.Children(j).Children)
ph_roll.fig.Children(j).Children(i).Color = "red";
end
end

%Simulate closed-loop and plot
[T_roll, X_sub_roll, U_sub_roll] = rocket.simulate_f(sys_roll, x0_roll, Tf, @mpc_roll.get_u, 0); 
ph_rollc = rocket.plotvis_sub(T_roll, X_sub_roll, U_sub_roll, sys_roll, xs, us);

for i = 1:length(ph_roll.fig.Children)
copyobj(ph_roll.fig.Children(i).Children,ph_rollc.fig.Children(i));
end
% saveas(ph_rollc.fig,'cl_roll_H3.png');
