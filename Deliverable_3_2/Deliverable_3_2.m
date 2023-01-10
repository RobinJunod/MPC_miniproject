addpath(fullfile('..', 'src'));
% TODO: This file should produce all the plots for the deliverable

%% lineraize-divide 
clear ;close all ;clc
Ts = 1/20; % Sample time
rocket = Rocket(Ts);
% get steady state values
[xs, us] = rocket.trim();
% linearize
sys = rocket.linearize(xs, us);
% create sub system
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

% TIME SIZE
H = 3; % Horizon length in seconds
Tf = 10; % simulation time


% new method for part 2
% u = mpc_x.get_u(x0, x_ref);


%% Design MPC controller for subsystem X
mpc_x = MpcControl_x(sys_x, Ts, H);
% x = [wy, beta, vx, x]'
x0 = [0, 0, 0, 0]';
x_ref = -4;

% Create controller to go from 0 state to -4
[u_x, T_opt_x, X_opt_x, U_opt_x] = mpc_x.get_u(x0, x_ref);

%Evaluate once and plot optimal openloop trajectory
U_opt_x(:,end+1) = nan;
ph_x = rocket.plotvis_sub(T_opt_x, X_opt_x, U_opt_x, sys_x, xs, us); % Plot as usual

%Change open-loop plot color
for j = 2:length(ph_x.fig.Children)
for i = 1:length(ph_x.fig.Children(j).Children)
ph_x.fig.Children(j).Children(i).Color = "red";
end
end

%Simulate closed-loop and plot
[T, X_sub, U_sub] = rocket.simulate_f(sys_x, x0, Tf, @mpc_x.get_u, x_ref);
ph_xc = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us, x_ref);

%Copy open-loop onto closed-loop
for i = 1:length(ph_x.fig.Children)
copyobj(ph_x.fig.Children(i).Children,ph_xc.fig.Children(i));
end
saveas(ph_xc.fig,'x_3_2.png');

%% Design MPC controller for subsystem Y
mpc_y = MpcControl_y(sys_y, Ts, H);
% y = [wx, alpha, vy, y]'
y0 = [0, 0, 0, 0]';
y_ref = -4;

% Create controller to go from 0 state to -4 m 
[u_y, T_opt_y, X_opt_y, U_opt_y] = mpc_y.get_u(y0,y_ref);

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
[T, X_sub, U_sub] = rocket.simulate_f(sys_y, y0, Tf, @mpc_y.get_u, y_ref);
ph_yc = rocket.plotvis_sub(T, X_sub, U_sub, sys_y, xs, us, y_ref);

for i = 1:length(ph_y.fig.Children)
copyobj(ph_y.fig.Children(i).Children,ph_yc.fig.Children(i));
end
saveas(ph_yc.fig,'y_3_2.png');
%% Design MPC controller for subsystem Z
mpc_z = MpcControl_z(sys_z, Ts, H);
% z = [vz, z]'
z0 = [0, 0]';
z_ref = -4;

% Create controller to go from 0 state -4 
[u_z, T_opt_z, X_opt_z, U_opt_z] = mpc_z.get_u(z0,z_ref);

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
[T, X_sub, U_sub] = rocket.simulate_f(sys_z, z0, Tf, @mpc_z.get_u, z_ref);
ph_zc = rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us, z_ref);

for i = 1:length(ph_z.fig.Children)
copyobj(ph_z.fig.Children(i).Children,ph_zc.fig.Children(i));
end

saveas(ph_zc.fig,'z_3_2.png');
%% Design MPC controller for subsystem roll
mpc_roll = MpcControl_roll(sys_roll, Ts, H);
% roll = [wz, gamma]'
roll0 = [0, 0]';
roll_ref = 0.61086;

% Create controller to go from 0 state to 35°
[u_roll, T_opt_roll, X_opt_roll, U_opt_roll] = mpc_roll.get_u(roll0,roll_ref);

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
[T, X_sub, U_sub] = rocket.simulate_f(sys_roll, roll0, Tf, @mpc_roll.get_u, roll_ref);
ph_rollc = rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us, roll_ref);

for i = 1:length(ph_roll.fig.Children)
copyobj(ph_roll.fig.Children(i).Children,ph_rollc.fig.Children(i));
end
saveas(ph_rollc.fig,'roll_3_2.png');















