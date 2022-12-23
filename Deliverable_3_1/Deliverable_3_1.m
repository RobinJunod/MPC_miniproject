addpath(fullfile('..', 'src'));
addpath(fullfile('..', 'templates'));
% TODO: This file should produce all the plots for the deliverable

%% lineraize-divide 
clear all ;close all ;clc
Ts = 1/20; % Sample time
rocket = Rocket(Ts);
% get steady state values
[xs, us] = rocket.trim();
% linearize
sys = rocket.linearize(xs, us);
% create sub system
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);




%% Design MPC controller for subsystems
H = 10; % Horizon length in seconds
mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
mpc_z = MpcControl_z(sys_z, Ts, H);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);

% create input state for the subsystem_x
% x = [wy, beta, vx, x]'
x = [0, 0, 0, 5]';
y = [0, 0, 0, 4]';
z = [-0.5,4]';
roll = [0,0.61]'; % 35° roll angle
% Create controller to go from x state to steadyState
[u_x, T_opt_x, X_opt_x, U_opt_x] = mpc_x.get_u(x);
% 
[u_y, T_opt_y, X_opt_y, U_opt_y] = mpc_y.get_u(y);
% 
[u_z, T_opt_z, X_opt_z, U_opt_z] = mpc_z.get_u(z);

[u_roll, T_opt_roll, X_opt_roll, U_opt_roll] = mpc_roll.get_u(roll);

% Evaluate once and plot optimal openâˆ’loop trajectory,
U_opt_x(:,end+1) = nan;
ph_x = rocket.plotvis_sub(T_opt_x, X_opt_x, U_opt_x, sys_x, xs, us); % Plot as usual

U_opt_y(:,end+1) = nan;
ph_y = rocket.plotvis_sub(T_opt_y, X_opt_y, U_opt_y, sys_y, xs, us); % Plot as usual
% 
U_opt_z(:,end+1) = nan;
ph_z = rocket.plotvis_sub(T_opt_z, X_opt_z, U_opt_z + us(3), sys_z, xs, us); % Plot as usual

U_opt_roll(:,end+1) = nan;
ph_roll = rocket.plotvis_sub(T_opt_roll, X_opt_roll, U_opt_roll, sys_roll, xs, us); % Plot as usual

