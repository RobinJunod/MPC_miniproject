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




%% Design MPC controller for subsystem X
H = 5; % Horizon length in seconds
mpc_x = MpcControl_x(sys_x, Ts, H);
% create input state for the subsystem_x
% x = [wy, beta, vx, x]'
x = [0, 0, 0, 5]';
% Create controller to go from x state to steadyState
[u, T_opt, X_opt, U_opt] = mpc_x.get_u(x);
% Evaluate once and plot optimal open−loop trajectory,
U_opt(:,end+1) = nan;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_x, xs, us); % Plot as usual

%% Design MPC controller for subsystem Z
H = 7; % Horizon length in seconds
mpc_y = MpcControl_z(sys_y, Ts, H);

% create input state for the subsystem_x
% x = [wy, beta, vx, x]'
y = [0, 3]';
% Create controller to go from x state to steadyState
[u, T_opt, X_opt, U_opt] = mpc_z.get_u(y);
% Evaluate once and plot optimal open−loop trajectory,
U_opt(:,end+1) = nan;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt + us(3) , sys_y, xs, us); % Plot as usual



%% Design MPC controller for subsystem Z
H = 7; % Horizon length in seconds
mpc_z = MpcControl_z(sys_z, Ts, H);

% create input state for the subsystem_x
% x = [wy, beta, vx, x]'
z = [0, 3]';
% Create controller to go from x state to steadyState
[u, T_opt, X_opt, U_opt] = mpc_z.get_u(z);
% Evaluate once and plot optimal open−loop trajectory,
U_opt(:,end+1) = nan;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt + us(3) , sys_z, xs, us); % Plot as usual