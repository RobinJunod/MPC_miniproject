addpath(fullfile('..', 'src'));
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
%Evaluate once and plot optimal open−loop trajectory, SYS X
[T, X_sub, U_sub] = rocket.simulate_f(sys_x, x0, Tf, @mpc_x.get_u, x_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us, x_ref);


%% Design MPC controller for subsystem Y
mpc_y = MpcControl_y(sys_y, Ts, H);
% y = [wx, alpha, vy, y]'
y0 = [0, 0, 0, 0]';
y_ref = -4;
%Evaluate once and plot optimal open−loop trajectory, SYS Y
[T, X_sub, U_sub] = rocket.simulate_f(sys_y, y0, Tf, @mpc_y.get_u, y_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_y, xs, us, y_ref);


%% Design MPC controller for subsystem Z
mpc_z = MpcControl_z(sys_z, Ts, H);
% z = [vz, z]'
z0 = [0, 0]';
z_ref = -4';
%Evaluate once and plot optimal open−loop trajectory, SYS Z
[T, X_sub, U_sub] = rocket.simulate_f(sys_z, z0, Tf, @mpc_z.get_u, z_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us, z_ref);


%% Design MPC controller for subsystem roll
mpc_roll = MpcControl_roll(sys_roll, Ts, H);
% roll = [wz, gamma]'
roll0 = [0, 0]';
roll_ref = 0.61086;
% Evaluate once and plot optimal open−loop trajectory, SYS ROLL
[T, X_sub, U_sub] = rocket.simulate_f(sys_roll, roll0, Tf, @mpc_roll.get_u, roll_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us, roll_ref);

















