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
% exemple of value to track
pos_ref = [0, 0, 0, 10]';

%% new method for part 2
u = mpc_x.get_u(x, pos_ref);
% Evaluate once and plot optimal open−loop trajectory,
[T, X_sub, U_sub] = rocket.simulate_f(sys_x, x0, Tf, @mpc_x.get_u, ref_x);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us, ref_x);



