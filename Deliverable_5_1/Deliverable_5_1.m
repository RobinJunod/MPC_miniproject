addpath(fullfile('..', 'src'));

%% TODO: This file should produce all the plots for the deliverable
clear
close all
clc
%% Prepare system

Ts = 1/20; % [s] Sample time
Tf = 30; % [s] Simulation time

rocket = Rocket(Ts);
% get steady state values
[xs, us] = rocket.trim();
% linearize
sys = rocket.linearize(xs, us);
% create sub system
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

H = 3; % Horizon length in seconds

% Define the four MPC controllers
mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
mpc_z = MpcControl_z(sys_z, Ts, H);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);
% Merge four subsystems
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);

% Ref function
ref = @(t_ , x_) ref_EPFL(t_);
x0 = zeros(size(sys.A,1),1);

rocket.mass = 1.794; % Manipulate mass for simulation


%% Without offset-free tracking (4.1)
[T, X, U, Ref] = rocket.simulate(x0, Tf, @mpc.get_u, ref);

% Plot pose
rocket.anim_rate = 5; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Merged lin. MPC in nonlinear simulation with rocket mass disturbance'; % Set a figure title


%% Offset-Free Tracking
[T, X, U, Ref, Z_hat] = rocket.simulate_est_z(x0, Tf, @mpc.get_u, ref, mpc_z, sys_z);

% Plot pose
rocket.anim_rate = 5; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Merged lin. MPC in nonlinear simulation with offset free tracking'; % Set a figure title






