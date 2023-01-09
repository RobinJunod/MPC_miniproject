addpath(fullfile('..', 'src'));

%% MPC reference
clear all ;close all ;clc
Ts = 1/20;
rocket = Rocket(Ts);

% NMPC
H = 2; % Horizon length in second
nmpc = NmpcControl(rocket, H);


%% Evaluate once and plot optimal open−loop trajectory NMPC
% pad last input to get consistent size with time and state
% (w, phi, v, p) Initial state
x0 = [deg2rad([0 0 0, 0 0 0]), 0 0 0, 0 0 0]';
ref = [0 0 1 0]';
[u, T_opt, X_opt, U_opt] = nmpc.get_u(x0, ref);
% open loop plot
U_opt(:,end+1) = nan;
ph = rocket.plotvis(T_opt, X_opt, U_opt, ref);


%% plot in closed loop with max roll = 15 NMPC
x0 = [deg2rad([0 0 0, 0 0 0]), 0 0 0, 0 0 0]';
ref = @(t_, x_) ref_EPFL(t_);
Tf = 30;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);

% Plot pose
rocket.anim_rate = 5; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'NMPC in nonlinear simulation (default {\gamma}_{ref}=15 deg)'; % Set a figure title


%% With max roll = 50 NMPC
x0 = [deg2rad([0 0 0, 0 0 0]), 0 0 0, 0 0 0]';
roll_max = deg2rad(50); % MPC reference with specified maximum roll = 50 deg
ref = @(t_, x_) ref_EPFL(t_, roll_max);
Tf = 30;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);

% Plot pose
rocket.anim_rate = 5; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'NMPC in nonlinear simulation (default {\gamma}_{ref}=50 deg)'; % Set a figure title

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% TODO: This file should produce all the plots for the deliverable
clear
close all
clc
% Prepare system
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

rocket.mass =  1.8; % Manipulate mass for simulation

%% Ref function for 15 deg
ref = @(t_ , x_) ref_EPFL(t_);
x0 = zeros(size(sys.A,1),1);
% Offset-Free Tracking
[T, X, U, Ref, Z_hat] = rocket.simulate_est_z(x0, Tf, @mpc.get_u, ref, mpc_z, sys_z);
% Plot pose
rocket.anim_rate = 5; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Continuous-time nonlinear model with estimator in closed-loop'; % Set a figure title


%% Ref function for 50 deg
roll_max = deg2rad(50); % MPC reference with specified maximum roll = 50 deg
ref = @(t_, x_) ref_EPFL(t_, roll_max);
x0 = zeros(size(sys.A,1),1);
% Offset-Free Tracking
[T, X, U, Ref, Z_hat] = rocket.simulate_est_z(x0, Tf, @mpc.get_u, ref, mpc_z, sys_z);
% Plot pose
rocket.anim_rate = 5; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Continuous-time nonlinear model with estimator in closed-loop'; % Set a figure title