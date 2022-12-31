addpath(fullfile('..', 'src'));

%% MPC reference
clear all ;close all ;clc
Ts = 1/20;
rocket = Rocket(Ts);

H = 2; % Horizon length in second
nmpc = NmpcControl(rocket, H);

%% Evaluate once and plot optimal open−loop trajectory
% pad last input to get consistent size with time and state
% (w, phi, v, p) Initial state
x0 = [deg2rad([0 0 0, 0 0 0]), 0 0 0, 0 0 0]';
ref = [0 0 1 0]';
[u, T_opt, X_opt, U_opt] = nmpc.get_u(x0, ref);
% open loop plot
U_opt(:,end+1) = nan;
ph = rocket.plotvis(T_opt, X_opt, U_opt, ref);

%% plot in closed loop with max roll = 15
Tf = 30;
x0 = [deg2rad([0 0 0, 0 0 0]), 0 0 0, 0 0 0]';
ref = @(t_, x_) ref_EPFL(t_);
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);

% Plot pose
rocket.anim_rate = 5; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'NMPC in nonlinear simulation (default {\gamma}_{ref}=15 deg)'; % Set a figure title


%% With max roll = 50

% MPC reference with specified maximum roll = 50 deg
roll_max = deg2rad(50);
ref = @(t_, x_) ref_EPFL(t_, roll_max);
Tf = 30;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);

% Plot pose
rocket.anim_rate = 5; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'NMPC in nonlinear simulation (default {\gamma}_{ref}=50 deg)'; % Set a figure title

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
%[u, T_opt, X_opt, U_opt] = nmpc.get_u(x, ref);
U_opt(:,end+1) = nan;
ph = rocket.plotvis(T_opt, X_opt, U_opt, ref);
Tf = 30;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);