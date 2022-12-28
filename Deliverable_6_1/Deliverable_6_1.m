addpath(fullfile('..', 'src'));

%% TODO: This file should produce all the plots for the deliverable

%% MPC reference with default maximum roll = 15 deg
Ts = 1/20;
rocket = Rocket(Ts);
H = 3; % Horizon length in seconds

nmpc = NmpcControl(rocket, H);

%% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
% (w, phi, v, p) Initial state
x0 = [deg2rad([0 0 0, 0 0 0]), 0 0 0, 0 0 0]';
ref = [10 0 0 0]';
[u, T_opt, X_opt, U_opt] = nmpc.get_u(x0, ref);
%%
ph = rocket.plotvis(T_opt, X_opt, U_opt, ref);
%% open loop
%ref = @(t_, x_) ref_EPFL(t_);
ref = [10 0 0 0]';
U_opt(:,end+1) = nan;
ph = rocket.plotvis(T_opt, X_opt, U_opt, ref);

%% plot in closed loop
Tf = 30;
ref = @(t_, x_) ref_EPFL(t_);
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);


%% With default max roll = 50
Ts = 1/20;
rocket = Rocket(Ts);
H = 3; % Horizon length in seconds
nmpc = NmpcControl(rocket, H);

% MPC reference with specified maximum roll = 50 deg
roll_max = deg2rad(50);
ref = @(t_, x_) ref_EPFL(t_, roll_max);

% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
[u, T_opt, X_opt, U_opt] = nmpc.get_u(x, ref);
U_opt(:,end+1) = nan;
ph = rocket.plotvis(T_opt, X_opt, U_opt, ref);
Tf = 30;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);