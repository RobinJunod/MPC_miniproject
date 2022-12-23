% testing the rocket class

%% Task 1.1

Ts = 1/20;
rocket = Rocket(Ts);

% input
d1 = 0;
d2 = 0;
Pavg = 60;
Pdiff = 0;
u = [d1, d2, Pavg, Pdiff]'; % (Assign appropriately)
[b_F, b_M] = rocket.getForceAndMomentFromThrust(u);

% states
w = deg2rad([2 -2 0])';
phi = deg2rad([-2 2 0])';
v = [0 0 0]';
p = [0 0 0]';
x = [w; phi; v; p]; % (Assign appropriately)
x_dot = rocket.f(x, u);


%% Task 1.2
Ts = 1/20;
rocket = Rocket(Ts);
Tf = 2.0; % Simulation end time

% (w, phi, v, p) Initial state
x0 = [deg2rad([0 0 0, 0 0 0]), 0 0 0, 0 0 0]'; 

% (d1 d2 Pavg Pdiff) Constant input
u = [deg2rad([0 0]), 60, 0 ]'; 

% Simulate unknown, nonlinear model
[T, X, U] = rocket.simulate(x0, Tf, u); 


rocket.anim_rate = 1.0; % Visualize at 1.0x real−time
rocket.vis(T, X, U);



%% PART 2 : LINEARIZED PART OF THE PROJECT
Ts = 1/20;
rocket = Rocket(Ts);
[xs, us] = rocket.trim(); % Compute steady−state for which 0 = f(xs,us)
sys = rocket.linearize(xs, us); % Linearize the nonlinear model about trim point

%% part 2.1
% devide into sub system
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

%% part 3 discretize


sys_x_d = c2d(sys_x, Ts);
sys_y_d = c2d(sys_y, Ts);
sys_z_d = c2d(sys_z, Ts);
sys_roll_d = c2d(sys_roll, Ts);

% constraints
Ts = 1/20; % Sample time
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys x, sys y, sys z, sys roll] = rocket.decompose(sys, xs, us);
% Design MPC controller
H = ..; % Horizon length in seconds
mpc x = MpcControl x(sys x, Ts, H);
% Get control input
u x = mpc x.get u(x)



