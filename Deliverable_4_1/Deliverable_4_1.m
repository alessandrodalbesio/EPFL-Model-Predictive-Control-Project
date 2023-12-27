addpath(fullfile('..', 'src'));

%close all
%clear all
%clc

%% TODO: This file should produce all the plots for the deliverable


Ts = 1/20;
rocket = Rocket(Ts);
[xs, us] = rocket.trim(); %compute steady-state for which 0 = f(xs, us)
sys = rocket.linearize(xs, us); %linearize the nonlinear model about trim point

[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

H = 8; % Horizon length in seconds 
mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
mpc_z = MpcControl_z(sys_z, Ts, H);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);

% Merge four sub−system controllers into one full−system controller
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);


% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
x0 = zeros(12,1);
ref4 = [-4 -4 -4 deg2rad(35)]';
[u, T_opt,X_opt, U_opt] = mpc.get_u(x0, ref4);
U_opt(:,end+1) = nan;

ph = rocket.plotvis(T_opt, X_opt, U_opt, ref4); % Plot as usual

% Setup reference function
ref = @(t , x ) ref_EPFL(t );
% Simulate
Tf = 30;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @mpc.get_u, ref);
% Visualize
rocket.anim_rate = 1; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Merged lin. MPC in nonlinear simulation'; % Set a figure title
