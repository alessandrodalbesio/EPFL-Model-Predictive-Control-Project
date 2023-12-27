%% Problem configuration
import gurobi.*;
clc; close all; clear;
addpath(fullfile('..', 'src'));

% Parameter choice
H = 8; % Horizon length [seconds]
Ts = 1/20; % Sample time [seconds]
Tf = 10; % Close-loop simulation time [seconds]
OFFSET_FREE_TRACKING = true;

% Starting point
x0 = [zeros(1,9),1,0,3]';

% System initialization
rocket = Rocket(Ts); % Create the rocket
[xs, us] = rocket.trim(); % Find one equilibrium point
sys = rocket.linearize(xs, us); % Linearize the system in the equilibrium point found
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us); % Split the system in four indipendent systems


%% Simulation
mpc_x = MpcControl_x(sys_x, Ts, H); % Controller for x state
mpc_y = MpcControl_y(sys_y, Ts, H); % Controller for y state
mpc_z = MpcControl_z(sys_z, Ts, H); % Controller for z state
mpc_roll = MpcControl_roll(sys_roll, Ts, H); % Controller for roll state
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll); % Merged controller
ref = @(t_, x_) ref_TVC(t_);
rocket.mass = 2.13;
if OFFSET_FREE_TRACKING == true; [T, X, U, Ref, Z_hat] = rocket.simulate_est_z(x0,Tf,@mpc.get_u,ref, mpc_z, sys_z); end
if OFFSET_FREE_TRACKING == false; [T, X, U, Ref] = rocket.simulate(x0, Tf, @mpc.get_u, ref); end

%% Plotting offset log

if OFFSET_FREE_TRACKING == true
    figure;
    plot(T, Z_hat(end,:));
    title('Disturbance estimation');
    grid on;
end

%% Rocket evolution
rocket.anim_rate = 1;
ph = rocket.plotvis(T,X,U,Ref);
ph.fig.Name = 'Offset-free tracking';