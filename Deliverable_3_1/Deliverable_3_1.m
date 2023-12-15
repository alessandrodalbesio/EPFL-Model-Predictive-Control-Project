%% Problem configuration
import gurobi.*;
clc; close all; clear;
addpath(fullfile('..', 'src'));
addpath(fullfile('..', 'utils'));


% Parameter choice
H = 8; % Horizon length [seconds]
Ts = 1/20; % Sample time [seconds]
Tf = 20; % Close-loop simulation time [seconds]
CLOSE_LOOP = true;

% Define starting points
x0 = [0;0;0;3]; % omega_y, beta, v_x, x
y0 = [0;0;0;3]; % omega_x, alpha, v_y, y
z0 = [0;3]; % v_z, z
roll0 = [0;30*pi/180]; % omega_z theta

% System initialization
rocket = Rocket(Ts); % Create the rocket
[xs, us] = rocket.trim(); % Find one equilibrium point
sys = rocket.linearize(xs, us); % Linearize the system in the equilibrium point
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us); % Split the system in four indipendent systems

% Setup the controllers
mpc_x = MpcControl_x(sys_x, Ts, H); % Controller for x state
mpc_y = MpcControl_y(sys_y, Ts, H); % Controller for y state
mpc_z = MpcControl_z(sys_z, Ts, H); % Controller for z state
mpc_roll = MpcControl_roll(sys_roll, Ts, H); % Controller for roll state

%% X state
if CLOSE_LOOP
    [T, X, U] = rocket.simulate_f(sys_x, x0, Tf, @mpc_x.get_u, 0);
else
    [~, T, X, U] = mpc_x.get_u(x0);
    U(:,end+1) = nan; % Nedded to have dimension consistency
end
rocket.plotvis_sub(T, X, U, sys_x, xs, us);  % Plot as usual

%% Y state
if CLOSE_LOOP
    [T, X, U] = rocket.simulate_f(sys_y, y0, Tf, @mpc_y.get_u, 0);
else
    [~, T, X, U] = mpc_y.get_u(y0);
    U(:,end+1) = nan; % Nedded to have dimension consistency
end
rocket.plotvis_sub(T, X, U, sys_y, xs, us);  % Plot as usual

%% Z state
if CLOSE_LOOP
    [T, X, U] = rocket.simulate_f(sys_z, z0, Tf, @mpc_z.get_u, 0);
else
    [~, T, X, U] = mpc_z.get_u(z0);
    U = U + us(3);
    U(:,end+1) = nan; % Nedded to have dimension consistency
end
rocket.plotvis_sub(T, X, U, sys_z, xs, us);  % Plot as usual

%% Roll state
if CLOSE_LOOP
    [T, X, U] = rocket.simulate_f(sys_roll, roll0, Tf, @mpc_roll.get_u, 0);
else
    % Evaluate once and plot optimal open-loop trajectory
    [~, T, X, U] = mpc_roll.get_u(roll0);
    U(:,end+1) = nan; % Nedded to have dimension consistency
end
rocket.plotvis_sub(T, X, U, sys_roll, xs, us);  % Plot as usual