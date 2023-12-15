%% Problem definition
import gurobi.*;
clc; clear; close all;
addpath(fullfile('..', 'src')); % Add the path with all the informations about the rocket 

% General settings
H = 8; % Horizon length [second]
Ts = 1/20; % Sample time
Tf = 20; % Close-loop simulation time [second]
CLOSE_LOOP = true; % Set tu true if we want to study the close-loop evolution otherwise set it to false
x0 = [0;0;0;0]; % omega_y, beta, v_x, x
y0 = [0;0;0;0]; % omega_x, alpha, v_y, y
z0 = [0;0]; % v_z, z
roll0 = [0;0]; % omega_z theta
ref = [-4,-4,-4,35*pi/180]; % Define the reference x,y,z,roll

% Initialize the system
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
    [T, X, U] = rocket.simulate_f(sys_x, x0, Tf, @mpc_x.get_u, ref(1,1));
else
    % Evaluate once and plot optimal open-loop trajectory
    [~, T, X, U] = mpc_x.get_u(x0, ref(1,1));
    U(:,end+1) = nan; % Nedded to have dimension consistency
end
rocket.plotvis_sub(T, X, U, sys_x, xs, us, ref(1,1));  % Plot as usual

%% Y state
if CLOSE_LOOP
    [T, X, U] = rocket.simulate_f(sys_y, y0, Tf, @mpc_y.get_u, ref(1,2));
else
    % Evaluate once and plot optimal open-loop trajectory
    [~, T, X, U] = mpc_y.get_u(y0, ref(1,2));
    U(:,end+1) = nan; % Nedded to have dimension consistency
end
rocket.plotvis_sub(T, X, U, sys_y, xs, us, ref(1,2));  % Plot as usual

%% Z state
if CLOSE_LOOP
    [T, X, U] = rocket.simulate_f(sys_z, z0, Tf, @mpc_z.get_u, ref(1,3));
else
    % Evaluate once and plot optimal open-loop trajectory
    [~, T, X, U] = mpc_z.get_u(z0, ref(1,3));
    U = U + us(3);
    U(:,end+1) = nan; % Nedded to have dimension consistency
end
rocket.plotvis_sub(T, X, U, sys_z, xs, us, ref(1,3));  % Plot as usual

%% Roll state
if CLOSE_LOOP
    [T, X, U] = rocket.simulate_f(sys_roll, roll0, Tf, @mpc_roll.get_u, ref(1,4));
else
    % Evaluate once and plot optimal open-loop trajectory
    [~, T, X, U] = mpc_roll.get_u(roll0, ref(1,4));
    U(:,end+1) = nan; % Nedded to have dimension consistency
end
rocket.plotvis_sub(T, X, U, sys_roll, xs, us, ref(1,4));  % Plot as usual