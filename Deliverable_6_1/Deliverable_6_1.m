clear; close all; clc;
addpath(fullfile('..', 'src'));

% Problem definition
Ts = 1/20; % Sampling period
H = 5; % Horizon length in seconds
x0 = zeros(12,1); % Initial states
Tf = 30;% Simulation time
MAX_ROLL_15DEG = false;
CLOSE_LOOP = true;
ANIMATION_RATE = 10;

% Problem definition
rocket = Rocket(Ts);
nmpc = NmpcControl(rocket, H);
if CLOSE_LOOP
    if MAX_ROLL_15DEG
        ref = @(t_, x_) ref_TVC(t_);
    else
        ref = @(t , x ) ref_TVC(t , deg2rad(50));
    end
    [T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
    rocket.anim_rate = ANIMATION_RATE;
    ph = rocket.plotvis(T, X, U, Ref);
    ph.fig.Name = 'Nonlinear MPC control of the rocket'; % Set a figure title
else
    ref4 = [2 2 2 deg2rad(50)]';
    [u, T_opt, X_opt, U_opt] = nmpc.get_u(x0, ref4);
    U_opt(:,end+1) = nan;
    ph = rocket.plotvis(T_opt, X_opt, U_opt, ref4);
end