clear; close all; clc;
addpath(fullfile('..', 'src'));

% Problem definition
Ts = 1/40; % Sampling period
H = 5; % Horizon length in seconds
x0 = zeros(12,1); % Initial states
Tf = 2.5;% Simulation time
ANIMATION_RATE = 10;

% Problem definition
rocket = Rocket(Ts);
rocket.mass = 1.75;
rocket.delay = 3;
nmpc = NmpcControl(rocket, H, 3);
ref = [0.5,0,1,deg2rad(65)]';

[T,X,U,Ref] = rocket.simulate(x0,Tf,@nmpc.get_u,ref);
rocket.anim_rate = ANIMATION_RATE;
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Nonlinear MPC control of the rocket'; % Set a figure title

