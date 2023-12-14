addpath(fullfile('..', 'src'));

%close all
%clear all
%clc

%% TODO: This file should produce all the plots for the deliverable
Tf = 5.0; % Simulation end time
Ts = 1/20; % Sample time
H = 9; % Horizon length in seconds 8+1

rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

x = [0;0;0;4];
x_ref=-4;
y = [0;0;0;4];
y_ref=-4
z=[0;4];
z_ref=-4
roll=[deg2rad([0,35])]';

%% Design MPC X controller
mpc_x = MpcControl_x(sys_x, Ts, H);
u_x = mpc_x.get_u(x);
%[u, T_opt, X_opt, U_opt] = mpc_x.get_u(x);
%U_opt(:,end+1) = nan;
%ph = rocket.plotvis_sub(T_opt, X_opt, U_opt,sys_x, xs, us); 


[T, X_sub, U_sub] = rocket.simulate_f(sys_x, x, Tf, @mpc_x.get_u, x_ref); 
ph2 = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us,x_ref);
ph2.fig.Name = 'X.3.2';

%% Design MPC y controller
mpc_y = MpcControl_y(sys_y, Ts, H);
u_x = mpc_y.get_u(y);
%[u, T_opt, X_opt, U_opt] = mpc_y.get_u(y);
%U_opt(:,end+1) = nan;
%ph = rocket.plotvis_sub(T_opt, X_opt, U_opt,sys_y, xs, us); 


[T, X_sub, U_sub] = rocket.simulate_f(sys_y, y, Tf, @mpc_y.get_u, y_ref); 
ph2 = rocket.plotvis_sub(T, X_sub, U_sub, sys_y, xs, us,y_ref);
ph2.fig.Name = 'Y.3.2';

%% Design MPC Z controller
mpc_z = MpcControl_z(sys_z, Ts, H);
u_x = mpc_z.get_u(z);
[u, T_opt, X_opt, U_opt] = mpc_z.get_u(z);
U_opt(:,end+1) = nan;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt,sys_z, xs, us); 


[T, X_sub, U_sub] = rocket.simulate_f(sys_z, z, Tf, @mpc_z.get_u, z_ref); 
ph2 = rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us,z_ref);
ph2.fig.Name = 'Z.3.2';
%% Design MPC roll controller
mpc_roll = MpcControl_roll(sys_roll, Ts, H);

roll=[deg2rad([0,35])]';
roll_ref = 3.1415/4;
u_x = mpc_roll.get_u(roll);

[u, T_opt, X_opt, U_opt] = mpc_roll.get_u(roll);
U_opt (:,end+1) = nan;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_roll, xs, us);

[T, X_sub, U_sub] = rocket.simulate_f(sys_roll, roll, Tf, @mpc_roll.get_u, roll_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us, roll_ref);
ph.fig.Name = 'Roll'; 