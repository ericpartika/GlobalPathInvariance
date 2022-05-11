%%%
% This is the set up for the vehicle model
%%%
clear all
close all

%% Init
global L;
global lr;
L = 0.3; % meters
lr = L/2;

%%% limits:
% velocity: 0.5 m/s
% steering angle: +- 20deg or +- 0.349066rad

% Initial conditions
x0 = [1; 5; 0; 0]; % z = (x_pos; y_pos; theta; delta) x and y are rear axle of vehicle
u0 = [0; 0]; % u = (v, omega) omega is steering angle rate 

