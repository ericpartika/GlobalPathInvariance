%%%
% This is a workspace for visualizing the vehicle, unsafe set, and target
% set
%%%
clear all
close all

L = 0.3; % meters
lr = L/2;
load('targetSet.mat')

%% Initial state

% limits:
% velocity: 0.5 m/s
% steering angle: +- 20deg

x = [1; 5; 0; 0]; % z = (x_pos; y_pos; theta; delta) x and y are rear axle of vehicle


u = [0, 0]'; % u = (v, omega) omega is steering angle rate 

%% vehicle model
% zdot
x1dot = u(1) * cos(x(3)); % v*cos(theta + beta)
x2dot = u(1) * sin(x(3)); % v*sin(theta + beta)
x3dot = u(1) * (tan(x(4)))/(L); % v*(tan(delta))/L; L is length of vehicle
x4dot = u(2);

x = x + [x1dot; x2dot; x3dot; x4dot];

%% Target Set
 % x_all contains the target set of states for a sine wave. Ignore rows 5
 % and 6
load('targetSet.mat')
x_all = x_all(:, 1:2000);

%% Unsafe set

U = [1,2,0.5,0.5;
     3,2.5,0.5,0.5;
     4,1,0.5,0.5];

%% Plotting

t = 0:pi/20:2*pi;

figure(1)
hold on
axis equal
quiver(x(1),x(2), (L*cos(x(3))), (L*sin(x(3))), 0, 'linewidth',3)
plot(x_all(1,:), x_all(2,:))
plot(t, sin(t))
rectangle('Position',U(1,:),'FaceColor',[1 0 0])
rectangle('Position',U(2,:),'FaceColor',[1 0 0])
rectangle('Position',U(3,:),'FaceColor',[1 0 0])

