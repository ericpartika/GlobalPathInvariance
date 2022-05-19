%% Trajectory tracking
clc
clear all
close all


%% System Parameters
v = 0.1; %%% Not the linear velocity input but the fixed speed
k1 = 10; %
k2 = 10;
k3 = 10;%
k4 = 50;
k5 = 10;
L=0.2;
r=5; %% radius of the circle to follow

SystemParameters.Length = 0.2;
SystemParameters.FixedVelocity =0.1;
Gains.k1 = 10; 
Gains.k2 = 10; 
Gains.k3 = 10;
Gains.k4 = 30;
Gains.k5 = 10;


%% Initial Conditions
X1_0= r+ 0.5; %% x-position
X2_0= 0;  %% y-position
X3_0=pi/2; %% car orientation
X4_0=0;    %% Wheel angle
X5_0=0; %% Fictitious state and we can always initialize it with zero
X6_0=0; %% Fictitious state and we can always initialize it with zero

x0 = [X1_0;X2_0;X3_0;X4_0;X5_0;X6_0]


%% Simulation time
Tmax = 40;  % End point
dt =0.01; % Time step
T = 0:dt:Tmax; % Time vector

%% Nonlinear Control
[x,TransStates,Belief, PredictedBelief,consX5,consX6] = Control(T,dt,x0,SystemParameters,Gains,r);

%% Plotting

x1 = x.x1;
x2 = x.x2;
x3 = x.x3;
x4 = x.x4;
x5 = x.x5;
x6 = x.x6;

% Trajectory
figure(1);
hold on;
lambda = -pi:0.01:pi;
    % desired path
    %plot(lambda, cos(lambda), 'r--', 'linewidth',1);
    plot(r*cos(lambda), r*sin(lambda), 'r--','color','green', 'linewidth',3);
    title('Unicycle Following circle with different initial conditions')
    xlabel('x_{1}')
    ylabel('x_{2}')
    %%% displaying the initial marker
    plot(x0(1), x0(2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'red');
    plot(x1(1:end-1), x2(1:end-1), 'r','color','red','linewidth',2);
    plot(Belief.x1(1:end-1), Belief.x2(1:end-1), 'r','color','blue','linewidth',2);
    plot(PredictedBelief.x1(1:end-1), PredictedBelief.x2(1:end-1), 'r--','color','cyan','linewidth',2);
    grid on;
hold off;


figure(3);
plot(T,TransStates.xi1,T,TransStates.xi2,T,TransStates.xi3)
title('\xi_1,\xi_2,\xi_3 versus time');
xlabel('t(sec)')
ylabel('position on the set \Gamma^{*}')
grid on;
legend('\xi_1','\xi_2','\xi_3')

% figure(4);
% plot(T,TransStates.eta2)
% title('\eta_2 versus time');
% xlabel('t(sec)')
% ylabel('Velocity of the robot')
% grid on;
% %legend('\xi_1','\xi_2','\xi_3')

figure(4);
plot(T,TransStates.eta2)
xlabel('$t(sec)$','FontSize',16,'Interpreter','latex')
ylabel('$\eta_2','FontSize',16,'Interpreter','latex')
l= legend(['$\eta_2$']);
set(l,'FontSize',16,'Interpreter','Latex');
grid on;

figure(5);
plot(T,TransStates.eta1)
xlabel('$t(sec)$','FontSize',16,'Interpreter','latex')
ylabel('$\eta_1','FontSize',16,'Interpreter','latex')
l= legend(['$\eta_1$']);
set(l,'FontSize',16,'Interpreter','Latex');
grid on;

figure(6);
plot(T,TransStates.eta3)
xlabel('$t(sec)$','FontSize',16,'Interpreter','latex')
ylabel('$\eta_3','FontSize',16,'Interpreter','latex')
l= legend(['$\eta_3$']);
set(l,'FontSize',16,'Interpreter','Latex');
grid on;



% % % % figure(4);
% % % % plot(T,u1_plot)
% % % % title('u_1 versus time');
% % % % xlabel('t(sec)')
% % % % ylabel('Magnitude of the control input')
% % % % grid on;
% % % % legend('u_1')
% % % % 
% % % % figure(5);
% % % % plot(T,u2_plot)
% % % % title('u_2 versus time');
% % % % xlabel('t(sec)')
% % % % ylabel('Magnitude of the control input')
% % % % grid on;
% % % % legend('u_2')
% % % 
% % % figure(6);
% % % plot(T,x_all(5,:),T,consX5)
% % % title('x_5 vs Constructed x5 using differentiaion');
% % % xlabel('t(sec)')
% % % ylabel('Magnitude of the control input')
% % % grid on;
% % % legend('x_5','x_5 (Constructed)')
% % 

%%%%%%%%% Graphs for the constructed states and the actual states

figure(7);
hold on;
plot(T(1:end),x5(1:end), 'r','color','green', 'linewidth',3)
plot(T(1:end),consX5(1:end), 'r--','color','red', 'linewidth',2)
plot(T(1:end),PredictedBelief.x5(1:end), 'r--','color','cyan', 'linewidth',2)
plot(T(1:end),Belief.x5(1:end), 'r--','color','blue', 'linewidth',2)
title('x_5 vs Constructed x5 using differentiaion');
xlabel('t(sec)')
ylabel('Magnitude of the control input')
grid on;
legend('x_5','x_5 (Constructed)')
hold off;
%% 
figure(8);
hold on;
plot(T(1:end),x6(1:end), 'r','color','green', 'linewidth',3)
plot(T(1:end),consX6(1:end), 'r--','color','red', 'linewidth',2)
plot(T(1:end),PredictedBelief.x6(1:end), 'r--','color','cyan', 'linewidth',2)
plot(T(1:end),Belief.x6(1:end), 'r--','color','blue', 'linewidth',2)
title('x_6 vs Constructed x6 using differentiaion');
xlabel('t(sec)')
ylabel('Magnitude of the control input')
grid on;
legend('x_6','x_6 (Constructed)')
hold off;

figure(9);
subplot(4,1,1);
hold on;
plot(T(1:end),x1(1:end), 'r','color','green', 'linewidth',3)
%plot(T(1:end),PredictedBelief.x1(1:end), 'r--','color','cyan', 'linewidth',2)
plot(T(1:end),Belief.x1(1:end), 'r--','color','blue', 'linewidth',2)
xlabel('t(sec)')
ylabel('(m)')
grid on;
hold off

subplot(4,1,2);
hold on;
plot(T(1:end),x2(1:end), 'r','color','green', 'linewidth',3)
%plot(T(1:end),PredictedBelief.x2(1:end), 'r--','color','cyan', 'linewidth',2)
plot(T(1:end),Belief.x2(1:end), 'r--','color','blue', 'linewidth',2)
xlabel('t(sec)')
ylabel('(m)')
grid on;
hold off;

subplot(4,1,3);
hold on;
plot(T(1:end),x3(1:end), 'r','color','green', 'linewidth',3)
%plot(T(1:end),PredictedBelief.x2(1:end), 'r--','color','cyan', 'linewidth',2)
plot(T(1:end),Belief.x3(1:end), 'r--','color','blue', 'linewidth',2)
xlabel('t(sec)')
ylabel('(m)')
grid on;
hold off;

subplot(4,1,4);
hold on;
plot(T(1:end),x4(1:end), 'r','color','green', 'linewidth',3)
%plot(T(1:end),PredictedBelief.x2(1:end), 'r--','color','cyan', 'linewidth',2)
plot(T(1:end),Belief.x4(1:end), 'r--','color','blue', 'linewidth',2)
xlabel('t(sec)')
ylabel('(m)')
grid on;
hold off;

figure(10);
subplot(2,1,1);
hold on;
plot(T(1:end),x5(1:end), 'r','color','green', 'linewidth',3)
%plot(T(1:end),PredictedBelief.x1(1:end), 'r--','color','cyan', 'linewidth',2)
plot(T(1:end),Belief.x5(1:end), 'r--','color','blue', 'linewidth',2)
xlabel('t(sec)')
ylabel('(m)')
grid on;
hold off

subplot(2,1,2);
hold on;
plot(T(1:end),x6(1:end), 'r','color','green', 'linewidth',3)
%plot(T(1:end),PredictedBelief.x2(1:end), 'r--','color','cyan', 'linewidth',2)
plot(T(1:end),Belief.x6(1:end), 'r--','color','blue', 'linewidth',2)
xlabel('t(sec)')
ylabel('(m)')
grid on;
hold off;





