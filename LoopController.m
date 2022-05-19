%% PathFollowing of Car-like Robot
clc
clear all
close all


%% System Parameters
v = 0.1; %%% Not the linear velocity input but the fixed speed
const_a = 3;
k_int = 0.5;

k1 = 3; %
k2 = 15*const_a;
k3 = 15*const_a;%
k4 = 10;
k5 =  10;
L=0.27;
r=1.5; %% radius of the circle to follow

udp = udpport;

%% Instantiate client object to run Motive API commands
% https://optitrack.com/software/natnet-sdk/

% Create Motive client object
dllPath = fullfile('c:','Users','adakhtar', 'Desktop', 'NatNetSDK','lib','x64','NatNetML.dll');
assemblyInfo = NET.addAssembly(dllPath); % Add API function calls
theClient = NatNetML.NatNetClientML(0);

% Create connection to localhost, data is now being streamed through client object
HostIP = '127.0.0.1';
theClient.Initialize(HostIP, HostIP); 

Drone_ID = 1;


%%
%%% Initial Conditions
X1_0= 1; %% x-position
X2_0= 0;  %% y-position
X3_0=pi/2; %% car orientation
X4_0=0;    %% Wheel angle
X5_0=0; %% Fictitious state and we can always initialize it with zero
X6_0=0; %% Fictitious state and we can always initialize it with zero

v_input = 0;
steering_angle = 0;

% Initial condition Vector
x0 = [X1_0;X2_0;X3_0;X4_0;X5_0;X6_0]


% Simulation time
Tmax = 12;  % End point
dt =0.05; % Time step
T = 0:dt:Tmax; % Time vector

% Simulation setup
x_all = zeros(size(x0,1),length(T));  % Intialization of a matrix to save all the values of the states 
x_Old = x0; % Giving x0 as a the name of an old state to use it in the discretized equations
x_all(:,1) = x0; % Saving the initial state in the full state matrix
xi1_plot = zeros(1,length(T));
xi2_plot = zeros(1,length(T));
xi3_plot = zeros(1,length(T));
eta1_plot = zeros(1,length(T));
eta2_plot = zeros(1,length(T));
eta3_plot = zeros(1,length(T));
u1_plot = zeros(1,length(T));
u2_plot = zeros(1,length(T));

%%% Giving values to the individual componets for the ease of coding
x1_Old = x_Old(1);
x2_Old = x_Old(2);
x3_Old = x_Old(3);
x4_Old = x_Old(4);
x5_Old = x_Old(5);
x6_Old = x_Old(6);
v_input_Old = v_input;


%%% for the first iteration the current states are assumed to be the old
%%% states
x1 = x1_Old;
x2 = x2_Old;
x3 = x3_Old;
x4 = x4_Old;
x5 = x5_Old;
x6 = x6_Old;
vel = 120;

pause(1)

xi1_old = 0;

for i=1:length(T)-1
    %% Transformed states

    [DronePos] = GetDronePosition(theClient, Drone_ID);

    x1 = DronePos(2)
    x2 = DronePos(4)

%     x3_Old = x3;
    x3 = DronePos(6);
    if(x3 < 0)
        x3 = x3 + 2*pi;
    end
%     if(abs(x3 - x3_Old))
%         
%     end

%     x3_DEG = x3*(180/pi)

    x4 = steering_angle;

xi1 = - r^2 + x1^2 + x2^2;
xi2 = 2*(v + x5)*(x1*cos(x3) + x2*sin(x3));
xi3 = (v*sin(x3) + x5*sin(x3))*(2*v*sin(x3) + 2*x5*sin(x3)) + ((v*tan(x4))/L + (x5*tan(x4))/L)*(2*x2*(v*cos(x3) + x5*cos(x3)) - 2*x1*(v*sin(x3) + x5*sin(x3))) + x6*(2*x1*cos(x3) + 2*x2*sin(x3)) + (v*cos(x3) + x5*cos(x3))*(2*v*cos(x3) + 2*x5*cos(x3));
Lg1Lf2S = (2*x2*(v*cos(x3) + x5*cos(x3)) - 2*x1*(v*sin(x3) + x5*sin(x3)))*(v/(L*cos(x4)^2) + x5/(L*cos(x4)^2));
Lg2Lf2S = 2*x1*cos(x3) + 2*x2*sin(x3);
Lf3S = -(2*v^3*x1*cos(x3)*tan(x4)^2 - 6*L^2*x5*x6 - 6*L^2*v*x6 + 2*x1*x5^3*cos(x3)*tan(x4)^2 + 2*v^3*x2*sin(x3)*tan(x4)^2 + 2*x2*x5^3*sin(x3)*tan(x4)^2 + 6*v*x1*x5^2*cos(x3)*tan(x4)^2 + 6*v^2*x1*x5*cos(x3)*tan(x4)^2 + 6*v*x2*x5^2*sin(x3)*tan(x4)^2 + 6*v^2*x2*x5*sin(x3)*tan(x4)^2 - 6*L*v*x2*x6*cos(x3)*tan(x4) - 6*L*x2*x5*x6*cos(x3)*tan(x4) + 6*L*v*x1*x6*sin(x3)*tan(x4) + 6*L*x1*x5*x6*sin(x3)*tan(x4))/L^2;

eta1 = atan2(x2,x1);
eta2 = -(x2*(v*cos(x3) + x5*cos(x3)) - x1*(v*sin(x3) + x5*sin(x3)))/(x1^2 + x2^2);
eta3 = cos(x3)*(v + x5)*((sin(x3)*(v + x5))/(x1^2 + x2^2) - (2*x1^2*sin(x3)*(v + x5))/(x1^2 + x2^2)^2 + (2*x1*x2*cos(x3)*(v + x5))/(x1^2 + x2^2)^2) - x6*((x2*cos(x3))/(x1^2 + x2^2) - (x1*sin(x3))/(x1^2 + x2^2)) - sin(x3)*(v + x5)*((cos(x3)*(v + x5))/(x1^2 + x2^2) - (2*x2^2*cos(x3)*(v + x5))/(x1^2 + x2^2)^2 + (2*x1*x2*sin(x3)*(v + x5))/(x1^2 + x2^2)^2) + (tan(x4)*((x1*cos(x3)*(v + x5))/(x1^2 + x2^2) + (x2*sin(x3)*(v + x5))/(x1^2 + x2^2))*(v + x5))/L;
Lg1Lf2P = (((x1*cos(x3)*(v + x5))/(x1^2 + x2^2) + (x2*sin(x3)*(v + x5))/(x1^2 + x2^2))*(v + x5)*(tan(x4)^2 + 1))/L;
Lg2Lf2P =(x1*sin(x3))/(x1^2 + x2^2) - (x2*cos(x3))/(x1^2 + x2^2);
Lf3P = -((v + x5)*(tan(x4)*x1^2 - 2*L*sin(x3)*x1 + tan(x4)*x2^2 + 2*L*cos(x3)*x2)*(L*v^2*x1^2 + L*v^2*x2^2 + L*x1^2*x5^2 + L*x2^2*x5^2 + 2*L*v*x1^2*x5 + 2*L*v*x2^2*x5 + 2*L*v^2*x1^2*cos(2*x3) - 2*L*v^2*x2^2*cos(2*x3) + 2*L*x1^2*x5^2*cos(2*x3) - 2*L*x2^2*x5^2*cos(2*x3) - v^2*x2^3*cos(x3)*tan(x4) - x2^3*x5^2*cos(x3)*tan(x4) - 3*L*x1^3*x6*cos(x3) + v^2*x1^3*sin(x3)*tan(x4) + x1^3*x5^2*sin(x3)*tan(x4) - 3*L*x2^3*x6*sin(x3) - v^2*x1^2*x2*cos(x3)*tan(x4) - x1^2*x2*x5^2*cos(x3)*tan(x4) - 3*L*x1*x2^2*x6*cos(x3) + v^2*x1*x2^2*sin(x3)*tan(x4) + x1*x2^2*x5^2*sin(x3)*tan(x4) - 3*L*x1^2*x2*x6*sin(x3) + 4*L*v*x1^2*x5*cos(2*x3) - 4*L*v*x2^2*x5*cos(2*x3) + 4*L*v^2*x1*x2*sin(2*x3) + 4*L*x1*x2*x5^2*sin(2*x3) - 2*v*x2^3*x5*cos(x3)*tan(x4) + 2*v*x1^3*x5*sin(x3)*tan(x4) - 2*v*x1^2*x2*x5*cos(x3)*tan(x4) + 2*v*x1*x2^2*x5*sin(x3)*tan(x4) + 8*L*v*x1*x2*x5*sin(2*x3)))/(L^2*(x1^2 + x2^2)^3);

xi1_plot(i) = xi1;
xi2_plot(i) = xi2;
xi3_plot(i) = xi3;

eta1_plot(i) = eta1;
eta2_plot(i) = eta2;
eta3_plot(i) = eta3;


    %%
    %%% Control inputs
    D=[Lg1Lf2P Lg2Lf2P;Lg1Lf2S Lg2Lf2S];
    M=inv(D);
%     det_d=det(D)
    %v_tang = -k4*(eta2-0.5)-k5*(eta3); %%working controller for a constant (takes 1 or 2 min to execute)

    xi1_intg = xi1_old + xi1*dt;

    v_tang = -k4*(eta2-0.1)-k5*(eta3); % Trangential controller

    v_tran=-k1*xi1-k2*xi2-k3*xi3 -k_int*xi1_intg;  % Transversal controller 

    u=M*[-Lf3P+v_tang;-Lf3S+v_tran];
    u1=u(1);
    u2=u(2);

%%%%%%%%% Taking the derivative to get the real input
%%% Controller dynamics %%%
x6 = x6_Old + u2*dt;
x5 = x5_Old + dt*x6;
v_input = x5 + v;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% saturator = 1;
% 
% if u1 > saturator 
%     u1 =saturator ;
% end
% 
% if u1 < -saturator 
%     u1 =-saturator ;
% end
% 
% if v_input > saturator 
%     v_input =saturator ;
% end
% 
% if v_input < -saturator 
%     v_input =-saturator ;
% end

steering_angle = 1*steering_angle + u1*dt;

% steering_angle = steering_angle*2;
u1

if(steering_angle < -0.7)
    steering_angle = -0.7;
elseif(steering_angle > 0.35)
    steering_angle = 0.35;
end
u1_plot(i) = u1;
u2_plot(i) = u2;


% The system (Unicycle)   
%     x1 = x1_Old + dt*( v_input *cos(x3) );
%     x2 = x2_Old + dt*( v_input*sin(x3) );
%     x3 = x3_Old + dt*( (v_input/L)*tan(x4) );
%     x4 = x4_Old + u1*dt;

    
    %%% Making the state vector, that might be helpful for the debugging
    %%% purposes
    x = [x1;x2;x3;x4;x5;x6];
   
%     x_Old = x; %%% At the end of the an iteration the current values become the old values
%     x1_Old = x_Old(1);
%     x2_Old = x_Old(2);
%     x3_Old = x_Old(3);
%     x4_Old = x_Old(4);
%     x5_Old = x_Old(5);
%     x6_Old = x_Old(6);
    
    v_input_Old = v_input;
    xi1_old = xi1;
    
    %%% Saving the current values of the state to the full state matrix,
    %%% that might be useful for plotting purposes
    x_all(:,i+1) = x; 


    %% Send Controls
    if(i>50)
        vel = 90;
    end
%     input = [round(steering_angle*180/pi)+40, 120-(v_input*20)];
    input = [round(steering_angle*180/pi)+40, vel];
    input
    write(udp, input, 'uint8', '192.168.2.104', 10002);
    pause(dt);
    
   end

write(udp, [40,0], 'uint8', '192.168.2.104', 10002);

%% Plotting

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
    plot(x_all(1,:), x_all(2,:), 'r','color','red','linewidth',2);
    grid on;
hold off;


figure(3);
plot(T,xi1_plot,T,xi2_plot,T,xi3_plot)
title('\xi_1,\xi_2,\xi_3 versus time');
xlabel('t(sec)')
ylabel('position on the set \Gamma^{*}')
grid on;
legend('\xi_1','\xi_2','\xi_3')

figure(4);
plot(T,u1_plot)
title('u_1 versus time');
xlabel('t(sec)')
ylabel('Magnitude of the control input')
grid on;
legend('u_1')

figure(5);
plot(T,u2_plot)
title('u_2 versus time');
xlabel('t(sec)')
ylabel('Magnitude of the control input')
grid on;
legend('u_2')


