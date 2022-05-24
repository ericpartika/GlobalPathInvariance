%% PathFollowing of Car-like Robot
clc
clear all
close all


%% System Parameters
v = 0.1; %%% Not the linear velocity input but the fixed speed
k_int = 1;

k1 = 3; %
k2 = 45;
k3 = 45;%
k4 = 10;
k5 =  10;
L=0.27;
r=1.5; %% radius of the circle to follow


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


%% Initialization
udp = udpport;

v_input = 0;
steering_angle = 0;

pause(1)
[DronePos] = GetDronePosition(theClient, Drone_ID);

x1 = DronePos(2);
x2 = DronePos(4);
x3 = DronePos(6);
if(x3 < 0)
    x3 = x3 + 2*pi;
end
x4 = steering_angle;
x5 = 0;
x6 = 0;
vel = 160;

x0 = [x1;x2;x3;x4;x5;x6]

% Simulation time
Tmax = 5;  % End point
dt =0.05; % Time step
T = 0:dt:Tmax; % Time vector

% Simulation setup
x_all = zeros(size(x0,1),length(T));  % Intialization of a matrix to save all the values of the states 
x_all(:,1) = x0; % Saving the initial state in the full state matrix
xi1_plot = zeros(1,length(T));
xi2_plot = zeros(1,length(T));
xi3_plot = zeros(1,length(T));
eta1_plot = zeros(1,length(T));
eta2_plot = zeros(1,length(T));
eta3_plot = zeros(1,length(T));
u1_plot = zeros(1,length(T));
u2_plot = zeros(1,length(T));
steeringangle_plot = zeros(1,length(T));

%%% Giving values to the individual componets for the ease of coding
x1_Old = x1;
x2_Old = x2;
x3_Old = x3;
x4_Old = x4;
x5_Old = x5;
x6_Old = x6;
v_input_Old = v_input;
xi1_old = 0;

lowpass_gain = 0.2;

%% Contol Loop
for i=1:length(T)
    % get measurments
    x1_Old = x1;
    x2_Old = x2;
    x3_Old = x3;
    
    [DronePos] = GetDronePosition(theClient, Drone_ID);
    x1m = DronePos(2); % x
    x2m = DronePos(4); % y
    x3m = DronePos(6); % theta
    if(x3m < 0)
        x3m = x3m + 2*pi;
    end
    x1 = x1_Old*lowpass_gain + (1-lowpass_gain)*x1m
    x2 = x2_Old*lowpass_gain + (1-lowpass_gain)*x2m
    x3 = x3_Old*lowpass_gain + (1-lowpass_gain)*x3m;
    x4 = steering_angle;

    % Transformed states
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

    %%% Control inputs
    D=[Lg1Lf2P Lg2Lf2P;Lg1Lf2S Lg2Lf2S];
    M=inv(D);
    %     det_d=det(D)

    xi1_intg = xi1_old + xi1*dt;
    v_tang = -k4*(eta2-0.1)-k5*(eta3); % Trangential controller
    v_tran=-k1*xi1-k2*xi2-k3*xi3 -k_int*xi1_intg;  % Transversal controller

    u=M*[-Lf3P+v_tang;-Lf3S+v_tran];
    u1=u(1);
    u2=u(2);

    x6 = x6 + u2*dt;
    x5 = x5 + dt*x6;
    v_input = x5 + v;
    steering_angle = 1*steering_angle + u1*dt;

    % saturate +- 20deg
    if(steering_angle < -0.35)
        steering_angle = -0.35;
    elseif(steering_angle > 0.35)
        steering_angle = 0.35;
    end

    % saving state for plots
    steeringangle_plot(i) = steering_angle;
    u1_plot(i) = u1;
    u2_plot(i) = u2;
    x = [x1;x2;x3;x4;x5;x6];
    xi1_old = xi1;
    x_all(:,i) = x;

    % Send Controls
    if(i>50)
        vel = 150;
    end
    %     input = [round(steering_angle*180/pi)+40, 120-(v_input*20)];
    input = [round(steering_angle*180/pi)+40, vel];
    write(udp, input, 'uint8', '192.168.2.101', 10002);

    % Loop at 5ms
    pause(dt);
   end

% stop car
write(udp, [40,0], 'uint8', '192.168.2.101', 10002);

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
    plot(x_all(1,1), x_all(2,1), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'red');
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

date = datestr(now, 'dd_mm_yy_HH_MM');

% save(['Logs/',date, '.mat'], 'i', 'k1', 'k2', 'k3', 'x_all', 'u1_plot', 'u2_plot', 'dt', 'eta1_plot', 'eta2_plot', 'eta3_plot')


