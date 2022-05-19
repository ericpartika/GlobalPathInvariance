%%%%%%%%%% Experimental results of the Car-like robot %%
clc
clear all 
close all

k1 = 150; %
k2 = 150;
k3 = 110;%
k4 = 10;
k5 = 10;
v = 0.1;
L = 0.2;
r = 5;
% Import the file
newData1 = importdata('Est.txt');
%newData1 = importdata('chameleon_test_1-2m_radius.txt');


time = newData1.data(:,1);
dt = newData1.data(:,2);
x1 = newData1.data(:,3);
x2 = newData1.data(:,4);
x3 = newData1.data(:,5);
x4 = newData1.data(:,6);
x5 = newData1.data(:,7);
x6 = newData1.data(:,8);

t = zeros(length(dt),1);

for(i=1:length(dt))
t(i) = sum(dt(1:i));
end

% v_input = newData1.data(:,9);
% SteeingRate = newData1.data(:,10);

figure(1);
hold on;
plot(x1(1), x2(1), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'green');
plot(x1,x2, 'color','green','linewidth',2);
lambda = 0:0.01:2*pi;
 % desired path
plot(r*cos(lambda), r*sin(lambda), 'r--', 'linewidth',1);
xlabel('x_{1}(m)','FontSize',16);
ylabel('x_{2}(m)','FontSize',16);
grid on;
hold off;

%% Constructing the transformed states

xi1 = x1.^2 + x2.^2 - r^2;
xi2 = 2.*x1.*(v.*cos(x3) + x5.*cos(x3)) + 2.*x2.*(v.*sin(x3) + x5.*sin(x3));
xi3 = (2.*L.*v.^2 + 2.*L.*x5.^2 + 4.*L.*v.*x5 + 2.*v.^2.*x2.*cos(x3).*tan(x4) + 2.*L.*x1.*x6.*cos(x3) + 2.*x2.*x5.^2.*cos(x3).*tan(x4) - 2.*v.^2.*x1.*sin(x3).*tan(x4) + 2.*L.*x2.*x6.*sin(x3) - 2.*x1.*x5.^2.*sin(x3).*tan(x4) + 4.*v.*x2.*x5.*cos(x3).*tan(x4) - 4.*v.*x1.*x5.*sin(x3).*tan(x4))./L;

eta1 = atan2(x2,x1);
eta2 = ((v + x5).*(x1.*sin(x3) - x2.*cos(x3)))./(x1.^2 + x2.^2);
eta3 = (cos(x3).*(v + x5).^2.*(2.*cos(x3).*x1.*x2 - sin(x3).*x1.^2 + sin(x3).*x2.^2))./(x1.^2 + x2.^2).^2 - (x6.*(x2.*cos(x3) - x1.*sin(x3)))./(x1.^2 + x2.^2) - (sin(x3).*(v + x5).^2.*(cos(x3).*x1.^2 + 2.*sin(x3).*x1.*x2 - cos(x3).*x2.^2))./(x1.^2 + x2.^2).^2 + (tan(x4).*(v + x5).^2.*(x1.*cos(x3) + x2.*sin(x3)))./(L.*(x1.^2 + x2.^2));


figure(2);
plot(t,xi1,t,xi2,t,xi3);
xlabel('$t(sec)$','FontSize',16,'Interpreter','latex');
ylabel('$\xi_1,\xi_2,\xi_3$','FontSize',16,'Interpreter','latex');
l= legend(['$\xi_{1}$'],['$\xi_{2}$'],['$\xi_{3}$']);
set(l,'FontSize',16,'Interpreter','Latex');
grid on;


% % figure(3);
% % plot(t,xi2)
% % xlabel('$t(sec)$','FontSize',16,'Interpreter','latex');
% % ylabel('$\xi_2$','FontSize',16,'Interpreter','latex');
% % grid on;
% % 
% % figure(4);
% % plot(t,xi3)
% % xlabel('$t(sec)$','FontSize',16,'Interpreter','latex');
% % ylabel('$\xi_3$','FontSize',16,'Interpreter','latex');
% % grid on;


figure(5);
plot(t,eta1)
xlabel('$t(sec)$','FontSize',16,'Interpreter','latex');
ylabel('$\eta_1$','FontSize',16,'Interpreter','latex');
grid on;

figure(6);
plot(t,eta2)
xlabel('$t(sec)$','FontSize',16,'Interpreter','latex');
ylabel('$\eta_2$','FontSize',16,'Interpreter','latex');
grid on;

figure(7);
plot(t,eta3)
xlabel('$t(sec)$','FontSize',16,'Interpreter','latex');
ylabel('$\eta_3$','FontSize',16,'Interpreter','latex');
grid on;

%% Actual Inputs of the robotic Model
figure(8);
plot(t,v_input+v)
xlabel('$t(sec)$','FontSize',16,'Interpreter','latex');
ylabel('$v$','FontSize',16,'Interpreter','latex');
grid on;

figure(9);
plot(t,SteeingRate)
xlabel('$t(sec)$','FontSize',16,'Interpreter','latex');
ylabel('$\omega$(Steering Rate)','FontSize',16,'Interpreter','latex');
grid on;


%% Constructing control inputs from the states

Lf3S = zeros(1,length(t));


Lg1Lf2S = zeros(1,length(t));
Lg2Lf2S = zeros(1,length(t));
Lg1Lf2P = zeros(1,length(t));
Lg2Lf2P = zeros(1,length(t));
Lf3P = zeros(1,length(t));
u1 = zeros(1,length(t));
u2 = zeros(1,length(t));

pos = zeros(1,length(t));
consX5 = zeros(1,length(t));
consX6 = zeros(1,length(t));
Vx = zeros(1,length(t));
Vy = zeros(1,length(t));


for i=2:length(t)
Lg1Lf2S(i) = (2*x2(i)*(v*cos(x3(i)) + x5(i)*cos(x3(i))) - 2*x1(i)*(v*sin(x3(i)) + x5(i)*sin(x3(i))))*(v/(L*cos(x4(i))^2) + x5(i)/(L*cos(x4(i))^2));
Lg2Lf2S(i) = 2*x1(i)*cos(x3(i)) + 2*x2(i)*sin(x3(i));

Lg1Lf2P(i) = ((x1(i)*(v*cos(x3(i)) + x5(i)*cos(x3(i))) + x2(i)*(v*sin(x3(i)) + x5(i)*sin(x3(i))))*((v*(tan(x4(i))^2 + 1))/L + (x5(i)*(tan(x4(i))^2 + 1))/L))/(x1(i)^2 + x2(i)^2);
Lg2Lf2P(i) = -(x2(i)*cos(x3(i)) - x1(i)*sin(x3(i)))/(x1(i)^2 + x2(i)^2);

Lf3S(i) = -(2*v^3*x1(i)*cos(x3(i))*tan(x4(i))^2 - 6*L^2*x5(i)*x6(i) - 6*L^2*v*x6(i) + 2*x1(i)*x5(i)^3*cos(x3(i))*tan(x4(i))^2 + 2*v^3*x2(i)*sin(x3(i))*tan(x4(i))^2 + 2*x2(i)*x5(i)^3*sin(x3(i))*tan(x4(i))^2 + 6*v*x1(i)*x5(i)^2*cos(x3(i))*tan(x4(i))^2 + 6*v^2*x1(i)*x5(i)*cos(x3(i))*tan(x4(i))^2 + 6*v*x2(i)*x5(i)^2*sin(x3(i))*tan(x4(i))^2 + 6*v^2*x2(i)*x5(i)*sin(x3(i))*tan(x4(i))^2 - 6*L*v*x2(i)*x6(i)*cos(x3(i))*tan(x4(i)) - 6*L*x2(i)*x5(i)*x6(i)*cos(x3(i))*tan(x4(i)) + 6*L*v*x1(i)*x6(i)*sin(x3(i))*tan(x4(i)) + 6*L*x1(i)*x5(i)*x6(i)*sin(x3(i))*tan(x4(i)))/L^2;
Lf3P(i) = -((v + x5(i))*(tan(x4(i))*x1(i)^2 - 2*L*sin(x3(i))*x1(i) + tan(x4(i))*x2(i)^2 + 2*L*cos(x3(i))*x2(i))*(L*v^2*x1(i)^2 + L*v^2*x2(i)^2 + L*x1(i)^2*x5(i)^2 + L*x2(i)^2*x5(i)^2 + 2*L*v*x1(i)^2*x5(i) + 2*L*v*x2(i)^2*x5(i) + 2*L*v^2*x1(i)^2*cos(2*x3(i)) - 2*L*v^2*x2(i)^2*cos(2*x3(i)) + 2*L*x1(i)^2*x5(i)^2*cos(2*x3(i)) - 2*L*x2(i)^2*x5(i)^2*cos(2*x3(i)) - v^2*x2(i)^3*cos(x3(i))*tan(x4(i)) - x2(i)^3*x5(i)^2*cos(x3(i))*tan(x4(i)) - 3*L*x1(i)^3*x6(i)*cos(x3(i)) + v^2*x1(i)^3*sin(x3(i))*tan(x4(i)) + x1(i)^3*x5(i)^2*sin(x3(i))*tan(x4(i)) - 3*L*x2(i)^3*x6(i)*sin(x3(i)) - v^2*x1(i)^2*x2(i)*cos(x3(i))*tan(x4(i)) - x1(i)^2*x2(i)*x5(i)^2*cos(x3(i))*tan(x4(i)) - 3*L*x1(i)*x2(i)^2*x6(i)*cos(x3(i)) + v^2*x1(i)*x2(i)^2*sin(x3(i))*tan(x4(i)) + x1(i)*x2(i)^2*x5(i)^2*sin(x3(i))*tan(x4(i)) - 3*L*x1(i)^2*x2(i)*x6(i)*sin(x3(i)) + 4*L*v*x1(i)^2*x5(i)*cos(2*x3(i)) - 4*L*v*x2(i)^2*x5(i)*cos(2*x3(i)) + 4*L*v^2*x1(i)*x2(i)*sin(2*x3(i)) + 4*L*x1(i)*x2(i)*x5(i)^2*sin(2*x3(i)) - 2*v*x2(i)^3*x5(i)*cos(x3(i))*tan(x4(i)) + 2*v*x1(i)^3*x5(i)*sin(x3(i))*tan(x4(i)) - 2*v*x1(i)^2*x2(i)*x5(i)*cos(x3(i))*tan(x4(i)) + 2*v*x1(i)*x2(i)^2*x5(i)*sin(x3(i))*tan(x4(i)) + 8*L*v*x1(i)*x2(i)*x5(i)*sin(2*x3(i))))/(L^2*(x1(i)^2 + x2(i)^2)^3);    

    D=[Lg1Lf2P(i) Lg2Lf2P(i);Lg1Lf2S(i) Lg2Lf2S(i)];
    M=inv(D);
%det_d=det(D);
%v_tang = -k4*(eta2-0.5)-k5*(eta3); %%working controller for a constant (takes 1 or 2 min to execute)


   v_tang = -k4*(eta2(i)-0.5)-k5*(eta3(i));

   v_tran=-k1*xi1(i)-k2*xi2(i)-k3*xi3(i);
   
   U=M*[-Lf3P(i)+v_tang;-Lf3S(i)+v_tran];
   
   u1(i)=U(1);
   u2(i)=U(2);
   
   Vx(i) = ((x1(i) - x1(i-1))) /dt(i);
   Vy(i) = ((x2(i) - x2(i-1))) /dt(i);

   consX5(i) = sqrt(Vx(i)^2 + Vy(i)^2);
   
   if (i>4)
        consX5(i) = (consX5(i) +consX5(i-1) + consX5(i-2))/3;
   end
   
   %consX5(i) = (pos(i) - pos(i-1))/dt(i) - v;
   consX6(i) = (consX5(i) - consX5(i-1))/dt(i);

end

figure(10);
plot(t(2:end),consX5(2:end))
xlabel('$t(sec)$','FontSize',16,'Interpreter','latex');
ylabel('$v$','FontSize',16,'Interpreter','latex');
grid on;
hold on;
plot(t(2:end),x5(2:end))

% 
% figure(11);
% plot(t(2:end),u2(2:end))
% xlabel('$t(sec)$','FontSize',16,'Interpreter','latex');
% ylabel('$\omega$(Steering Rate)','FontSize',16,'Interpreter','latex');
% grid on;




