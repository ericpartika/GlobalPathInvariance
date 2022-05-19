function [State,TransStates,Belief, PredictedBelief, consX5,consX6] = Control( T,dt,x0,SystemParameters,Gains,r )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here



L = SystemParameters.Length;
v = SystemParameters.FixedVelocity;
k1 = Gains.k1; 
k2 = Gains.k2; 
k3 = Gains.k3;
k4 = Gains.k4;
k5 = Gains.k5;

u1 = 0;
u2 = 0;
Vinput = 0;

% Simulation setup
u1_plot = zeros(1,length(T));
u2_plot = zeros(1,length(T));

TransStates.xi1 = zeros(1,length(T));
TransStates.xi2 = zeros(1,length(T));
TransStates.xi3 = zeros(1,length(T));

TransStates.eta1 = zeros(1,length(T));
TransStates.eta2 = zeros(1,length(T));
TransStates.eta3 = zeros(1,length(T));

State.x1 = zeros(1,length(T));
State.x2 = zeros(1,length(T));
State.x3 = zeros(1,length(T));
State.x4 = zeros(1,length(T));
State.x5 = zeros(1,length(T));
State.x6 = zeros(1,length(T));

Belief.x1 = zeros(1,length(T));
Belief.x2 = zeros(1,length(T));
Belief.x3 = zeros(1,length(T));
Belief.x4 = zeros(1,length(T));
Belief.x5 = zeros(1,length(T));
Belief.x6 = zeros(1,length(T));

PredictedBelief.x1 = zeros(1,length(T));
PredictedBelief.x2 = zeros(1,length(T));
PredictedBelief.x3 = zeros(1,length(T));
PredictedBelief.x4 = zeros(1,length(T));
PredictedBelief.x5 = zeros(1,length(T));
PredictedBelief.x6 = zeros(1,length(T));

Measurement.y1 = zeros(1,length(T));
Measurement.y2 = zeros(1,length(T));
Measurement.y3 = zeros(1,length(T));

% State.x1(1:2) = x0(1); 
% State.x2(1:2) = x0(2);
% State.x3(1:2) = x0(3);
% State.x4(1:2) = x0(4);
% State.x5(1:2) = x0(5);
% State.x6(1:2) = x0(6);


State.x1(1) = x0(1); 
State.x2(1) = x0(2);
State.x3(1) = x0(3);
State.x4(1) = x0(4);
State.x5(1) = x0(5);
State.x6(1) = x0(6);

State.x1(2) = x0(1); 
State.x2(2) = x0(2) + 0.002; 
State.x3(2) = x0(3); 
State.x4(2) = x0(4); 
State.x5(2) = x0(5); 
State.x6(2) = x0(6); 

Belief = State;


x1 = x0(1);
x2 = x0(2);
x3 = x0(3);
x4 = x0(4);
x5 = x0(5);
x6 = x0(6);


consX5 = zeros(1,length(T));
consX5(1) = x5;

consX6 = zeros(1,length(T));
consX6(1) = x6;

Vx = zeros(1,length(T));
Vy = zeros(1,length(T));

%% EKF setup

% Prior
%mu = [X1_0 X2_0 X3_0 X4_0 X5_0 X6_0]'; % mean (mu)
mu = x0; % mean (mu)
mup = mu;
% S = 0.0001*eye(6);% covariance (Sigma)
S = 0.00001*eye(6);% covariance (Sigma)

% Discrete system
Ad = zeros(6,6);
Cd = zeros(3,6);
Bd = zeros(6,2);

%%% For simulation 
% Disturbance model
R = 0.03*S;
[RE, Re] = eig (R);

% Measurement model defined below
Q = [1e-4 0      0;
     0     1e-4 0
     0     0      1e-4];%
n = length(Ad(1,:));
m = length(Q(:,1));


%%% For the EKF



Cd = [1 0 0 0 0 0;
      0 1 0 0 0 0
      0 0 1 0 0 0];
  
    mu1 = mu(1);
    mu2 = mu(2);
    mu3 = mu(3);
    mu4 = mu(4);
    mu5 = mu(5);
    mu6 = mu(6);
    
    est_x1 = mu1;
    est_x2 = mu2;
    est_x3 = mu3;
    est_x4 = mu4;
    

    Belief = State;
for i=2:length(T)-1
    %% Construction of X5 and X6 states from the position (x1,x2)    

    %if (i<=30/dt) 
        
                % Vx(i) = ((State.x1(i) - State.x1(i-1))) /dt;
                % Vy(i) = ((State.x2(i) - State.x2(i-1))) /dt;
                
                Vx(i) = ((Belief.x1(i) - Belief.x1(i-1))) /dt;
                Vy(i) = ((Belief.x2(i) - Belief.x2(i-1))) /dt;

                consX5(i) = sqrt(Vx(i)^2 + Vy(i)^2) - v;
                consX6(i) = (consX5(i) - consX5(i-1))/dt;

                x5 = consX5(i);
                x6 = consX6(i);
                
                est_x5 = x5;
                est_x6 = x6;

                %%% Transformed states

%                 xi1 =    -r*r+x1*x1+x2*x2;
%                 xi2 =    (v+x5)*(x1*cos(x3)+x2*sin(x3))*2.0;
%                 xi3 =    (v*sin(x3)+x5*sin(x3))*(v*sin(x3)*2.0+x5*sin(x3)*2.0)+((v*tan(x4))/L+(x5*tan(x4))/L)*(x2*(v*cos(x3)+x5*cos(x3))*2.0-x1*(v*sin(x3)+x5*sin(x3))*2.0)+x6*(x1*cos(x3)*2.0+x2*sin(x3)*2.0)+(v*cos(x3)+x5*cos(x3))*(v*cos(x3)*2.0+x5*cos(x3)*2.0);
%                 Lg1Lf2S =    (x2*(v*cos(x3)+x5*cos(x3))*2.0-x1*(v*sin(x3)+x5*sin(x3))*2.0)*((v*1.0/pow(cos(x4),2.0))/L+(x5*1.0/pow(cos(x4),2.0))/L);
%                 Lg2Lf2S =    x1*cos(x3)*2.0+x2*sin(x3)*2.0;
%                 Lf3S =    -1.0/(L*L)*((L*L)*v*x6*-6.0-(L*L)*x5*x6*6.0+(v*v*v)*x1*cos(x3)*pow(tan(x4),2.0)*2.0+x1*(x5*x5*x5)*cos(x3)*pow(tan(x4),2.0)*2.0+(v*v*v)*x2*sin(x3)*pow(tan(x4),2.0)*2.0+x2*(x5*x5*x5)*sin(x3)*pow(tan(x4),2.0)*2.0+v*x1*(x5*x5)*cos(x3)*pow(tan(x4),2.0)*6.0+(v*v)*x1*x5*cos(x3)*pow(tan(x4),2.0)*6.0+v*x2*(x5*x5)*sin(x3)*pow(tan(x4),2.0)*6.0+(v*v)*x2*x5*sin(x3)*pow(tan(x4),2.0)*6.0-L*v*x2*x6*cos(x3)*tan(x4)*6.0-L*x2*x5*x6*cos(x3)*tan(x4)*6.0+L*v*x1*x6*sin(x3)*tan(x4)*6.0+L*x1*x5*x6*sin(x3)*tan(x4)*6.0);
%                 eta1 =    atan(x2/x1);
%                 eta2 =    -(x2*(v*cos(x3)+x5*cos(x3))-x1*(v*sin(x3)+x5*sin(x3)))/(x1*x1+x2*x2);
%                 eta3 =    ((v*sin(x3)+x5*sin(x3))/(x1*x1+x2*x2)+x1*1.0/pow(x1*x1+x2*x2,2.0)*(x2*(v*cos(x3)+x5*cos(x3))-x1*(v*sin(x3)+x5*sin(x3)))*2.0)*(v*cos(x3)+x5*cos(x3))-(v*sin(x3)+x5*sin(x3))*((v*cos(x3)+x5*cos(x3))/(x1*x1+x2*x2)-x2*1.0/pow(x1*x1+x2*x2,2.0)*(x2*(v*cos(x3)+x5*cos(x3))-x1*(v*sin(x3)+x5*sin(x3)))*2.0)-(x6*(x2*cos(x3)-x1*sin(x3)))/(x1*x1+x2*x2)+(((v*tan(x4))/L+(x5*tan(x4))/L)*(x1*(v*cos(x3)+x5*cos(x3))+x2*(v*sin(x3)+x5*sin(x3))))/(x1*x1+x2*x2);
%                 Lg1Lf2P =    ((x1*(v*cos(x3)+x5*cos(x3))+x2*(v*sin(x3)+x5*sin(x3)))*((v*(pow(tan(x4),2.0)+1.0))/L+(x5*(pow(tan(x4),2.0)+1.0))/L))/(x1*x1+x2*x2);
%                 Lg2Lf2P =    -(x2*cos(x3)-x1*sin(x3))/(x1*x1+x2*x2);
%                 Lf3P =    -1.0/(L*L)*1.0/pow(x1*x1+x2*x2,3.0)*(v+x5)*((x1*x1)*tan(x4)+(x2*x2)*tan(x4)-L*x1*sin(x3)*2.0+L*x2*cos(x3)*2.0)*(L*(v*v)*(x1*x1)+L*(v*v)*(x2*x2)+L*(x1*x1)*(x5*x5)+L*(x2*x2)*(x5*x5)+L*v*(x1*x1)*x5*2.0+L*v*(x2*x2)*x5*2.0+L*(v*v)*(x1*x1)*cos(x3*2.0)*2.0-L*(v*v)*(x2*x2)*cos(x3*2.0)*2.0+L*(x1*x1)*(x5*x5)*cos(x3*2.0)*2.0-L*(x2*x2)*(x5*x5)*cos(x3*2.0)*2.0-(v*v)*(x2*x2*x2)*cos(x3)*tan(x4)-(x2*x2*x2)*(x5*x5)*cos(x3)*tan(x4)-L*(x1*x1*x1)*x6*cos(x3)*3.0+(v*v)*(x1*x1*x1)*sin(x3)*tan(x4)+(x1*x1*x1)*(x5*x5)*sin(x3)*tan(x4)-L*(x2*x2*x2)*x6*sin(x3)*3.0-(v*v)*(x1*x1)*x2*cos(x3)*tan(x4)-(x1*x1)*x2*(x5*x5)*cos(x3)*tan(x4)-L*x1*(x2*x2)*x6*cos(x3)*3.0+(v*v)*x1*(x2*x2)*sin(x3)*tan(x4)+x1*(x2*x2)*(x5*x5)*sin(x3)*tan(x4)-L*(x1*x1)*x2*x6*sin(x3)*3.0+L*v*(x1*x1)*x5*cos(x3*2.0)*4.0-L*v*(x2*x2)*x5*cos(x3*2.0)*4.0+L*(v*v)*x1*x2*sin(x3*2.0)*4.0+L*x1*x2*(x5*x5)*sin(x3*2.0)*4.0-v*(x2*x2*x2)*x5*cos(x3)*tan(x4)*2.0+v*(x1*x1*x1)*x5*sin(x3)*tan(x4)*2.0-v*(x1*x1)*x2*x5*cos(x3)*tan(x4)*2.0+v*x1*(x2*x2)*x5*sin(x3)*tan(x4)*2.0+L*v*x1*x2*x5*sin(x3*2.0)*8.0);
      %end
    

%         
%          Vx(i) = ((Belief.x1(i-1) - Belief.x1(i-2))) /dt;
%          Vy(i) = ((Belief.x2(i-1) - Belief.x2(i-2))) /dt;
% 
%          consX5(i) = sqrt(Vx(i)^2 + Vy(i)^2) - v;
%          consX6(i) = (consX5(i) - consX5(i-1))/dt;
% 
%          est_x5 = consX5(i);
%          est_x6 = consX6(i);
%         
        xi1 =    -r*r+est_x1*est_x1+est_x2*est_x2;
        xi2 =    (v+est_x5)*(est_x1*cos(est_x3)+est_x2*sin(est_x3))*2.0;
        xi3 =    (v*sin(est_x3)+est_x5*sin(est_x3))*(v*sin(est_x3)*2.0+est_x5*sin(est_x3)*2.0)+((v*tan(est_x4))/L+(est_x5*tan(est_x4))/L)*(est_x2*(v*cos(est_x3)+est_x5*cos(est_x3))*2.0-est_x1*(v*sin(est_x3)+est_x5*sin(est_x3))*2.0)+est_x6*(est_x1*cos(est_x3)*2.0+est_x2*sin(est_x3)*2.0)+(v*cos(est_x3)+est_x5*cos(est_x3))*(v*cos(est_x3)*2.0+est_x5*cos(est_x3)*2.0);
        Lg1Lf2S =    (est_x2*(v*cos(est_x3)+est_x5*cos(est_x3))*2.0-est_x1*(v*sin(est_x3)+est_x5*sin(est_x3))*2.0)*((v*1.0/pow(cos(est_x4),2.0))/L+(est_x5*1.0/pow(cos(est_x4),2.0))/L);
        Lg2Lf2S =    est_x1*cos(est_x3)*2.0+est_x2*sin(est_x3)*2.0;
        Lf3S =    -1.0/(L*L)*((L*L)*v*est_x6*-6.0-(L*L)*est_x5*est_x6*6.0+(v*v*v)*est_x1*cos(est_x3)*pow(tan(est_x4),2.0)*2.0+est_x1*(est_x5*est_x5*est_x5)*cos(est_x3)*pow(tan(est_x4),2.0)*2.0+(v*v*v)*est_x2*sin(est_x3)*pow(tan(est_x4),2.0)*2.0+est_x2*(est_x5*est_x5*est_x5)*sin(est_x3)*pow(tan(est_x4),2.0)*2.0+v*est_x1*(est_x5*est_x5)*cos(est_x3)*pow(tan(est_x4),2.0)*6.0+(v*v)*est_x1*est_x5*cos(est_x3)*pow(tan(est_x4),2.0)*6.0+v*est_x2*(est_x5*est_x5)*sin(est_x3)*pow(tan(est_x4),2.0)*6.0+(v*v)*est_x2*est_x5*sin(est_x3)*pow(tan(est_x4),2.0)*6.0-L*v*est_x2*est_x6*cos(est_x3)*tan(est_x4)*6.0-L*est_x2*est_x5*est_x6*cos(est_x3)*tan(est_x4)*6.0+L*v*est_x1*est_x6*sin(est_x3)*tan(est_x4)*6.0+L*est_x1*est_x5*est_x6*sin(est_x3)*tan(est_x4)*6.0);
        eta1 =    atan(est_x2/est_x1);
        eta2 =    -(est_x2*(v*cos(est_x3)+est_x5*cos(est_x3))-est_x1*(v*sin(est_x3)+est_x5*sin(est_x3)))/(est_x1*est_x1+est_x2*est_x2);
        eta3 =    ((v*sin(est_x3)+est_x5*sin(est_x3))/(est_x1*est_x1+est_x2*est_x2)+est_x1*1.0/pow(est_x1*est_x1+est_x2*est_x2,2.0)*(est_x2*(v*cos(est_x3)+est_x5*cos(est_x3))-est_x1*(v*sin(est_x3)+est_x5*sin(est_x3)))*2.0)*(v*cos(est_x3)+est_x5*cos(est_x3))-(v*sin(est_x3)+est_x5*sin(est_x3))*((v*cos(est_x3)+est_x5*cos(est_x3))/(est_x1*est_x1+est_x2*est_x2)-est_x2*1.0/pow(est_x1*est_x1+est_x2*est_x2,2.0)*(est_x2*(v*cos(est_x3)+est_x5*cos(est_x3))-est_x1*(v*sin(est_x3)+est_x5*sin(est_x3)))*2.0)-(est_x6*(est_x2*cos(est_x3)-est_x1*sin(est_x3)))/(est_x1*est_x1+est_x2*est_x2)+(((v*tan(est_x4))/L+(est_x5*tan(est_x4))/L)*(est_x1*(v*cos(est_x3)+est_x5*cos(est_x3))+est_x2*(v*sin(est_x3)+est_x5*sin(est_x3))))/(est_x1*est_x1+est_x2*est_x2);
        Lg1Lf2P =    ((est_x1*(v*cos(est_x3)+est_x5*cos(est_x3))+est_x2*(v*sin(est_x3)+est_x5*sin(est_x3)))*((v*(pow(tan(est_x4),2.0)+1.0))/L+(est_x5*(pow(tan(est_x4),2.0)+1.0))/L))/(est_x1*est_x1+est_x2*est_x2);
        Lg2Lf2P =    -(est_x2*cos(est_x3)-est_x1*sin(est_x3))/(est_x1*est_x1+est_x2*est_x2);
        Lf3P =    -1.0/(L*L)*1.0/pow(est_x1*est_x1+est_x2*est_x2,3.0)*(v+est_x5)*((est_x1*est_x1)*tan(est_x4)+(est_x2*est_x2)*tan(est_x4)-L*est_x1*sin(est_x3)*2.0+L*est_x2*cos(est_x3)*2.0)*(L*(v*v)*(est_x1*est_x1)+L*(v*v)*(est_x2*est_x2)+L*(est_x1*est_x1)*(est_x5*est_x5)+L*(est_x2*est_x2)*(est_x5*est_x5)+L*v*(est_x1*est_x1)*est_x5*2.0+L*v*(est_x2*est_x2)*est_x5*2.0+L*(v*v)*(est_x1*est_x1)*cos(est_x3*2.0)*2.0-L*(v*v)*(est_x2*est_x2)*cos(est_x3*2.0)*2.0+L*(est_x1*est_x1)*(est_x5*est_x5)*cos(est_x3*2.0)*2.0-L*(est_x2*est_x2)*(est_x5*est_x5)*cos(est_x3*2.0)*2.0-(v*v)*(est_x2*est_x2*est_x2)*cos(est_x3)*tan(est_x4)-(est_x2*est_x2*est_x2)*(est_x5*est_x5)*cos(est_x3)*tan(est_x4)-L*(est_x1*est_x1*est_x1)*est_x6*cos(est_x3)*3.0+(v*v)*(est_x1*est_x1*est_x1)*sin(est_x3)*tan(est_x4)+(est_x1*est_x1*est_x1)*(est_x5*est_x5)*sin(est_x3)*tan(est_x4)-L*(est_x2*est_x2*est_x2)*est_x6*sin(est_x3)*3.0-(v*v)*(est_x1*est_x1)*est_x2*cos(est_x3)*tan(est_x4)-(est_x1*est_x1)*est_x2*(est_x5*est_x5)*cos(est_x3)*tan(est_x4)-L*est_x1*(est_x2*est_x2)*est_x6*cos(est_x3)*3.0+(v*v)*est_x1*(est_x2*est_x2)*sin(est_x3)*tan(est_x4)+est_x1*(est_x2*est_x2)*(est_x5*est_x5)*sin(est_x3)*tan(est_x4)-L*(est_x1*est_x1)*est_x2*est_x6*sin(est_x3)*3.0+L*v*(est_x1*est_x1)*est_x5*cos(est_x3*2.0)*4.0-L*v*(est_x2*est_x2)*est_x5*cos(est_x3*2.0)*4.0+L*(v*v)*est_x1*est_x2*sin(est_x3*2.0)*4.0+L*est_x1*est_x2*(est_x5*est_x5)*sin(est_x3*2.0)*4.0-v*(est_x2*est_x2*est_x2)*est_x5*cos(est_x3)*tan(est_x4)*2.0+v*(est_x1*est_x1*est_x1)*est_x5*sin(est_x3)*tan(est_x4)*2.0-v*(est_x1*est_x1)*est_x2*est_x5*cos(est_x3)*tan(est_x4)*2.0+v*est_x1*(est_x2*est_x2)*est_x5*sin(est_x3)*tan(est_x4)*2.0+L*v*est_x1*est_x2*est_x5*sin(est_x3*2.0)*8.0);
%     

    %%% Saving the transformed states for plotting purposes
    TransStates.xi1(i) = xi1;
    TransStates.xi2(i) = xi2;
    TransStates.xi3(i) = xi3;

    TransStates.eta1(i) = eta1;
    TransStates.eta2(i) = eta2;
    TransStates.eta3(i) = eta3;
    
    %% Control inputs
    D=[Lg1Lf2P Lg2Lf2P;Lg1Lf2S Lg2Lf2S];
    %M=inv(D);

    v_tang = -k4*(eta2-0.5)-k5*(eta3);
    v_tran=-k1*xi1-k2*xi2-k3*xi3;

    u=D\[-Lf3P+v_tang;-Lf3S+v_tran];
    u1=u(1);
    u2=u(2);
    
    

    saturator = 1;

    if u1 > saturator 
        u1 =saturator ;
    end

    if u1 < -saturator 
        u1 =-saturator ;
    end

    if u2 > saturator 
        u2 =saturator ;
    end

    if u2 < -saturator 
        u2 =-saturator ;
    end


    u1_plot(i) = u1;
    u2_plot(i) = u2;
    
    Vinput = v + x5;
    


 
    
    % Select a motion disturbance
    e = RE*sqrt(Re)*randn(n,1);

    % The system (Unicycle)   
        x1 = x1 + dt*( (Vinput)*cos(x3) ) + e(1);
        x2 = x2 + dt*( (Vinput)*sin(x3) ) + e(2);
        x3 = x3 + dt*( ((Vinput)/L)*tan(x4) ) + e(3);
        x4 = x4 + u1*dt + e(4);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         %x5 = x5_Old + x6*dt; %%% Fictitius state
         %x6 = x6_Old + u2*dt; %%% Fictitius state
         % Adding noise to the fictitious states

        x5 = x5 + e(5);
        x6 = x6 + e(6);
         
        State.x1(i+1) = x1 ;
        State.x2(i+1) = x2 ;
        State.x3(i+1) = x3 ;
        State.x4(i+1) = x4 ;
        State.x5(i+1) = x5 ;
        State.x6(i+1) = x6 ;
     
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% New Added Stuff %%%%%%%%%%%%%%%%%%%     
    
    % Select a motion disturbance \\\ done above
    %e = RE*sqrt(Re)*randn(n,1);  \\\ done above
    % Update state
%     x(:,t) = Ad*x(:,t-1) + e;  \\\ done above
% 
    % Take measurement
    % Select a measurement NOISE
    %d = sqrt(Q)*randn(m,1);
    d = sqrt(Q)*((randn(m,1)));
    % Determine measurement
    %y(:,t) = Cd*[x1; x2; x3;x4;x5;x6] + d;
    measure = Cd*[x1; x2; x3;x4;x5;x6] + d;
    
    Measurement.y1(i) = measure(1);
    Measurement.y2(i) = measure(2);
    Measurement.y3(i) = measure(3);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% New Added Stuff Ends %%%%%%%%%%%%%%%%     
        
    %% Extended Kalman Filter
    %%%% Kalman filter can only use the knowledge of two things
    %%%% 1) Control inputs
    %%%% 2) Measurements
    
    % Prediction update
    Ad=[ 1, 0,  (-v*sin(mu3) - mu5*sin(mu3))*dt,                                                     0,       cos(mu3)*dt,    0;
         0, 1,  ( v*cos(mu3) + mu5*cos(mu3))*dt,                                                    0,       sin(mu3)*dt,    0;
         0, 0,                             1,  ((v*(tan(mu4)^2 + 1))/L + (mu5*(tan(mu4)^2 + 1))/L)*dt,   (tan(mu4)/L)*dt,    0;
         0, 0,                             0,                                                    1,                0,    0;
         0, 0,                             0,                                                    0,                1, 1*dt;
         0, 0,                             0,                                                    0,                0,    1];


    
    
    mup1 = mu1 + dt*( v*cos(mu3)+ mu5*cos(mu3) );
    mup2 = mu2 + dt*( v*sin(mu3)+ mu5*sin(mu3) );
    mup3 = mu3 + dt*((v/L)*tan(mu4)+(mu5/L)*tan(mu4));
    mup4 = mu4 + dt*u1;
    mup5 = mu5 + dt*mu6;
    mup6 = mu6 + dt*u2;    
    
    mup = [mup1;mup2;mup3;mup4;mup5;mup6];
    
    Sp = Ad*S*Ad' + R;
    
    % Linearization
    Ht = [1   0   0 0 0 0;
          0   1   0 0 0 0;
          0   0   1 0 0 0];
    % Measurement update
    K = Sp*Ht'*inv(Ht*Sp*Ht'+Q);
    %mu = mup + K*(y(:,t)-sqrt(mup(1)^2 + mup(3)^2));
    mu = mup + K*(measure-[mup(1);mup(2);mup(3)]);
    S = (eye(n)-K*Ht)*Sp;
   
    
    % Store results
        PredictedBelief.x1(i+1) = mup(1);
        PredictedBelief.x2(i+1) = mup(2);
        PredictedBelief.x3(i+1) = mup(3);
        PredictedBelief.x4(i+1) = mup(4);
        PredictedBelief.x5(i+1) = mup(5);
        PredictedBelief.x6(i+1) = mup(6);

        Belief.x1(i+1) = mu(1);
        Belief.x2(i+1) = mu(2);
        Belief.x3(i+1) = mu(3);
        Belief.x4(i+1) = mu(4);
        Belief.x5(i+1) = mu(5);
        Belief.x6(i+1) = mu(6);
        
        %%% Making the belief available for the next iteration of the loop

        mu1 = mu(1);
        mu2 = mu(2);
        mu3 = mu(3);
        mu4 = mu(4);
        mu5 = mu(5);
        mu6 = mu(6);
        
    est_x1 = mu1;
    est_x2 = mu2;
    est_x3 = mu3;
    est_x4 = mu4;
    %est_x5 = mu5;
    %est_x6 = mu6;
        

        
%         mup_S(:,t+1) = mup;
%         mu_S(:,t+1) = mu;

   end


end

