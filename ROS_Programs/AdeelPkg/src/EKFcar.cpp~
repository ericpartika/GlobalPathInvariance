#include <iostream>
#include "EKFcar.h"
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/Core>
#include "ros/ros.h"

using namespace Eigen;
using Eigen::MatrixXd;
using namespace std;

void EKFcar::bark() {
	std::cout << "Woof!" << std::endl;
}

void EKFcar::PredictionUpdate(double Vinput, double SteeringRate, double VirtualInput, double dt)
{
  
  mu1 = mu(0,0);
  mu2 = mu(1,0);
  mu3 = mu(2,0);
  mu4 = mu(3,0);
  mu5 = mu(4,0);
  mu6 = mu(5,0);
  
  MatrixXd Ad(6,6);
  
  
  Ad <<  1, 0,  (-v*sin(mu3) - mu5*sin(mu3))*dt,                                                    0,       cos(mu3)*dt,    0,
         0, 1,  ( v*cos(mu3) + mu5*cos(mu3))*dt,                                                    0,       sin(mu3)*dt,    0,
         0, 0,                             1,  ((v*(tan(mu4)*tan(mu4) + 1))/L + (mu5*(tan(mu4)*tan(mu4) + 1))/L)*dt,   (tan(mu4)/L)*dt,    0,
         0, 0,                             0,                                                    1,                0,        0,
         0, 0,                             0,                                                    0,                1,        1*dt,
         0, 0,                             0,                                                    0,                0,        1;
    
    mup1 = mu1 + dt*( v*cos(mu3)+ mu5*cos(mu3) );
    mup2 = mu2 + dt*( v*sin(mu3)+ mu5*sin(mu3) );
    mup3 = mu3 + dt*((v/L)*tan(mu4)+(mu5/L)*tan(mu4));
    mup4 = mu4 + dt*SteeringRate;
    mup5 = mu5 + dt*mu6;
    mup6 = mu6 + dt*VirtualInput;	
    
    mup << mup1, mup2, mup3, mup4, mup5, mup6;
    
    Sp = Ad*S*Ad.transpose() + R;
    
}

void EKFcar::MesurementUpdate(double measX1, double measX2, double measX3)
{

  Matrix<double, 6, 3> K = Sp*Ht.transpose()*( (Ht*Sp*Ht.transpose()+Q).inverse() );
  
  Matrix<double, 3, 1> MeasureDiff;
  
  MeasureDiff << measX1-mup1, measX2-mup2, measX3-mup3; 
  
  mu = mup + K*(MeasureDiff);
  
  S = ( MatrixXd::Identity(6,6) -K*Ht)*Sp;
  
  estx1 = mu(0,0);
  estx2 = mu(1,0);
  estx3 = mu(2,0);
  estx4 = mu(3,0);
  estx5 = mu(4,0);
  estx6 = mu(5,0);
  
}

//Constructor
EKFcar::EKFcar()
{
  cout << "Adeel ";
   mu1 = 0;
   mu2 = 0;
   mu3 = 0;
   mu4 = 0;
   mu5 = 0;
   mu6 = 0;
  
  //declared for the time being, it has to replaced!!!
   //v = 0.1;
   //L = 1; 

   u1 = 0;
   u2 = 0;
   
   double ds = 0.00001;
     S << ds, 0, 0, 0, 0, 0,
     0, ds, 0, 0, 0, 0,
     0, 0, ds, 0, 0, 0,
     0, 0, 0, ds*10, 0, 0,
     0, 0, 0, 0, ds*10, 0,
     0, 0, 0, 0, 0, ds*100;
     
     // Disturbance model
     R = 0.07*S;
     
    //Measurement model defined below
    Q << 1e-3,             0,              0,
                 0,     1e-3,              0,
                 0,             0,      1e-3;
		 
    //Linearization of Measurement Model
    Ht << 1,   0,   0, 0, 0, 0,
          0,   1,   0, 0, 0, 0,
          0,   0,   1, 0, 0, 0;		 
   
   
}

// This function must be called only once to give initial estimate to the EKF
// The first three measured states comes from the IPS
void EKFcar::InitializeEKF(double mX1, double mX2, double mX3, double Fixedpush, double Lenght)
{
  measX1 = mX1;
  measX2 = mX2;
  measX3 = mX3;
   
  mu1 = measX1;
  mu2 = measX2;
  mu3 = measX3;
  mu4 = 0;
  mu5 = 0;
  mu6 = 0;
  
  mu << mu1, mu2, mu3, mu4, mu5, mu6;
  
  //declared for the time being, it has to replaced!!!
   v = Fixedpush;
   L = Lenght; 
   u1 = 0;
   u2 = 0;
}
