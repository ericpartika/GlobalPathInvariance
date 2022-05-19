#include <iostream>
#include <math.h>

#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/Core>
#include "ros/ros.h"
#include "RobotSim.h"

using namespace Eigen;
using Eigen::MatrixXd;
using namespace std;

void RobotSim::simulation (double x1, double x2, double x3, double x4, double x5, double x6, double dt, double Vinput, double u1, double u2)
{
  // The system (Unicycle)   
  x1 = x1 + dt*( (Vinput)*cos(x3) ) ;
  x2 = x2 + dt*( (Vinput)*sin(x3) ) ;
  x3 = x3 + dt*( ((Vinput)/L)*tan(x4) ) ;
  x4 = x4 + u1*dt ;
  // Adding noise to the fictitious states
  x5 = x5 + x6*dt;
  x6 = x6 + u2*dt;
  
  SimX1 = x1;
  SimX2 = x2;
  SimX3 = x3;
  SimX4 = x4;
  SimX5 = x5;
  SimX6 = x6;
 
}

RobotSim::RobotSim()
{
//     SimX1 = 0;
//     SimX2 = 0;
//     SimX3 = 0;
//     SimX4 = 0;
//     SimX5 = 0;
//     SimX6 = 0;
  
}

void RobotSim::SimulationInitialization(double x1, double x2, double x3, double x4, double x5, double x6, double Lenght)
{
  SimX1 = x1;
  SimX2 = x2;
  SimX3 = x3;
  SimX4 = x4;
  SimX5 = x5;
  SimX6 = x6;
  L = Lenght;
}