
#ifndef RobotSim_H
#define RobotSim_H

#include "ros/ros.h"
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/Core>

using namespace Eigen;
using Eigen::MatrixXd;
using namespace std;

class RobotSim{
private:
  double L;
public:
    double SimX1 ;
    double SimX2 ;
    double SimX3 ;
    double SimX4 ;
    double SimX5 ;
    double SimX6 ;
 
    void simulation (double x1, double x2, double x3, double x4, double x5, double x6, double dt, double Vinput, double u1, double u2);
    RobotSim();	
    void SimulationInitialization(double x1, double x2, double x3, double x4, double x5, double x6, double length);
    int age;
    void bark();
};

#endif
