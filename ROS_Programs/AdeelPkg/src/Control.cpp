#include <iostream>
#include "Control.h"
#include <math.h>

#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/Core>
#include "ros/ros.h"

using namespace Eigen;
using Eigen::MatrixXd;
using namespace std;


void Control::ControlLaw(double x1, double x2, double x3, double x4, double x5, double x6, double vfixed, double L, double radius)
{

   xi1 =   -radius*radius + x1*x1 + x2*x2;
   xi2 =    (vfixed+x5)*(x1*cos(x3)+x2*sin(x3))*2.0;
   xi3 =    (vfixed*sin(x3)+x5*sin(x3))*(vfixed*sin(x3)*2.0+x5*sin(x3)*2.0)+((vfixed*tan(x4))/L+(x5*tan(x4))/L)*(x2*(vfixed*cos(x3)+x5*cos(x3))*2.0-x1*(vfixed*sin(x3)+x5*sin(x3))*2.0)+x6*(x1*cos(x3)*2.0+x2*sin(x3)*2.0)+(vfixed*cos(x3)+x5*cos(x3))*(vfixed*cos(x3)*2.0+x5*cos(x3)*2.0);

  double Lg1Lf2S =    (x2*(vfixed*cos(x3)+x5*cos(x3))*2.0-x1*(vfixed*sin(x3)+x5*sin(x3))*2.0)*((vfixed*1.0/pow(cos(x4),2.0))/L+(x5*1.0/pow(cos(x4),2.0))/L);
  double Lg2Lf2S =    x1*cos(x3)*2.0+x2*sin(x3)*2.0;
  double Lf3S =    -1.0/(L*L)*((L*L)*vfixed*x6*-6.0-(L*L)*x5*x6*6.0+(vfixed*vfixed*vfixed)*x1*cos(x3)*pow(tan(x4),2.0)*2.0+x1*(x5*x5*x5)*cos(x3)*pow(tan(x4),2.0)*2.0+(vfixed*vfixed*vfixed)*x2*sin(x3)*pow(tan(x4),2.0)*2.0+x2*(x5*x5*x5)*sin(x3)*pow(tan(x4),2.0)*2.0+vfixed*x1*(x5*x5)*cos(x3)*pow(tan(x4),2.0)*6.0+(vfixed*vfixed)*x1*x5*cos(x3)*pow(tan(x4),2.0)*6.0+vfixed*x2*(x5*x5)*sin(x3)*pow(tan(x4),2.0)*6.0+(vfixed*vfixed)*x2*x5*sin(x3)*pow(tan(x4),2.0)*6.0-L*vfixed*x2*x6*cos(x3)*tan(x4)*6.0-L*x2*x5*x6*cos(x3)*tan(x4)*6.0+L*vfixed*x1*x6*sin(x3)*tan(x4)*6.0+L*x1*x5*x6*sin(x3)*tan(x4)*6.0);

   eta1 =    atan(x2/x1);
   eta2 =    -(x2*(vfixed*cos(x3)+x5*cos(x3))-x1*(vfixed*sin(x3)+x5*sin(x3)))/(x1*x1+x2*x2);
   eta3 =    ((vfixed*sin(x3)+x5*sin(x3))/(x1*x1+x2*x2)+x1*1.0/pow(x1*x1+x2*x2,2.0)*(x2*(vfixed*cos(x3)+x5*cos(x3))-x1*(vfixed*sin(x3)+x5*sin(x3)))*2.0)*(vfixed*cos(x3)+x5*cos(x3))-(vfixed*sin(x3)+x5*sin(x3))*((vfixed*cos(x3)+x5*cos(x3))/(x1*x1+x2*x2)-x2*1.0/pow(x1*x1+x2*x2,2.0)*(x2*(vfixed*cos(x3)+x5*cos(x3))-x1*(vfixed*sin(x3)+x5*sin(x3)))*2.0)-(x6*(x2*cos(x3)-x1*sin(x3)))/(x1*x1+x2*x2)+(((vfixed*tan(x4))/L+(x5*tan(x4))/L)*(x1*(vfixed*cos(x3)+x5*cos(x3))+x2*(vfixed*sin(x3)+x5*sin(x3))))/(x1*x1+x2*x2);

  double Lg1Lf2P =    ((x1*(vfixed*cos(x3)+x5*cos(x3))+x2*(vfixed*sin(x3)+x5*sin(x3)))*((vfixed*(pow(tan(x4),2.0)+1.0))/L+(x5*(pow(tan(x4),2.0)+1.0))/L))/(x1*x1+x2*x2);
  double Lg2Lf2P =    -(x2*cos(x3)-x1*sin(x3))/(x1*x1+x2*x2);
  double Lf3P =    -1.0/(L*L)*1.0/pow(x1*x1+x2*x2,3.0)*(vfixed+x5)*((x1*x1)*tan(x4)+(x2*x2)*tan(x4)-L*x1*sin(x3)*2.0+L*x2*cos(x3)*2.0)*(L*(vfixed*vfixed)*(x1*x1)+L*(vfixed*vfixed)*(x2*x2)+L*(x1*x1)*(x5*x5)+L*(x2*x2)*(x5*x5)+L*vfixed*(x1*x1)*x5*2.0+L*vfixed*(x2*x2)*x5*2.0+L*(vfixed*vfixed)*(x1*x1)*cos(x3*2.0)*2.0-L*(vfixed*vfixed)*(x2*x2)*cos(x3*2.0)*2.0+L*(x1*x1)*(x5*x5)*cos(x3*2.0)*2.0-L*(x2*x2)*(x5*x5)*cos(x3*2.0)*2.0-(vfixed*vfixed)*(x2*x2*x2)*cos(x3)*tan(x4)-(x2*x2*x2)*(x5*x5)*cos(x3)*tan(x4)-L*(x1*x1*x1)*x6*cos(x3)*3.0+(vfixed*vfixed)*(x1*x1*x1)*sin(x3)*tan(x4)+(x1*x1*x1)*(x5*x5)*sin(x3)*tan(x4)-L*(x2*x2*x2)*x6*sin(x3)*3.0-(vfixed*vfixed)*(x1*x1)*x2*cos(x3)*tan(x4)-(x1*x1)*x2*(x5*x5)*cos(x3)*tan(x4)-L*x1*(x2*x2)*x6*cos(x3)*3.0+(vfixed*vfixed)*x1*(x2*x2)*sin(x3)*tan(x4)+x1*(x2*x2)*(x5*x5)*sin(x3)*tan(x4)-L*(x1*x1)*x2*x6*sin(x3)*3.0+L*vfixed*(x1*x1)*x5*cos(x3*2.0)*4.0-L*vfixed*(x2*x2)*x5*cos(x3*2.0)*4.0+L*(vfixed*vfixed)*x1*x2*sin(x3*2.0)*4.0+L*x1*x2*(x5*x5)*sin(x3*2.0)*4.0-vfixed*(x2*x2*x2)*x5*cos(x3)*
tan(x4)*2.0+vfixed*(x1*x1*x1)*x5*sin(x3)*tan(x4)*2.0-vfixed*(x1*x1)*x2*x5*cos(x3)*tan(x4)*2.0+vfixed*x1*(x2*x2)*x5*sin(x3)*tan(x4)*2.0+L*vfixed*x1*x2*x5*sin(x3*2.0)*8.0);
    
   // Control inputs
    MatrixXd D(2,2);
    D(0,0) = Lg1Lf2P;
    D(0,1) = Lg2Lf2P;
    D(1,0) = Lg1Lf2S;
    D(1,1) = Lg2Lf2S;
        
    double detD = D.determinant();
    //ROS_INFO("%f\n",detD);
        
    MatrixXd M(2,2);
    M = D.inverse();
        
    double v_tang = -k4*(eta2 - 0.5) - k5*(eta3);
    double v_tran = -k1*xi1 - k2*xi2 - k3*xi3;

    //ROS_INFO("v_tang=%f, v_tran=%f", v_tang, v_tran);

    VectorXd G(2);
    G(0) = -Lf3P + v_tang;
    G(1) = -Lf3S + v_tran;

    VectorXd U(2);
    U = M * G;
    
    double u1 = U(0);
    double u2 = U(1);
    
    
    //cout<< "u1 =" << u1;
    
    Vinput = vfixed + x5;
    //cout<< "u1 =" << u1;
    SteeringRateInput = u1; 
    VirtualInput = u2;
}


Control::Control()//constructor
{
  
  k1 = 30;
  k2 = 20;
  k3 = 10;
  k4 = 30;
  k5 = 20;
  vfixed = 0.1;
  L = 0.2;
  radius = 0.2;
}