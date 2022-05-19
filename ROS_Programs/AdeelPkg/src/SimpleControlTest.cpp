
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <Eigen/Dense>
#include <fstream>

using namespace Eigen;
using namespace std;

#define PI 3.14159265

ofstream myfile;
int mycount = 0;

    int radius = 5;
    double x1 = radius+ 0.5;
    double x2 = 0.002;
    double x3 = PI/2;
    double x4 = 0;
    double x5 = 0.1;
    double x6 = 0;
    double mytime = 0;

void Control (double T)
{

    
  double v_tran, v_tang;

  
  double v = 0.1;
  double L = 0.2;
  double vfixed = v;
  
  
  double k1 = 10;
  double k2 = 10;
  double k3 = 10;
  double k4 = 10;
  double k5 = 10;
  double dt = 0.01;
  
  mytime = mytime + dt;
  
  double xi1 =   -radius*radius + x1*x1 + x2*x2;
  double xi2 =    (vfixed+x5)*(x1*cos(x3)+x2*sin(x3))*2.0;
  double xi3 =    (vfixed*sin(x3)+x5*sin(x3))*(vfixed*sin(x3)*2.0+x5*sin(x3)*2.0)+((vfixed*tan(x4))/L+(x5*tan(x4))/L)*(x2*(vfixed*cos(x3)+x5*cos(x3))*2.0-x1*(vfixed*sin(x3)+x5*sin(x3))*2.0)+x6*(x1*cos(x3)*2.0+x2*sin(x3)*2.0)+(vfixed*cos(x3)+x5*cos(x3))*(vfixed*cos(x3)*2.0+x5*cos(x3)*2.0);

  double Lg1Lf2S =    (x2*(vfixed*cos(x3)+x5*cos(x3))*2.0-x1*(vfixed*sin(x3)+x5*sin(x3))*2.0)*((vfixed*1.0/pow(cos(x4),2.0))/L+(x5*1.0/pow(cos(x4),2.0))/L);
  double Lg2Lf2S =    x1*cos(x3)*2.0+x2*sin(x3)*2.0;
  double Lf3S =    -1.0/(L*L)*((L*L)*vfixed*x6*-6.0-(L*L)*x5*x6*6.0+(vfixed*vfixed*vfixed)*x1*cos(x3)*pow(tan(x4),2.0)*2.0+x1*(x5*x5*x5)*cos(x3)*pow(tan(x4),2.0)*2.0+(vfixed*vfixed*vfixed)*x2*sin(x3)*pow(tan(x4),2.0)*2.0+x2*(x5*x5*x5)*sin(x3)*pow(tan(x4),2.0)*2.0+vfixed*x1*(x5*x5)*cos(x3)*pow(tan(x4),2.0)*6.0+(vfixed*vfixed)*x1*x5*cos(x3)*pow(tan(x4),2.0)*6.0+vfixed*x2*(x5*x5)*sin(x3)*pow(tan(x4),2.0)*6.0+(vfixed*vfixed)*x2*x5*sin(x3)*pow(tan(x4),2.0)*6.0-L*vfixed*x2*x6*cos(x3)*tan(x4)*6.0-L*x2*x5*x6*cos(x3)*tan(x4)*6.0+L*vfixed*x1*x6*sin(x3)*tan(x4)*6.0+L*x1*x5*x6*sin(x3)*tan(x4)*6.0);

  double eta1 =    atan(x2/x1);
  double eta2 =    -(x2*(vfixed*cos(x3)+x5*cos(x3))-x1*(vfixed*sin(x3)+x5*sin(x3)))/(x1*x1+x2*x2);
  double eta3 =    ((vfixed*sin(x3)+x5*sin(x3))/(x1*x1+x2*x2)+x1*1.0/pow(x1*x1+x2*x2,2.0)*(x2*(vfixed*cos(x3)+x5*cos(x3))-x1*(vfixed*sin(x3)+x5*sin(x3)))*2.0)*(vfixed*cos(x3)+x5*cos(x3))-(vfixed*sin(x3)+x5*sin(x3))*((vfixed*cos(x3)+x5*cos(x3))/(x1*x1+x2*x2)-x2*1.0/pow(x1*x1+x2*x2,2.0)*(x2*(vfixed*cos(x3)+x5*cos(x3))-x1*(vfixed*sin(x3)+x5*sin(x3)))*2.0)-(x6*(x2*cos(x3)-x1*sin(x3)))/(x1*x1+x2*x2)+(((vfixed*tan(x4))/L+(x5*tan(x4))/L)*(x1*(vfixed*cos(x3)+x5*cos(x3))+x2*(vfixed*sin(x3)+x5*sin(x3))))/(x1*x1+x2*x2);

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
        
    v_tang = -k4*(eta2 - 0.5) - k5*(eta3);
    v_tran = -k1*xi1 - k2*xi2 - k3*xi3;

    //ROS_INFO("v_tang=%f, v_tran=%f", v_tang, v_tran);


    VectorXd G(2);
    G(0) = -Lf3P + v_tang;
    G(1) = -Lf3S + v_tran;

    VectorXd U(2);
    U = M * G;
    double u2 = U(1);
    double u1 = U(0);
    
    x6 = x6 + u2*dt;
    x5 = x5 + x6*dt;
    
    double Vinput = v + x5;
    
    ROS_INFO("Control Inputs: %f %f\n",u1,u2);
    
    // The system (Unicycle)   
    x1 = x1 + dt*( (Vinput)*cos(x3) );
    x2 = x2 + dt*( (Vinput)*sin(x3) );
    x3 = x3 + dt*( ((Vinput)/L)*tan(x4) );
    x4 = x4 + u1*dt;
    
    ROS_INFO("Est States: %f, %f, %f, %f, %f, %f", x1, x2, x3, x4, x5, x6);
    myfile << mytime << " "<< dt << " "<< x1 << " "<< x2 << " "<< x3 << " "<< x4 << " "<< x5 << " "<< x6 << " "<< std::endl;
         mycount = mycount + 1;
    
    //outfile << data << endl;
   
        
  
   
  //std_msgs::String msg;
  //std::stringstream ss; 
  //ss << "Adeel ";
  //msg.data = ss.str();
  //ROS_INFO("%s", msg.data.c_str());
  ROS_INFO("FBL'\n");
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "talker");


  ros::NodeHandle n;



  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(100);

  double T = 0;
  
  
   myfile.open ("chameleon_test.txt", ios::out | ios::app);
   myfile << "time,dt,x1,x2,x3,x4,x5,x6\n";
//   myfile.close();
  
  while (ros::ok())
  {
    
    Control(T);
    
//     std_msgs::String msg;
//     std::stringstream ss;
//     ss << "Hello World:D " << count;
//     msg.data = ss.str();
//     ROS_INFO("%s", msg.data.c_str());
//     chatter_pub.publish(msg);
     ros::spinOnce();
     loop_rate.sleep();

     
  }

  myfile.close();
  return 0;
}

