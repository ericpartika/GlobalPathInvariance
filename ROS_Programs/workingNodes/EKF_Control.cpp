
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <Eigen/Dense>
#include <fstream>
#include <unsupported/Eigen/MatrixFunctions>

#include "Dog.h"

using namespace Eigen;
using namespace std;

#define PI 3.14159265

ofstream myfile;
int mycount = 0;
double v = 0.1;
double L = 0.2;


int DiffCount = 0;
int radius = 5;
    

class State{
  public:
    double x1, x2, x3, x4, x5, x6;
    void set_values(double,double,double,double,double,double);
  } CurrentState, PreviousState, UpdatedState, CurrentBelief, PreviousBelief, UpdatedBelief; 

void State::set_values(double a, double b, double c, double d, double e, double f)
{
  x1 = a;
  x2 = b;
  x3 = c;
  x4 = d;
  x5 = e;
  x6 = f;
}  


void RobotModel(double &mytime, double dt, State &CurrentBelief, State &CurrentState, double IntegVelocityInput, double SteeringRateInput)
{
  // Note that the input of this model is rate of change of steering while the input of the chameleon is steering angle.
  
  double x1 = CurrentBelief.x1;
  double x2 = CurrentBelief.x2;
  double x3 = CurrentBelief.x3;
  double x4 = CurrentBelief.x4;
  double x5 = CurrentBelief.x5;
  double x6 = CurrentBelief.x6; 
  
  // The system (Unicycle)   
    x1 = x1 + dt*( (IntegVelocityInput)*cos(x3) );
    x2 = x2 + dt*( (IntegVelocityInput)*sin(x3) );
    x3 = x3 + dt*( ((IntegVelocityInput)/L)*tan(x4) );
    x4 = x4 + SteeringRateInput*dt;
    
 CurrentBelief.x1 = x1;
 CurrentBelief.x2 = x2;
 CurrentBelief.x3 = x3;
 CurrentBelief.x4 = x4;

    
    ROS_INFO("Est States: %f, %f, %f, %f, %f, %f", x1, x2, x3, x4, x5, x6);
     myfile << mytime << " "<< dt << " "<< x1 << " "<< x2 << " "<< x3 << " "<< x4 << " "<< x5 << " "<< x6 << " "<< std::endl;

}
    
void Diff (double dt, State &CurrentState, State &CurrentBelief, State PreviousBelief, double &DiffVelocityInput)
{
  if (DiffCount == 0)
  {
    double Vx = ( ( CurrentBelief.x1 + 0.0004*cos(CurrentBelief.x3) )-  CurrentBelief.x1 ) /dt;
    double Vy = ( ( CurrentBelief.x2 + 0.0004*sin(CurrentBelief.x3) ) - CurrentBelief.x2 )/dt;
    
    double consX5 = sqrt(Vx*Vx + Vy*Vy) - v;
    double consX6 = (consX5 - CurrentBelief.x5)/dt;
    
    CurrentBelief.x5 = consX5;
    CurrentBelief.x6 = consX6;
    
  }
  else
  {
    double Vx = ( CurrentBelief.x1 - PreviousBelief.x1 ) /dt;
    double Vy = ( CurrentBelief.x2 - PreviousBelief.x2 ) /dt; 
  
    double consX5 = sqrt(Vx*Vx + Vy*Vy) - v;
    double consX6 = (consX5 - CurrentBelief.x5)/dt;
 
    CurrentState.x5 = consX5;
    CurrentState.x6 = consX6;
    DiffVelocityInput = v + consX5;
  }
  DiffCount = DiffCount + 1;  
}



void Control (double &mytime, double dt, double T, State &CurrentBelief, double &IntegVelocityInput, double &SteeringRateInput)
{
    
  
  double x1 = CurrentBelief.x1;
  double x2 = CurrentBelief.x2;
  double x3 = CurrentBelief.x3;
  double x4 = CurrentBelief.x4;
  double x5 = CurrentBelief.x5;
  double x6 = CurrentBelief.x6;  
  
  double v_tran, v_tang;    

  double vfixed = v;
  
  double k1 = 10;
  double k2 = 10;
  double k3 = 10;
  double k4 = 10;
  double k5 = 10;
  
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
    

    SteeringRateInput = u1;
    
    x6 = x6 + u2*dt;
    x5 = x5 + x6*dt;
    IntegVelocityInput = v + x5;
    
    //These two lines should be commented out
    CurrentBelief.x5 = x5;
    CurrentBelief.x6 = x6; 
    
    ROS_INFO("Control Inputs: %f %f\n",u1,u2);
    
    
    //outfile << data << endl;
   
  //std_msgs::String msg;
  //std::stringstream ss; 
  //ss << "Adeel ";
  //msg.data = ss.str();
  //ROS_INFO("%s", msg.data.c_str());
  ROS_INFO("FBL'\n");
}



void EKF(double dt, State CurrentBelief, const MatrixXd& Q, const MatrixXd &Cd, const MatrixXd &Sp, const MatrixXd &S,const MatrixXd& R, double u1, double u2)
{

   MatrixXd mat = MatrixXd::Random(5,1);
   MatrixXd d(5,1);
   d = Q.sqrt()*mat;
   
   double mu1 = CurrentBelief.x1;
   double mu2 = CurrentBelief.x2;
   double mu3 = CurrentBelief.x3;
   double mu4 = CurrentBelief.x4;
   double mu5 = CurrentBelief.x5;
   double mu6 = CurrentBelief.x6;
   
   
   
   MatrixXd measure(5,1);
   //IPS_Diff = Matrix<double, 6, 1>::Zero();
   MatrixXd IPS_Diff = Matrix<double, 5, 5>::Zero();
   IPS_Diff(0,0) = CurrentBelief.x1;
   IPS_Diff(1,0) = CurrentBelief.x2;
   IPS_Diff(2,0) = CurrentBelief.x3;
   IPS_Diff(4,0) = CurrentBelief.x5;
   IPS_Diff(5,0) = CurrentBelief.x6;
   
   measure = Cd*IPS_Diff;
         
    MatrixXd Ad(6,6);    
   
   Ad << 1, 0,  (-v*sin(mu3) - mu5*sin(mu3))*dt,                                                    0,       cos(mu3)*dt,    0,
         0, 1,  ( v*cos(mu3) + mu5*cos(mu3))*dt,                                                    0,       sin(mu3)*dt,    0,
         0, 0,                             1,  ((v*(tan(mu4)*tan(mu4) + 1))/L + (mu5*(tan(mu4)*tan(mu4) + 1))/L)*dt,   (tan(mu4)/L)*dt,    0,
         0, 0,                             0,                                                    1,                0,        0,
         0, 0,                             0,                                                    0,                1,        1*dt,
         0, 0,                             0,                                                    0,                0,        1;
	 
	 
	 
    double mup1 = mu1 + dt*( v*cos(mu3)+ mu5*cos(mu3) );
    double mup2 = mu2 + dt*( v*sin(mu3)+ mu5*sin(mu3) );
    double mup3 = mu3 + dt*((v/L)*tan(mu4)+(mu5/L)*tan(mu4));
    double mup4 = mu4 + dt*u1;
    double mup5 = mu5 + dt*mu6;
    double mup6 = mu6 + dt*u2;   
    
    MatrixXd Ht(5,6); 
    
    Ht << 1,   0,   0, 0, 0, 0,
          0,   1,   0, 0, 0, 0,
          0,   0,   1, 0, 0, 0,
          0,   0,   0, 0, 1, 0,
          0,   0,   0, 0, 0, 1;
	  
	//Sp(0,0) = 10;  
    //Sp = Ad*S*Ad.transpose() + R;	


    
    //Measurement update
//     K = Sp*Ht'*inv(Ht*Sp*Ht'+Q);
//     %mu = mup + K*(y(:,t)-sqrt(mup(1)^2 + mup(3)^2));
//     mu = mup + K*(measure-[mup(1);mup(2);mup(3);mup(5);mup(6);]);
  
}


int main(int argc, char **argv)
{
  
  ///////////////////// EKF Initialization ////////////////////////
  
  
        Dog woofy;
	woofy.age = 5;
	woofy.bark();
	cout << "Woofy is " << woofy.age << " years old." << endl;
	woofy.bark();
  
    //VectorXd mu;
    //VectorXd mup;
    Matrix<double, 6, 6> S; // Covariance (Sigma)
    Matrix<double, 6, 6> Sp; // Covariance (Sigma)
    MatrixXd RE;
    Matrix<double, 6, 6> R; // Prediction covariance
    Matrix<double, 5, 5> Q; // Measurement covariance
    
    Matrix<double,5,6> Cd;
    Matrix<double,6,6> Ad;

  
   RowVectorXd mu(6);
   mu << 0,0,0,0,0,0;
  
  RowVectorXd mup(6);
  mup << 0,0,0,0,0,0;
  
  S = 0.00001* Matrix<double, 6, 6>::Identity(); // Covariance (Sigma)  
  R = 0.01 * Matrix<double, 6, 6>::Identity(); // Prediction covariance
  
  EigenSolver<MatrixXd> es(R);
  //MatrixXd D = es.pseudoEigenvalueMatrix();
  RE = es.pseudoEigenvectors();
  Q = Matrix<double, 5, 5>::Zero(); // Measurement covariance
  Q(0,0) = 1e-4;
  Q(1,1) = 1e-4;
  Q(2,2) = 1e-4;
  Q(3,3) = 1e-1;
  Q(4,4) = 1e-0;
  
  Cd = Matrix<double, 5, 6>::Zero();
  Cd(0,0) = 1;
  Cd(1,1) = 1;
  Cd(2,2) = 1;
  Cd(3,4) = 1;
  Cd(4,5) = 1;
  
  ////////////////////////////////////////////////////////////////
  
  double VelocityInput = 0;
  double SteeringRateInput = 0;  
  double mytime = 0;
  
  double x1 = radius+ 0.1;
  double x2 = 0.0;
  double x3 = PI/2;
  double x4 = 0;
  double x5 = 0;
  double x6 = 0;
  double IntegVelocityInput = 0;
  double DiffVelocityInput = 0;

  
  ros::init(argc, argv, "talker");


  ros::NodeHandle n;



  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(100);

  double T = 0;
  double dt = 0.01;
  
  // The values of x1 x2 x3 comes from the IPS, we call it previous states and offset the current sates by few centimeters to avoid the singularity in the contorl law
  // For Chameleon Robot it is always fair enough to assume that the initial steeringa angle is 0.
  // We get 
  CurrentState.set_values(x1,x2,x3,x4,x5,x6);
  CurrentBelief = CurrentState;
  
  
   myfile.open ("chameleon_test.txt", ios::out | ios::app);
   myfile << "time,dt,x1,x2,x3,x4,x5,x6\n";
//   myfile.close();
    
   
   
//   MatrixXd Q = MatrixXd::Zero(5,5);
//   Q(0,0) = 1e-4;
//   Q(1,1) = 1e-4;
//   Q(2,2) = 1e-4;
//   Q(3,3) = 1e-1;
//   Q(4,4) = 1e-0;
// 
//   std::cout << "The matrix A is:\n" << A << "\n\n";
//   std::cout << "The matrix square root of A is:\n" << Q.sqrt() << "\n\n";
  
  while (ros::ok())
  {
    
    //Diff (dt, CurrentState, CurrentBelief, PreviousBelief, DiffVelocityInput);
    Control(mytime,dt,T, CurrentBelief, IntegVelocityInput, SteeringRateInput);
    //RobotModel(dt, CurrentBelief, CurrentState, DiffVelocityInput, SteeringRateInput);
    RobotModel(mytime, dt, CurrentBelief, CurrentState, IntegVelocityInput, SteeringRateInput);
    EKF(dt, CurrentBelief, Q, Cd, S, Sp, R, IntegVelocityInput, SteeringRateInput);

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

