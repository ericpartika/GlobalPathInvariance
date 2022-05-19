
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <Eigen/Dense>
#include <fstream>
#include "Dog.h"
#include "EKFcar.h"
#include "Control.h"
#include "RobotSim.h"

#include <cstdlib>
#include <cmath>
#include <ctime>


#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

#include <geometry_msgs/Twist.h> //Velocity vector
#include <geometry_msgs/Pose2D.h>

//#include <clearpath_horizon/RawEncoders.h>
//#include <indoor_pos/ips_msg.h>



using namespace Eigen;
using namespace std;

#define PI 3.14159265

ofstream myfileEst;
ofstream myfileStates;
int mycount = 0;


void RandonMeasurementNoise(double &d1, double &d2, double &d3)
{
  Matrix<double, 3, 3> Q;
    //Measurement model defined below
    Q << 1e-3,             0,              0,
                 0,     1e-3,              0,
                 0,             0,      1e-3;
		 
  srand (time(NULL));
  boost::mt19937 gener(rand()%1000);
  boost::normal_distribution<> normal( 0.0, sqrt( Q(0,0)) );
  boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > rng(gener, normal);
  
  d1 = rng();
  //cout << d1 << endl;
  
  rng.engine().seed(); 
  rng.distribution().reset();
  
  
  boost::mt19937 gener1(rand()%1000);
  boost::normal_distribution<> normal1( 0.0, sqrt( Q(1,1)) );
  boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > rng1(gener1, normal1);
  
  d2 = rng1();
  //cout << d2 << endl;
  rng1.engine().seed(); 
  rng1.distribution().reset();
  
  
  boost::mt19937 gener2(rand()%1000);
  boost::normal_distribution<> normal2( 0.0, sqrt( Q(2,2)) );
  boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > rng2(gener2, normal2);
  
  d3 = rng2();
  //cout << d3 << endl;
  rng2.engine().seed(); 
  rng2.distribution().reset();
  
}


void ips_callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
  	double x1 = msg->x;
	double x2 = msg->y;
	double x3 = msg->theta;
	
	ROS_INFO("IPS States: %f, %f, %f, %f, %f, %f",x1, x2, x3);
	
}

 
int main(int argc, char **argv)
{
    int radius = 5;
    double x1 = radius + 0.4;
    double x2 = 0.0;
    double x3 = PI/2;
    double x4 = 0.0;
    double x5 = 0.0;
    double x6 = 0.0;
    
    double dt = 0.01;
    
  
    double v = 0.1;
    double L = 0.2;
    int count = 1;
    double Vinput = 0.0;
    double SteeringRateInput = 0.0;
    double VirtualInput = 0.0;
    double d1, d2, d3;
    

  
  EKFcar Car;
  Control CarControl;
  RobotSim Sim;
  Sim.SimulationInitialization(x1,x2,x3,x4,x5,x6,L);

  
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;
  
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  
  // Setup pubs/subs
  ros::Subscriber ips_sub = n.subscribe("/mocap_node/ground_pose", 1, ips_callback);
  //velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1); // Clearpath robot


  ros::Rate loop_rate(100);

  double T = 0;
  
  
   myfileEst.open ("Est.txt", ios::out | ios::app);
   myfileEst << "time,dt,estx1,estx2,estx3,estx4,estx5,estx6\n";
   
   myfileStates.open ("States.txt", ios::out | ios::app);
   myfileStates << "time,dt,x1,x2,x3,x4,x5,x6\n";
//   myfile.close();
  
  while (ros::ok())
  {
    
    if (count == 1)
    {
      Car.InitializeEKF(x1,x2,x3,v,L);
      Car.PredictionUpdate(Vinput, SteeringRateInput, 0, dt);//it takes two control inputs. For the first time we assume that both control inputs are zero
      Car.MesurementUpdate(x1,x2,x3);
      //CarControl.ControlLaw(x1,x2,x3,x4,x5,x6,v,L,radius);
      CarControl.ControlLaw(Car.estx1, Car.estx2, Car.estx3, Car.estx4, Car.estx5, Car.estx6, v,L,radius);
      Vinput = CarControl.Vinput;
      SteeringRateInput = CarControl.SteeringRateInput;
      VirtualInput = CarControl.VirtualInput;
      Sim.simulation(Sim.SimX1,Sim.SimX2,Sim.SimX3,Sim.SimX4,Sim.SimX5,Sim.SimX6,dt,Vinput,SteeringRateInput,VirtualInput);
      count = 2;
    }
       
    Car.PredictionUpdate(Vinput, SteeringRateInput,VirtualInput, dt);
    RandonMeasurementNoise(d1, d2, d3);
    Car.MesurementUpdate(Sim.SimX1 + d1,Sim.SimX2 + d2,Sim.SimX3 + d3);
    //Car.MesurementUpdate(Sim.SimX1,Sim.SimX2,Sim.SimX3);
    //CarControl.ControlLaw(Car.estx1, Car.estx2, Car.estx3, Car.estx4, Car.estx5, Car.estx6, v,L,radius);
    CarControl.ControlLaw(Sim.SimX1,Sim.SimX2,Sim.SimX3,Sim.SimX4,Sim.SimX5,Sim.SimX6,v,L,radius);
    //CarControl.ControlLaw(Car.estx1, Car.estx2, Car.estx3, Car.estx4, Car.estx5, Car.estx6, v,L,radius);
     Vinput = CarControl.Vinput;
     SteeringRateInput = CarControl.SteeringRateInput;
     VirtualInput = CarControl.VirtualInput;

    Sim.simulation(Sim.SimX1,Sim.SimX2,Sim.SimX3,Sim.SimX4,Sim.SimX5,Sim.SimX6,dt,Vinput,SteeringRateInput,VirtualInput);
    //Sim.simulation(Car.estx1, Car.estx2, Car.estx3, Car.estx4, Car.estx5, Car.estx6,dt,Vinput,SteeringRateInput,VirtualInput);
    T = T +dt;
    
    //ROS_INFO("Robot States: %f, %f, %f, %f, %f, %f",Sim.SimX1, Sim.SimX2, Sim.SimX3, Sim.SimX4, Sim.SimX5, Sim.SimX6);
//     ROS_INFO("Est States: %f, %f, %f, %f, %f, %f",Car.estx1, Car.estx2, Car.estx3, Car.estx4, Car.estx5, Car.estx6);
    myfileEst << T << " "<< dt << " "<< Car.estx1 << " "<< Car.estx2 << " "<< Car.estx3 << " "<< Car.estx4 << " "<< Car.estx5 << " "<< Car.estx6 << " "<< std::endl;
    myfileStates << T << " "<< dt << " "<< Sim.SimX1 << " "<< Sim.SimX2 << " "<< Sim.SimX3 << " "<< Sim.SimX4 << " "<< Sim.SimX5 << " "<< Sim.SimX6 << " "<< std::endl;

     ros::spinOnce();
     loop_rate.sleep(); 
  }
  myfileEst.close();
  myfileStates.close();
  return 0;
}
