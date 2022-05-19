#include <ros/ros.h>
//#include <std_msgs/String.h>
//#include <AdeelController/GradStudent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

//#include <tf/transform_listener.h>

double x4 = 0.0;
double linVel = 0.0;
double u1, u2;
bool firstTime = true;
double currentTime, lastTime;

void callback(const nav_msgs::OdometryConstPtr& msg)
{
	if(firstTime)
	{
		currentTime = ros::Time::now().toSec();
		lastTime = currentTime;
		firstTime = false;
		return;
	}

	currentTime = ros::Time::now().toSec();
	double dt;
	dt = currentTime - lastTime;

	ROS_INFO("dt : %lf seconds", dt);

	
	//ROS_INFO("Age: %d", msg->age);
	//ROS_INFO("Gender: %d", msg->gender);
	tf::Quaternion q;
	double roll, pitch, yaw;
	tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	//ROS_INFO("RPY = (%lf, %lf, %lf)", roll, pitch, yaw);
	//ROS_INFO("Yaw = %lf)", (180/3.14)*yaw);

	double x1, x2, x3, v, L, xi1, xi2, Lf2S, Lf2P, Lg1LfS, Lg2LfS, eta1, eta2, Lg1LfP, Lg2LfP, radius, v_tran, v_tang, k1, k2 ;
	x1 = msg->pose.pose.position.x;
	x2 = msg->pose.pose.position.y;
	//x3 = (180/3.14)*yaw;
	x3 = yaw;
	ROS_INFO("x1: %lf, x2: %lf, x3: %lf, x4: %lf", x1, x2, x3, x4);	

	// Fixed vehicle parameters
	v = 0.1;

	//Linear velocity calculation

	// Controller Gains
	k1 = 5; 
	k2 = 5; 

	//radius
	radius = 2.0;
	
	
  xi1 = - radius*radius + x1*x1 + x2*x2;
  xi2 = 2*(v + x4)*(x1*cos(x3) + x2*sin(x3));
  Lg1LfS = 2*x2*(v*cos(x3) + x4*cos(x3)) - 2*x1*(v*sin(x3) + x4*sin(x3));
  Lg2LfS = 2*x1*cos(x3) + 2*x2*sin(x3);
  Lf2S = (v*sin(x3) + x4*sin(x3))*(2*v*sin(x3) + 2*x4*sin(x3)) + (v*cos(x3) + x4*cos(x3))*(2*v*cos(x3) + 2*x4*cos(x3));
  eta1 = atan(x2/x1);
  eta2 = -(x2*(v*cos(x3) + x4*cos(x3)) - x1*(v*sin(x3) + x4*sin(x3)))/(x1*x1 + x2*x2);
  Lg1LfP = (x1*(v*cos(x3) + x4*cos(x3)) + x2*(v*sin(x3) + x4*sin(x3)))/(x1*x1 + x2*x2);
  Lg2LfP = -(x2*cos(x3) - x1*sin(x3))/(x1*x1 + x2*x2);
  Lf2P = ((v*sin(x3) + x4*sin(x3))/(x1*x1 + x2*x2) + (2*x1*(x2*(v*cos(x3) + x4*cos(x3)) - x1*(v*sin(x3) + x4*sin(x3))))/( (x1*x1 + x2*x2)*(x1*x1 + x2*x2) ))*(v*cos(x3) + x4*cos(x3)) - (v*sin(x3) + x4*sin(x3))*((v*cos(x3) + x4*cos(x3))/(x1*x1 + x2*x2) - (2*x2*(x2*(v*cos(x3) + x4*cos(x3)) - x1*(v*sin(x3) + x4*sin(x3))))/( (x1*x1 + x2*x2)*(x1*x1 + x2*x2) ) );

	// Control inputs
	MatrixXd D(2,2);
	D(0,0)= Lg1LfP;
	D(0,1)= Lg2LfP;
	D(1,0)= Lg1LfS;
	D(1,1)= Lg2LfS;

	MatrixXd M(2,2);
	M=D.inverse();

	v_tran= -k1*xi1-k2*xi2;

	v_tang= -k2*(eta2 - 0.5);

	VectorXd G(2),U(2);
	G(0) = -Lf2P+v_tang;
	G(1) = -Lf2S+v_tran;

	U = M * G;
	
		  
	u1=U(0);
	u2=U(1);
	
	linVel = linVel + dt*u2;
	//linVel = linVel + (dt * u2);

	
	if(linVel>=1)  linVel = 1;
	if(linVel<=-1) linVel = -1;
	if(u1>=1) u1 = 1;
	if(u1<=-1) u1 = -1;

		
	

	ROS_INFO("Linear Velocity: %lf, Angular velocity= %lf", linVel, u2);
	

	//x4 =  x4 + dt*u2;
        x4 = x4 + u2*dt;

	lastTime = currentTime;
}



int main (int argc, char **argv)
{
	
	ros::init(argc, argv, "TestControllerSubscriber"); //Initiliazing a ros node
	ros::NodeHandle nh; //Creating ros nodehandles

	//ros::Subscriber sub = nh.subscribe("myowntopic", 1, callback);
	ros::Subscriber sub = nh.subscribe("base_pose_ground_truth", 1, callback);
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
	geometry_msgs::Twist controlSig;
	ros::Rate r(100); // 10 hz

	while(ros::ok())
	{
		
		//ROS_INFO("Hello"); // Just like a printf statement
		
		controlSig.linear.x = linVel;
		controlSig.angular.z = u1;
		pub.publish(controlSig);

		ros::spinOnce();
		r.sleep();
	}
		return 0;


}
