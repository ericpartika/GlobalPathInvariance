#ifndef EKFcar_H
#define EKFcar_H

#include "ros/ros.h"
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/Core>

using namespace Eigen;
using Eigen::MatrixXd;
using namespace std;


class EKFcar{
  
private:
	double mu1 ;
        double mu2 ;
        double mu3 ;
        double mu4 ;
        double mu5 ;
        double mu6 ;  
	
	
	double mup1 ;
        double mup2 ;
        double mup3 ;
        double mup4 ;
        double mup5 ;
        double mup6 ;
	
	double measX1;
	double measX2;
	double measX3;
	
	double v ;
        double L ;
	double u1;
	double u2;
	
	Matrix<double, 6, 1> mu; // Belief
	Matrix<double, 6, 1> mup; // PredictedBelief
	
	Matrix<double, 6, 6> S; // Covariance (Sigma)
	Matrix<double, 6, 6> Sp; // Predicted Covariance (Sigma_p)
	Matrix<double, 6, 6> R; // Prediction covariance
	Matrix<double, 3, 3> Q; // Measurement covariance
	
	Matrix<double, 3, 6> Ht;
	
public:
	int age;
	double estx1, estx2, estx3, estx4, estx5, estx6;
	double EstimatedStates[6];
	void bark();
	void PredictionUpdate(double Vinput, double SteeringRate, double VirtualInput, double dt);
	void MesurementUpdate(double measX1, double measX2, double measX3);
	EKFcar();//constructor
	void InitializeEKF(double mX1, double mX2, double mX3,double Fixedpush, double Lenght);//overloaded constructor
	~EKFcar() {};
};

#endif
