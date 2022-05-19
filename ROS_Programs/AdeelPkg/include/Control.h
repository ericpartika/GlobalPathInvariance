#ifndef CONTROL_H
#define CONTROL_H

class Control {
  
private:
  double x1;
  double x2;
  double x3;
  double x4;
  double x5;
  double x6;
  
  double xi1;
  double xi2;
  double xi3;
  double  eta1;
  double  eta2;
  double  eta3;
  
  
  double k1, k2, k3, k4, k5;
  double vfixed, L, radius;

public:
	double SteeringRateInput, Vinput, VirtualInput;
	Control();//constructor
	void ControlLaw(double x1, double x2, double x3, double x4, double x5, double x6, double v, double L, double radius);
};

#endif
