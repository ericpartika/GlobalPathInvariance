# GlobalPathInvariance

This repository contains the code for the work on global path invariance developed by Adeel Akhtar, Nan Wang, and Eric Partika

## HybridCarModel.m

This is the setup for the model of the vehicle. It contains the initial conditions and global variables for vehicle dimensions. This is where any calls to the functions should be tested. Two examples for the unsafe set and target set are included. 

## targetSet.m

Returns true if state is in target set within some threshold. The target set can be visualized by the function y = sin(x). X and Y threshhold is set to 0.05(5cm) and angular threshold is set to 0.2(approx. 12deg). To modify the target set these thresholds can be tuned. Currently the steering angle of the vehicle is not being considered but can be added if desired. The angular threshold is relatively high to allow for the vehicle to converge to the path in a wider range of poses. This function includes targetSet.mat, which contains the basis for the targetSet.

## unsafeSet.m

Returns true if value is in unsafe set. The unsafe set contains three squares of width 0.5(50cm) with bottom left corners located at (1,2), (3,2.5), and (4,1). 

# Flow map and set

The flow map and set are described by C.m and f.m. The flow set spans all of R^4. The flow map contains the differential equations that govern the evolution of the state.

## TrajectoryGenerationInput.m

This file is a scratch work space for visualizing everything. If you run it it will produce two figures. Figure 1 shows the vehicle position, unsafe set, and target set. Figure 2 shows the target set with vectors representing x(3) or θ, the orientation of the vehicle. 


