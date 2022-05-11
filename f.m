function xdot = f(x,u)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab M-file                Author: Eric Partika
%
% Description: Flow map
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% differential equations
x1dot = u(1) * cos(x(3)); % v*cos(theta)
x2dot = u(1) * sin(x(3)); % v*sin(theta)
x3dot = u(1) * (tan(x(4)))/(L); % v*(tan(delta))/L; L is length of vehicle
x4dot = u(2); % delta range is +- 20deg

xdot = [x1dot; x2dot; x3dot; x4dot];

end