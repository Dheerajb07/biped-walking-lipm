function dX = lipm1d_ode(t,X)
% system parameters
%m = 32; %kg
%com = 0.4687; %m
zc = 0.4; %m
g = 9.81;

% %step length
% s = 0.2; %m
% %support exchange
% xf = 0.1; %m

% eom: xddot = g/zc*x
% state-space 
dX = zeros(2,1);
dX(1) = X(2);
dX(2) = g/zc*X(1);

end