function dX = lipm3d_ode(t,X)
% system parameters
%m = 32; %kg
%com = 0.4687; %m
zc = 0.8; %m
g = 9.81;

% eom: xddot = g/zc*x
%      yddot = g/zc*y
% state-space 
dX = zeros(4,1);
dX(1) = X(3);
dX(2) = X(4);
dX(3) = g/zc*X(1);
dX(4) = g/zc*X(2);
end