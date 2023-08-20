clear;clc;
% import urdf
robot = importrobot("model/huron.urdf","DataFormat","column");
robot.Gravity = [0,0,-9.81];
config = homeConfiguration(robot);
%%
load desJointStates.mat

config = [desjointStates.theta_left(:,100);desjointStates.theta_right(:,100)];
% centerOfMass(robot,config)
show(robot,config);