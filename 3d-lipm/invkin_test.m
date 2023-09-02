clear;clc;
% import urdf
robot = importrobot("model/huron.urdf","DataFormat","column");
robot.Gravity = [0,0,-9.81];
config = homeConfiguration(robot);
%%
load desJointStates.mat

config = [desJointStates.theta_left(:,350);desJointStates.theta_right(:,350)];
% centerOfMass(robot,config)
show(robot,config)