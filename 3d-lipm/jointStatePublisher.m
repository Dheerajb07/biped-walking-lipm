function jointStatePublisher(desJointStates)
% Initialize ROS node
rosinit;

% Create a publisher for joint state messages
jointStatePub = rospublisher('/huron/joint_group_position_controller/command', 'std_msgs/Float64MultiArray');

% Define joint names and initial joint values
jointNames = {'l_ankle_pitch_joint', 'l_ankle_roll_joint', 'l_hip_pitch_joint','l_hip_roll_joint',...
               'l_hip_yaw_joint','l_knee_pitch_joint','r_ankle_pitch_joint','r_ankle_roll_joint',...
               'r_hip_pitch_joint','r_hip_roll_joint','r_hip_yaw_joint','r_knee_pitch_joint'};
numJoints = numel(jointNames);
jointValues = zeros(1, numJoints);

% Create a JointState message object
jointStateMsg = rosmessage('std_msgs/Float64MultiArray');
jointStateMsg.Data = jointValues;
% Set the publishing rate
publishingRate = 100; % 0.01s interval
rate = rosrate(publishingRate);

n_ts = size(desJointStates.theta_left,2);
% Main loop
for i = 1:n_ts
    % Update joint values (you can replace this with your logic)
%     % left leg jont vals
%     jointValues(:,1) = desJointStates.theta_left(5,i); %l_ankle_pitch_joint
%     jointValues(:,2) = desJointStates.theta_left(6,i); %l_ankle_roll_joint
%     jointValues(:,3) = desJointStates.theta_left(3,i); %l_hip_pitch_joint
%     jointValues(:,4) = desJointStates.theta_left(2,i); %l_hip_roll_joint
%     jointValues(:,5) = desJointStates.theta_left(1,i); %l_hip_yaw_joint
%     jointValues(:,6) = desJointStates.theta_left(4,i); %l_knee_pitch_joint
%     % right leg joint vals
%     jointValues(:,7) = desJointStates.theta_right(5,i); %r_ankle_pitch_joint
%     jointValues(:,8) = desJointStates.theta_right(6,i); %r_ankle_roll_joint
%     jointValues(:,9) = desJointStates.theta_right(3,i); %r_hip_pitch_joint
%     jointValues(:,10) = desJointStates.theta_right(2,i); %r_hip_roll_joint
%     jointValues(:,11) = desJointStates.theta_right(1,i); %r_hip_yaw_joint
%     jointValues(:,12) = desJointStates.theta_right(4,i); %r_knee_pitch_joint
    
    jointValues = [desJointStates.theta_left(:,1)' desJointStates.theta_right(:,1)'];
    
    % Update joint state message
    jointStateMsg.Data = jointValues;
%     jointStateMsg.Header.Stamp = rostime('now');

    % Publish the joint state message
    send(jointStatePub, jointStateMsg);

    waitfor(rate); % Wait for the specified interval
end

% Clean up when the script is stopped
rosshutdown;
end
