%% Walking Pattern Generation
% 3D LIPM gait generation
clear;clc;
% CoM Trajectory Generation
% walk parameters - foot positions
numSteps = 3;
n_steps = numSteps + 2; % no. of steps + 2
% lipm initial foot hold
px0 = 0; py0 = 0;
% step params
step_len = 0.4;
step_width = 0.155;
[sx,sy] = step_params(step_len,step_width,n_steps);

% sx = [0 0.4 0.4 0.4 0];
% sy = [0.155/2 0.155 0.155 0.155 0.155];
%sy = 0.155*ones(1,n_steps);

fl = 0.215; 
fw = 0.082;

zc = 1; % com height
Ts = 1; % swing phase time
g = 9.81;

time_period = n_steps*Ts;
dt = 0.01;
time = 0:dt:time_period;

% initial conditions
xi = px0; yi = py0;
vxi = 0; vyi = 0;
p_x = px0; p_y = py0;
x = 0; y = 0;
vx = 0; vy =0 ;

% eom params
Tc = sqrt(zc/g);
C = cosh(Ts/Tc);
S = sinh(Ts/Tc);

% eval func params
a = 10;
b = 1;
D = a*(C - 1)^2 + b*(S/Tc)^2;

n = 0; % curr step
T = 0; % curr time

z_com = zc*ones(size(time));
x_com = [xi];
y_com = [yi];
vx_com = [vxi];
vy_com = [vyi];
fc_x = [];
fc_y = [];
for i = 1:n_steps
    fc_x = [fc_x p_x];
    fc_y = [fc_y p_y];
    % walk primitive
    x_bar = sx(n+1)/2;
    y_bar = (-1)^n*sy(n+1)/2;
    vx_bar = x_bar*(C + 1)/(Tc*S);
    vy_bar = y_bar*(C - 1)/(Tc*S);
    % target states
    x_d = p_x + x_bar;
    y_d = p_y +  y_bar;
    vx_d = vx_bar;
    vy_d = vy_bar;
    % modified foot positions
    p_x = -(a*(C-1)/D)*(x_d - C*xi - Tc*S*vxi) -  (b*S/(Tc*D))*(vx_d - S/Tc*xi - C*vxi);
    p_y = -(a*(C-1)/D)*(y_d - C*yi - Tc*S*vyi) -  (b*S/(Tc*D))*(vy_d - S/Tc*yi - C*vyi);

    % integrate dynamics
    t = T+dt:dt:T+Ts;
    x = (xi - p_x)*cosh((t - T)/Tc) + Tc*vxi*sinh((t - T)/Tc) + p_x;
    y = (yi - p_y)*cosh((t - T)/Tc) + Tc*vyi*sinh((t - T)/Tc) + p_y;
    vx = (xi - p_x)/Tc*sinh((t - T)/Tc) + vxi*cosh((t - T)/Tc);
    vy = (yi - p_y)/Tc*sinh((t - T)/Tc) + vyi*cosh((t - T)/Tc);
   
    x_com = [x_com x];
    y_com = [y_com y];
    vx_com = [vx_com vx];
    vy_com = [vy_com vy];
    
    T = T + Ts;
    n = n + 1;
    % next foot position
    p_x = p_x + sx(n);
    p_y = p_y - (-1)^n*sy(n);
        
    % next states
    xi = x(end);
    yi = y(end);
    vxi = vx(end);
    vyi = vy(end);
end
fc_x = [fc_x p_x];
fc_y = [fc_y p_y];
% lipm footholds
fcp_x = fc_x;
fcp_y = fc_y;
fcp_y(end) = 0;
%modify initial foot position
fc_y(1) = -0.155/2;
%fc_y(end) = 0.155/2;

%% generating foot trajectories
swing_height = 0.1; %m
timesteps = 0:Ts:time_period;
% waypoints for right and left foot
foot_right_wp = [fc_x(1) fc_x(1) fc_x(3) fc_x(3) fc_x(5) fc_x(5);
               fc_y(1) fc_y(1) fc_y(3) fc_y(3) fc_y(5) fc_y(5);
               0 0 0 0 0 0];
foot_left_wp = [fc_x(2) fc_x(2) fc_x(2) fc_x(4) fc_x(4) fc_x(6);
               fc_y(2) fc_y(2) fc_y(2) fc_y(4) fc_y(4) fc_y(6);
               0 0 0 0 0 0];
% init
footPos_right = [];
footPos_left = [];
footVel_right = [];
footVel_left = [];

for i = 1:n_steps
    footpos_right_i = foot_right_wp(:,i);
    footpos_right_f = foot_right_wp(:,i+1);
    footpos_left_i = foot_left_wp(:,i);
    footpos_left_f = foot_left_wp(:,i+1);
    ti = timesteps(i);
    tf = timesteps(i+1);

    if i == 1
        swing_r = 0;
        swing_l = 0;
    elseif i < n + 1
        if mod(i,2) == 0        % right swing when i is even
            swing_r = swing_height;
            swing_l = 0;
        else                    % left swing when i is odd
            swing_r = 0;
            swing_l = swing_height;
        end
    else
        swing_r = 0;
        swing_l = 0;
    end
    
    if i < n_steps
        [q_r,qd_r,qdd_r] = getSwingFootTraj(footpos_right_i,footpos_right_f,swing_r,ti,tf-dt,dt);
        [q_l,qd_l,qdd_l] = getSwingFootTraj(footpos_left_i,footpos_left_f,swing_l,ti,tf-dt,dt);
    else
        [q_r,qd_r,qdd_r] = getSwingFootTraj(footpos_right_i,footpos_right_f,swing_r,ti,tf,dt);
        [q_l,qd_l,qdd_l] = getSwingFootTraj(footpos_left_i,footpos_left_f,swing_l,ti,tf,dt);
    end

    footPos_right = [footPos_right q_r];
    footPos_left = [footPos_left q_l];
    footVel_right = [footVel_right qd_r];
    footVel_left = [footVel_left qd_l];
end

%% Inverse Kinematics
% matlab and gazebo have different reference frames - data needs to be
% transformed to the gazebo world frame
desiredStates = convertMat2gazebo([x_com;y_com;z_com],[vx_com;vy_com;zeros(size(time))],footPos_right,footVel_right,footPos_left,footVel_left);

desJointStates = getJointStates(desiredStates);
save desJointStates.mat desJointStates -mat

%% Publish joint states to gazebo
jointStatePublisher(desJointStates)

%% animate CoM and foot motion
px = px0;
py = py0;
n = 1;

figure
for i = 1:size(time,2)
    if mod(i,100) == 0
        n = n + 1;
        px = fcp_x(n);
        py = fcp_y(n);
    end
    % LIPM
    plot3([px x_com(:,i)],[py y_com(:,i)],[0 z_com(:,i)],'-b','LineWidth',2)
    hold on
    plot3(x_com(:,i),y_com(:,i),z_com(:,i),'o','MarkerSize',15,'MarkerFaceColor','r')
    % CoM
    plot3(x_com(:,1:i),y_com(:,1:i),z_com(:,1:i));
    % right foot
    plot3(footPos_right(1,1:i),footPos_right(2,1:i),footPos_right(3,1:i));
    % left foot
    plot3(footPos_left(1,1:i),footPos_left(2,1:i),footPos_left(3,1:i));
    grid on
    axis([-0.4 1.4 -0.4 0.4 0 1.2])
    pause(dt)
    hold off
end

%% Plots
%calc foot centers and bottom right corner pts for plotting
x_off = 0.05;
y_off = fw/2;
xLeft = [fc_x(1) - x_off];
yLeft = [fc_y(1) - y_off];
for i = 1:n_steps
    xLeft(i+1) = fc_x(i+1) - x_off;
    yLeft(i+1) = fc_y(i+1) - y_off;
end

figure
hold on
grid on
axis equal
% plot footsteps
for i = 1:n_steps+1
    rectangle('Position', [xLeft(i), yLeft(i),fl,fw], 'EdgeColor', 'b','LineWidth', 2);
end
% plot CoM traj
plot(x_com,y_com,'r','LineWidth',3)
plot(fcp_x,fcp_y,'--o','MarkerSize',7.5,'MarkerFaceColor','r')
plot(fc_x,fc_y,'ob')
xlabel('X (m)')
ylabel('Y (m)')
title('3D LIPM Gait')
legend('CoM Traj','ZMP Traj','Foot Center')

% position and velocity plots
figure

subplot(2,2,1)
plot(time,x_com)
xlabel('time (sec)')
ylabel('x (m)')
title('x\_com vs t')

subplot(2,2,2)
plot(time,y_com)
xlabel('time (sec)')
ylabel('y (m)')
title('y\_com vs t')

subplot(2,2,3)
plot(time,vx_com)
xlabel('time (sec)')
ylabel('Vx (m/s)')
title('Vx\_com vs t')

subplot(2,2,4)
plot(time,vy_com)
xlabel('time (sec)')
ylabel('Vy (m/s)')
title('Vy\_com vs t')