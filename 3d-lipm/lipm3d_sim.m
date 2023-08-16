clear;clc;
%function lipm3d_sim(tspan,x_i)
tspan = [0 1];
%x_i = [-1 0.1 5 0];
%x_i = [-0.1500    0.1000    0.98    -0.375];
x_i= [-0.2    0.0775    0.6836   -0.2224];
% lipm 3d sim
zc = 1; %m
g = 9.81;

[t,sys_out] = ode45(@lipm3d_ode,tspan,x_i);
x = sys_out(:,1);
y = sys_out(:,2);
xdot = sys_out(:,3);
ydot = sys_out(:,4);
z = zc*ones(size(t,1),1);

%% PLOTS
% x y z plot sim wrt time
figure
dt = t(2)-t(1);
for i = 1:size(t,1)
    plot3([0 x(i)],[0 y(i)],[0 z(i)],'-b','LineWidth',2)
    hold on
    plot3(x(i),y(i),z(i),'o','MarkerSize',5,'MarkerFaceColor','r')
    plot3(x(1:i),y(1:i),z(1:i),'g')
    grid on
    axis([-0.5 0.5 -0.5 0.5 0 1.25])
    xlabel("x (m)")
    ylabel("y (m)")
    zlabel("z (m)")
    title("3D LIPM Response")
    pause(dt*5)
    hold off
end


% x vs y plot
figure
plot(x,y)
grid on
xlabel("x (m)")
ylabel("y (m)")
title("X vs Y Plot")

% vel vs t plot
figure
subplot(2,1,1)
plot(t,xdot)
grid on
xlabel("t (s)")
ylabel("Vx (m/s)")
title("Vx vs Time")
subplot(2,1,2)
plot(t,ydot)
grid on
xlabel("t (s)")
ylabel("Vy (m/s)")
title("Vy vs Time")
% 
% % orbital energy plot
% E = round(orbital_energy(sys_out,zc),3);
% figure
% plot(t,E);
% grid on
% xlabel("Time (s)")
% ylabel("Orbital Energy")

%end
