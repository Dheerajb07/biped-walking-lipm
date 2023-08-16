function lipm1d_sim(tspan,x_i,zc)
% lipm sim
%zc = 0.4; %m
g = 9.81;
% sim time
%tspan = [0,0.8];
% intital cond.
%x_init = [-0.151,0.467];
%x_i = [-0.2 1.05]; 
%x_init = [0.01 0];
[t,sys_out] = ode45(@lipm1d_ode,tspan,x_i);
x = sys_out(:,1);
xdot = sys_out(:,2);
z = zc*ones(size(t,1),1);

%% PLOTS
% % x vs z plot
% figure
% plot(x,z,'-o')
% grid on
% xlabel("x (m)")
% ylabel("z (m)")
% title("LIPM response")

figure
%hold on
dt = t(2)-t(1);
for i = 1:size(t,1)
    plot([0 x(i)],[0 z(i)],'-b','LineWidth',4)
    hold on
    plot(x(i),z(i),'o','MarkerSize',15,'MarkerFaceColor','r')
    plot(x(i),z(i),'o',[0 x(i)],[0 z(i)],'-')
    grid on
    axis([-0.2 0.2 0 1.25*zc])
    axis equal
    xlabel("x (m)")
    ylabel("z (m)")
    title("LIPM Response")
    pause(dt*7.5)
    hold off
end


% x vs t plot
figure
plot(t,x)
grid on
xlabel("t (s)")
ylabel("x (m)")
title("X vs Time")

% vel vs plot
figure
plot(t,xdot)
grid on
xlabel("t (s)")
ylabel("V (m/s)")
title("Xdot vs Time")

% orbital energy plot
E = round(orbital_energy(sys_out,zc),3);
figure
plot(t,E);
grid on
xlabel("Time (s)")
ylabel("Orbital Energy")

end
