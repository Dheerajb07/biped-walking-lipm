clear;clc;
g = 9.81;

% Gait parameters
zc = 0.4; %m
% initial cond
x_i = [0.01 0];
% final cond
x_f = [-0.01 0];
% number of steps
n_steps = 5;
%step length
s = 0.2; %m
% walking speed
v_walk = 0.25; %m/s

% orbital energy: E = 1/2*xdot^2 - g/(2*zc)*x^2;
E = zeros(n_steps+1,1);
E(1) = orbital_energy(x_i,zc);
E(end) = orbital_energy(x_f,zc);
for i = 2:n_steps
    E(i) = orbital_energy([0 v_walk],zc);
end

% E0 = orbital_energy([x_i(1),0],zc);                    % intial energy
% E1 = orbital_energy([0 v_walk],zc) ;            % intermediate energy
% E2 = orbital_energy([x_f(1),0],zc);                    % final energy

% exchange positions
x_exch = zeros(n_steps,1);
for i = 1:n_steps
    x_exch(i) = support_exchange(E(i),E(i+1),s,zc);
end
  
% x_f1 = support_exchange(E0,E1,s,zc);
% x_f2 = support_exchange(E1,E2,s,zc);

% speed at exchange
v_exch = zeros(n_steps,1);
for i = 1:n_steps
    v_exch(i) = speedAtExchange(E(i),x_exch(i),zc);
end
% v_ex1 = speedAtExchange(E0,x_f1,zc);
% v_ex2 = speedAtExchange(E1,x_f2,zc);

%%
% timesteps of phase change
t_phase = zeros(n_steps+2,1);
x_start = x_i;
x_end = [x_exch(1) v_exch(1)];
t_phase(2) = transferTime(x_start,x_end,zc);
for i = 2:n_steps
    x_start = [x_exch(i-1)-s v_exch(i-1)];
    x_end = [x_exch(i) v_exch(i)];
    t_phase(i+1) = t_phase(i) + transferTime(x_start,x_end,zc);
end
x_start = [x_exch(end)-s v_exch(end)];
x_end = x_f;
t_phase(end) = t_phase(end-1) + transferTime(x_start,x_end,zc);

% t_f1 = transferTime(x_i,[x_f1 v_ex1],zc);
% t_f2 = t_f1 + transferTime([x_f1-s,v_ex1],[x_f2,v_ex2],zc);
% tf = t_f2 + transferTime([x_f2-s,v_ex2],x_f,zc);

%% Sim
% phase 1: start until 1st exchange
% lipm_sim()[0,0.8],[x_f1,v_ex1]
% [time1,out1] = ode45(@lipm1d_ode,[t0,t_f1],x_i);
% [time2,out2] = ode45(@lipm1d_ode,[t_f1,t_f2],[x_f1-s,v_ex1]);
% [time3,out3] = ode45(@lipm1d_ode,[t_f2,tf],[x_f2-s,v_ex2]);

[t,y] = ode45(@lipm1d_ode,[t_phase(1),t_phase(2)],x_i);
time = t;
out = y;
for i = 1:n_steps
    x_t = [x_exch(i)-s v_exch(i)];
    [t,y] = ode45(@lipm1d_ode,[t_phase(i+1),t_phase(i+2)],x_t);
    time = [time;t];
    out = [out;y];
end
[t,y] = ode45(@lipm1d_ode,[t_phase(end-1),t_phase(end)],x_f);

%lipm_sim([t0,t_f1],x_i)
%lipm_sim([0,t_f2-t_f1],[x_f1-s,v_ex1])

% time = [time1;time2;time3];
% out = [out1;out2;out3];

t_exch = t_phase(2:end-1);

lipm_gait_sim(out(:,1),time,t_exch,zc,s);

figure
plot(time,out(:,1))
axis([0 t_phase(end) -s s])
grid on
xlabel("t (s)")
ylabel("x (m)")
title("X vs Time")
hold on
plot(time,zeros(size(time)))

figure
plot(time,out(:,2))
grid on
xlabel("t (s)")
ylabel("v (m/s)")
title("velocity vs time")