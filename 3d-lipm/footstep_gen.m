%function [left_foot_wp,righ_foot_wp] = footstep_gen(n_steps,sx,sy,Ts,p_left_i,p_right_i)
%params   
n_steps = 5;
sx = 0.4;

Ts = 1;
p_r_i = [0; -0.0775; 0];
p_l_i = [0;  0.0775; 0];

% calculate waypoints for footsteps to generate trajectory
time_period= Ts*n_steps;
t_wp = 0:Ts/2:time_period;

left_foot_wp = zeros(1,size(t_wp,2));
righ_foot_wp = zeros(1,size(t_wp,2));
p_r = p_r_i;
p_l = p_l_i;

phase = 1;
% right leg swings first
i = 1;
for t = t_wp     
    if t < 1
        
       
    
    righ_foot_wp(i) = p_r;
    left_foot_wp(i) = p_l;
end

