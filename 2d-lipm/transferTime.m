function step_time = transferTime(x_i,x_f,zc)
    g = 9.81;
    t_c = sqrt(zc/g);
    step_time = t_c*log((x_f(1) + t_c*x_f(2))/(x_i(1) + t_c*x_i(2))); % sec
end