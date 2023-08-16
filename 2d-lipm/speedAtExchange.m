function v_ex = speedAtExchange(E,x_f,zc)
    g = 9.81;    
    v_ex = sqrt(2*E + g/zc*x_f^2);
end