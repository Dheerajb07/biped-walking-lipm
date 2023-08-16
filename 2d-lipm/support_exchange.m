function x_f = support_exchange(E1,E2,s,zc)
    g = 9.81;
    x_f = zc/(g*s)*(E2 - E1) + s/2; 
end