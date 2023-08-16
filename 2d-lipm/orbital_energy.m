function E = orbital_energy(x,zc)
    g = 9.81;
    E = -g/(2*zc)*x(:,1).^2 + 1/2*x(:,2).^2;
end