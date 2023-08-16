function lipm_gait_sim(x,t,t_exch,zc,s)
    % x_exch: array of x values where leg exchange occurs
    n_steps = size(t_exch,1);
    t_idx = zeros(n_steps,1);
    for i = 1:n_steps
        idx = find(t==t_exch(i));
        t_idx(i) = idx(1);
    end
    z = zc*ones(size(t,1),1);
    figure
    %hold on
    dt = t(2) - t(1);
    j = 1;
    for i = 1:size(x,1)
        if j<n_steps+1
            if i > t_idx(j)
                j = j + 1;
            end
        end
        plot([(j-1)*s (j-1)*s+x(i)],[0 z(i)],'-b','LineWidth',4)
        hold on
        plot((j-1)*s+x(i),z(i),'o','MarkerSize',15,'MarkerFaceColor','r')
        grid on
        axis equal
        axis([-s (n_steps+1)*s 0 1.25*zc])
        xlabel("x (m)")
        ylabel("z (m)")
        title("LIPM Response")
        pause(dt*20)
        hold off
    end
    
end