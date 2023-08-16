function [sx,sy] = step_params(step_len,step_width,ns)
    % init
    sx = zeros(1,ns);
    sy = zeros(1,ns);
    
    for i = 1:ns
        if i == 1
            sx(i) = 0;
            sy(i) = step_width/2;
        elseif i<ns
            sx(i) = step_len;
            sy(i) = step_width;
        else
            sx(i) = 0;
            sy(i) = step_width;
        end
    end

end