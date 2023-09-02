function jointStates = getJointStates(desiredStates)
    n = size(desiredStates.comPos,2);
    % rot matrices - body/foot wrt world
    Rb_w = eye(3);
    Rf_right_w = [0 -1  0;
                  0  0  1;
                 -1  0  0];
    Rf_left_w = Rf_right_w;
    % init
    jointStates.theta_right = zeros(6,n);
    jointStates.theta_left = zeros(6,n);

    for i = 1:n
        % Tranformation matrix - Base (hip) wrt world  
        Pb_w = desiredStates.comPos(:,i); 
        Tb_w = [Rb_w Pb_w; 0 0 0 1];
        % Transformation matrix - Foot wrt world
        Pf_right_w = desiredStates.rightFootPos(:,i);
        Tf_right_w = [Rf_right_w Pf_right_w; 0 0 0 1];
        Pf_left_w = desiredStates.leftFootPos(:,i);
        Tf_left_w = [Rf_left_w Pf_left_w; 0 0 0 1];
        % Transformation matrix - Base wrt Foot
        Tb_f_right = Tf_right_w\Tb_w;
        Tb_f_left = Tf_left_w\Tb_w;
        % inv kin
        theta_right = invKinFoot2Body(Tb_f_right,false);
        theta_left = invKinFoot2Body(Tb_f_left,true);
        jointStates.theta_right(:,i) = theta_right;
        jointStates.theta_left(:,i) = theta_left;
    end
end