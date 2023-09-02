function jointAngles = invKinFoot2Body(Tb,isLeft)
    % inputs: Tb - tranformation matrix of body wrt foot
    % convention:
    %  th1 - 'hip_yaw_joint'
    %  th2 - 'hip_roll_joint'
    %  th3 - 'hip_pitch_joint'
    %  th4 - 'knee_pitch_joint'
    %  th5 - 'ankle_pitch_joint'
    %  th6 - 'ankle_roll_joint'
    % params
    L1 = 0.0775;
    if isLeft
        L1 = -L1;
    end
    L2 = 0.1451; 
    L3 = 0.5128;
    L4 = 0.3700;
    L5 = 0.0948;
    % decouple
    Twrist = Tb + [0 0 0  L2;
                  0 0 0 -L1;
                  0 0 0  0 ;
                  0 0 0  1];
    % extract pos and orientation
    R = Twrist(1:3,1:3);
    p = Twrist(1:3,4);
    n = R(:,1);
    s = R(:,2);
    a = R(:,3);

    % inverse kinematics analytical sol
    cos4 = ((p(1)+L5)^2 + p(2)^2 + p(3)^2 - L3^2 - L4^2)/(2*L3*L4);
    temp = 1 - cos4^2;
    if temp < 0
        temp = 0;
        disp('Waning: Unable to reach desired end-effector position/orientation');
    end
    
    th4 = atan2(sqrt(temp),cos4);
    % NOTE: you can put -sqrt(temp) to change direction of knee bending
    temp = (p(1)+L5)^2+p(2)^2;
    if temp < 0
        temp = 0;
        disp('Warning: Unable to reach desired end-effector position/orientation');
    end
    th5 = atan2(-p(3),sqrt(temp))-atan2(sin(th4)*L3,cos(th4)*L3+L4);
    th6 = atan2(p(2),-p(1)-L5);
    temp = 1-(sin(th6)*a(1)+cos(th6)*a(2))^2;
    if temp < 0
        temp = 0;
        disp('Warning: Unable to reach desired end-effector position/orientation');
    end
    th2 = atan2(-sqrt(temp),sin(th6)*a(1)+cos(6)*a(2));
    th2 = th2 + pi/2; % pi/2 offset
    th1 = atan2(-sin(th6)*s(1)-cos(th6)*s(2),-sin(th6)*n(1)-cos(th6)*n(2));
    
    th345 = atan2(a(3),cos(th6)*a(1)-sin(th6)*a(2));
    th345 = th345 - pi;
    th3 = th345 - th4 - th5;
    
    % Pack the corresponding joint angles
    jointAngles = [th1 th2 th3 th4 th5 th6];
    
end