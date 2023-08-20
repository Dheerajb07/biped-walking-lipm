function desiredStates = convertMat2gazebo(com,comdot,rFoot,rFootdot,lFoot,lFootdot)
    % init
    desiredStates.comPos = zeros(size(com));
    desiredStates.comVel = zeros(size(com));
    desiredStates.rightFootPos = zeros(size(com));
    desiredStates.leftFootPos = zeros(size(com));
    desiredStates.rightFootVel = zeros(size(com));
    desiredStates.leftFootVel = zeros(size(com));
    % rotation matrix matlab wrt gazebo
    Rmat_gaz = [0 -1 0;
                1  0 0;
                0  0 1];

    for i = 1: size(com,2)
        desiredStates.comPos(:,i) = Rmat_gaz*com(:,i);
        desiredStates.comVel(:,i) = Rmat_gaz*comdot(:,i);
        desiredStates.rightFootPos(:,i) = Rmat_gaz*rFoot(:,i);
        desiredStates.leftFootPos(:,i) = Rmat_gaz*lFoot(:,i);
        desiredStates.rightFootVel(:,i) = Rmat_gaz*rFootdot(:,i);
        desiredStates.leftFootVel(:,i) = Rmat_gaz*lFootdot(:,i);
    end

end