% MATLAB CODE FOR CONVERTING EULER ANGLES TO QUATERNIONS

% DEFINE THE EULER ANGLES (IN DEGRESS) FOR THE 3-2-1 SEQUENCE
ang = [30; 45; 60]; % [Yaw, Pitch, Roll] in degrees
rotation1 = 3; % First rotation axis (Yaw)
rotation2 = 2; % Second rotation axis (Pitch)
rotation3 = 1; % Third rotation axis (Roll)


% CONVERT EULER ANGLES TO QUATERNIONS
q = eul2quat(ang, rotation1, rotation2, rotation3);
disp('The resulting quaternion is:');
disp(q);

% FUNCTION TO CONVERT EULER ANGLES TO QUATERNIONS
function [q] = eul2quat(ang, rotation1, rotation2, rotation3)
    q1 = [0; 0; 0; 1];
    q2 = [0; 0; 0; 1];
    q3 = [0; 0; 0; 1];
    
    % CONVERT ANGLES FROM DEGREES TO RADIANS
    q1(1) = sin(ang(1) * pi / 360);
    q1(4) = cos(ang(1) * pi / 360);

    q2(2) = sin(ang(2) * pi / 360);
    q2(4) = cos(ang(2) * pi / 360);

    q3(3) = sin(ang(3) * pi / 360);
    q3(4) = cos(ang(3) * pi / 360);

    % QUATERNION MULTIPLICATION IN EACH CASE
    if ((rotation1 == 1) & (rotation2 == 2) & (rotation3 == 3))
        qtemp = qmult(q1, q2);
        q = qmult(qtemp, q3);
    elseif ((rotation1 == 1) & (rotation2 == 3) & (rotation3 == 2))
        qtemp = qmult(q1, q3);
        q = qmult(qtemp, q2);
    elseif ((rotation1 == 2) & (rotation2 == 3) & (rotation3 == 1))
        qtemp = qmult(q2, q3);
        q = qmult(qtemp, q1);
    elseif ((rotation1 == 2) & (rotation2 == 1) & (rotation3 == 3))
        qtemp = qmult(q2, q1);
        q = qmult(qtemp, q3);
    elseif ((rotation1 == 3) & (rotation2 == 2) & (rotation3 == 1))
        qtemp = qmult(q3, q2);
        q = qmult(qtemp, q1);
    elseif ((rotation1 == 3) & (rotation2 == 1) & (rotation3 == 2))
        qtemp = qmult(q3, q1);
        q = qmult(qtemp, q2);
    end
    
    % QUATERNION NORMALIZATION
    q = qnorm(q');
end

% QUATERNION MULTIPLICATION
function q = qmult(q1, q2)
    q = zeros(4,1);
    q(1) = q1(4) * q2(1) + q1(1) * q2(4) + q1(2) * q2(3) - q1(3) * q2(2);
    q(2) = q1(4) * q2(2) - q1(1) * q2(3) + q1(2) * q2(4) + q1(3) * q2(1);
    q(3) = q1(4) * q2(3) + q1(1) * q2(2) - q1(2) * q2(1) + q1(3) * q2(4);
    q(4) = q1(4) * q2(4) - q1(1) * q2(1) - q1(2) * q2(2) - q1(3) * q2(3);
end

% FUNCTION TO NORMALIZE QUATERNION
function q_out = qnorm(q)
    q_out = q / norm(q);
end
