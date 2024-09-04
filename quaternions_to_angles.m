% MATLAB CODE FOR CONVERTING QUATERNIONS TO EULER ANGLES

% DEFINE A SAMPLE QUATERNION
q = [0.3604;0.4397;0.0223;0.8224];

% CONVERT QUATERNION TO EULER ANGLES USING 3-2-1 SEQUENCE (YAW, PITCH, ROLL)
rot_seq = 5; % 3-2-1 sequence
ang = q2ang(q, rot_seq);
disp('The resulting Euler angles (Pitch, Roll, Yaw) are:');
disp(ang);

% FUNCTION TO CONVERT QUATERNION TO EULER ANGLES
function ang = q2ang(q, rot_seq)
    % CALCULATE THE DIRECTION COSINE MATRIX (DCM) FROM THE QUATERNION
    dcm(1,1) = q(1)*q(1) - q(2)*q(2) - q(3)*q(3) + q(4)*q(4);
    dcm(1,2) = 2.0 * (q(1)*q(2) + q(3)*q(4));
    dcm(1,3) = 2.0 * (q(1)*q(3) - q(2)*q(4));
    dcm(2,1) = 2.0 * (q(1)*q(2) - q(3)*q(4));
    dcm(2,2) = q(2)*q(2) - q(1)*q(1) - q(3)*q(3) + q(4)*q(4);
    dcm(2,3) = 2.0 * (q(2)*q(3) + q(1)*q(4));
    dcm(3,1) = 2.0 * (q(1)*q(3) + q(2)*q(4));
    dcm(3,2) = 2.0 * (q(2)*q(3) - q(1)*q(4));
    dcm(3,3) = q(3)*q(3) - q(1)*q(1) - q(2)*q(2) + q(4)*q(4);

    % CONVERT DCM TO EULER ANGLES BASED ON THE SPECIFIED ROTATION SEQUENCE
    if (rot_seq == 1)
        temp = sqrt(dcm(3,2)*dcm(3,2) + dcm(3,3)*dcm(3,3));
        ang(1) = -atan2(dcm(3,2), dcm(3,3));       
        ang(2) = atan2(dcm(3,1), temp);
        ang(3) = -atan2(dcm(2,1), dcm(1,1));
    elseif (rot_seq == 2)
        temp = sqrt(dcm(1,1)*dcm(1,1) + dcm(3,1)*dcm(3,1));
        ang(1) = atan2(dcm(2,3), dcm(2,2));       
        ang(2) = atan2(dcm(3,1), dcm(1,1));
        ang(3) = -atan2(dcm(2,1), temp);
    elseif (rot_seq == 3)
        temp = sqrt(dcm(1,1)*dcm(1,1) + dcm(1,3)*dcm(1,3));
        ang(1) = -atan2(dcm(3,2), dcm(2,2));        
        ang(2) = -atan2(dcm(1,3), dcm(1,1));
        ang(3) = atan2(dcm(1,2), temp);
    elseif (rot_seq == 4)
        temp = sqrt(dcm(3,1)*dcm(3,1) + dcm(3,3)*dcm(3,3));
        ang(1) = -atan2(dcm(3,2), temp);        
        ang(2) = atan2(dcm(3,1), dcm(3,3));
        ang(3) = atan2(dcm(1,2), dcm(2,2));
    elseif (rot_seq == 5)
        temp = sqrt(dcm(2,3)*dcm(2,3) + dcm(3,3)*dcm(3,3));
        ang(1) = atan2(dcm(2,3), dcm(3,3));       
        ang(2) = -atan2(dcm(1,3), temp);
        ang(3) = atan2(dcm(1,2), dcm(1,1));
    elseif (rot_seq == 6)
        temp = sqrt(dcm(1,3)*dcm(1,3) + dcm(3,3)*dcm(3,3));
        ang(1) = atan2(dcm(2,3), temp);        
        ang(2) = -atan2(dcm(1,3), dcm(3,3));
        ang(3) = -atan2(dcm(2,1), dcm(2,2));
    end 
    ang = ang * 180 / pi; % Convert angles from radians to degrees
end
