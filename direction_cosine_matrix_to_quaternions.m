% MATLAB CODE FOR CONVERTING DIRECTION COSINE MATRIX TO QUATERNIONS

% DEFINE THE ROTATION ANGLES IN RADIANS
theta_z = pi / 6; % 30 degrees (Yaw)
theta_y = pi / 4; % 45 degrees (Pitch)
theta_x = pi / 3; % 60 degrees (Roll)

% ROTATION MATRIX ABOUT THE Z-AXIS (YAW)
R_z = [cos(theta_z), sin(theta_z), 0;
       -sin(theta_z), cos(theta_z), 0;
       0, 0, 1];

% ROTATION MATRIX ABOUT THE Y-AXIS (PITCH)
R_y = [cos(theta_y), 0, -sin(theta_y);
       0, 1, 0;
       sin(theta_y), 0, cos(theta_y)];

% ROTATION MATRIX ABOUT THE X-AXIS (ROLL)
R_x = [1, 0, 0;
       0, cos(theta_x), sin(theta_x);
       0, -sin(theta_x), cos(theta_x)];

% COMBINED DCM (R_x * R_y * R_z) FOR THE 3-2-1 SEQUENCE
dcm = R_x * R_y * R_z;
disp('The Direction Cosine Matrix (DCM) is:');
disp(dcm);

% Convert DCM to quaternion
q = dcm2quat(dcm);

% Display the resulting quaternion
disp('The resulting quaternion is:');
disp(q);

% FUNCTION TO CONVERT DCM TO QUATERNION
function q = dcm2quat(dcm)
    q = [0 0 0 1]';
    tr = trace(dcm);
    num0 = 1.0 + tr;
    num1 = 1.0 + 2 * dcm(1, 1) - tr;
    num2 = 1.0 + 2 * dcm(2, 2) - tr;
    num3 = 1.0 + 2 * dcm(3, 3) - tr;
    if ((num0 >= num1) && (num0 >= num2) && (num0 >= num3))
        p0 = sqrt(num0);
        p1 = (dcm(2, 3) - dcm(3, 2)) / p0;
        p2 = (dcm(3, 1) - dcm(1, 3)) / p0;
        p3 = (dcm(1, 2) - dcm(2, 1)) / p0;
    elseif ((num1 > num0) && (num1 > num2) && (num1 > num3))
        p1 = sqrt(num1);
        p0 = (dcm(2, 3) - dcm(3, 2)) / p1;
        p2 = (dcm(2, 1) + dcm(1, 2)) / p1;
        p3 = (dcm(1, 3) + dcm(3, 1)) / p1;
    elseif ((num2 > num0) && (num2 > num1) && (num2 > num3))
        p2 = sqrt(num2);
        p0 = (dcm(3, 1) - dcm(1, 3)) / p2;
        p1 = (dcm(2, 1) + dcm(1, 2)) / p2;
        p3 = (dcm(3, 2) + dcm(2, 3)) / p2;
    elseif ((num3 > num0) && (num3 > num1) && (num3 > num2))
        p3 = sqrt(num3);
        p0 = (dcm(1, 2) - dcm(2, 1)) / p3;
        p1 = (dcm(1, 3) + dcm(3, 1)) / p3;
        p2 = (dcm(3, 2) + dcm(2, 3)) / p3;
    end
    q(1) = 0.5 * p1;
    q(2) = 0.5 * p2;
    q(3) = 0.5 * p3;
    q(4) = 0.5 * p0;
    q = qnorm(q')';
end

% FUNCTION TO NORMALIZE THE QUATERNION
function q_out = qnorm(q)
    q_out = q / norm(q);
end
