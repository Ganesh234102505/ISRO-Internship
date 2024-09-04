clc; clear all; close all;

% Define the original system matrices
A = [0 0 0 1 0 0; 
     0 0 0 0 1 0; 
     0 0 0 0 0 1; 
     0 0 0 0 0 0; 
     0 0 0 0 0 0; 
     0 0 0 0 0 0];
B = [0 0 0; 
     0 0 0; 
     0 0 0; 
     0.3727 0 0; 
     0 0.4299 0; 
     0 0 0.5271];
C = [1 0 0 0 0 0; 
     0 1 0 0 0 0; 
     0 0 1 0 0 0];
D = zeros(3, 3);

% Define the desired closed-loop poles
desired_poles = [-1 -1.5+j*5 -1.5-j*5 -2.5 -3 -3.5];

% Compute the state feedback gain matrix K using the place function
K = place(A, B, desired_poles);

% Display the gain matrix K
disp('State feedback gain matrix K:');
disp(K);

% Define the closed-loop system
A_cl = A - B * K;
B_cl = B;
C_cl = eye(size(A_cl));
D_cl = zeros(size(B_cl));

% Create the state-space system
sys_cl = ss(A_cl, B_cl, C_cl, D_cl);

% Compute the step response
[y, t] = step(sys_cl);

% Convert quaternions to Euler angles
quaternion1 = y(:, 1);
quaternion2 = y(:, 2);
quaternion3 = y(:, 3);
quaternion0 = sqrt(1 - quaternion1.^2 - quaternion2.^2 - quaternion3.^2); % Assume unit quaternion

% Calculate Euler angles
roll = atan2(2.*(quaternion0.*quaternion1 + quaternion2.*quaternion3), 1 - 2.*(quaternion1.^2 + quaternion2.^2));
pitch = asin(2.*(quaternion0.*quaternion2 - quaternion3.*quaternion1));
yaw = atan2(2.*(quaternion0.*quaternion3 + quaternion1.*quaternion2), 1 - 2.*(quaternion2.^2 + quaternion3.^2));

% Convert radians to degrees
roll = rad2deg(roll);
pitch = rad2deg(pitch);
yaw = rad2deg(yaw);

% Plot the Euler angles
figure;
subplot(3, 1, 1);
plot(t, roll);
title('Step Response of Roll (\phi)');
xlabel('Time (s)');
ylabel('Roll (degrees)');
grid on;

subplot(3, 1, 2);
plot(t, pitch);
title('Step Response of Pitch (\theta)');
xlabel('Time (s)');
ylabel('Pitch (degrees)');
grid on;

subplot(3, 1, 3);
plot(t, yaw);
title('Step Response of Yaw (\psi)');
xlabel('Time (s)');
ylabel('Yaw (degrees)');
grid on;

eigenvalues = eig(A_cl);
disp('Eigenvalues of the closed-loop A matrix:');
disp(eigenvalues);
