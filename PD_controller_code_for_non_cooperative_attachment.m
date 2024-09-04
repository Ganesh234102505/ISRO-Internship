% MATLAB Code to Simulate 1:1 Attachment under Original PD Control


J_original = diag([2.683, 2.326, 1.897]); % Original inertia matrix
J1 = [2.683, 0.22, 0.43; 0.18, 2.326, 0.24; 0.29, 0.14, 2.897]; % Inertia matrix after 1:1 attachment
% J2 = [26.83, 0.412, 0.213; 0.314, 23.26, 0.192; 0.293, 0.144, 18.97]; % Inertia matrix after 1:10 attachment

Kp = 1.97;
Kd = 3.219; 



%% Initial conditions
q = [0.9893; 0.0789; 0.0941; -0.0789]; % Initial quaternion
omega = [3; -3; 3];              % Initial angular rate
L_dist = [1; 1; 1];              % Initial disturbance torque

t_total = 300;      % Total simulation time
dt = 0.1;           % Time step
time = 0:dt:t_total;
n = length(time);


attitude_angles = zeros(3, n);     % Preallocate arrays for results
%% loop
for i = 1:n
    t = time(i);
    Lc = -Kp * q(2:4) - Kd * omega;    % control torque using PD controller
    if t >= 100 && t < 115 
        J = J1;
        L_dist = [10; 10; 10]; 
    else
        J = J_original;
        L_dist = [0; 0; 0];
    end
    q_dot = 0.5 * quatmultiply(q', [0; omega]')';    % Quaternion kinematics
    omega_dot = J \ (-cross(omega, J*omega) + Lc + L_dist);    % dynamics
    
    % Update states
    q = q + q_dot * dt;
    omega = omega + omega_dot * dt;
   
    q = q / norm(q);         % Normalize quaternion
    attitude_angles(:, i) = quat2eul(q', 'ZYX')' * (180/pi); % Convert to degrees
end

%% Plot attitude angles
figure;

subplot(3, 1, 1);
plot(time, attitude_angles(1, :));
title('Roll Angle Over Time');
xlabel('Time (s)');
ylabel('Roll Angle (degrees)');
ylim([-200 200]);

subplot(3, 1, 2);
plot(time, attitude_angles(2, :));
title('Pitch Angle Over Time');
xlabel('Time (s)');
ylabel('Pitch Angle (degrees)');
ylim([-200 200]);

subplot(3, 1, 3);
plot(time, attitude_angles(3, :));
title('Yaw Angle Over Time');
xlabel('Time (s)');
ylabel('Yaw Angle (degrees)');
ylim([-200 200]);
