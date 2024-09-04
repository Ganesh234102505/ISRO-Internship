% Adaptive Controller Design for Satellite Attitude Control

% Define system parameters
J_original = [2.683, 0, 0; 0, 2.326, 0; 0, 0, 1.897]; % Original inertia matrix
J1 = [2.683, 0.22, 0.43; 0.18, 2.326, 0.24; 0.29, 0.14, 2.897]; % 1:1 attachment inertia matrix
%J2 = [26.83, 0.412, 0.213; 0.314, 23.26, 0.192; 0.293, 0.144, 18.97]; % Inertia matrix after 1:10 attachment from the PDF

%% Initial conditions
q_v0 = [0.9893; 0.0789; 0.0941; -0.0789]; % Initial attitude quaternion
omega0 = [3; -3; 3]; % Initial angular velocity
a = 0.2;
k_initial = 0.5; % Initial control gain
k_final = 2.0; % Final control gain after attachment
zeta_initial = 0.4; % Initial control gain
zeta_final = 0.7; % Final control gain after attachment
D = diag(0.01 * ones(1, 9)); % Control gains
L_dist = [0.1; 0.1; 0.1]; % Disturbance torque
t_span = [0, 300]; % Simulation time span
attachment_time = 100; % Time of non-cooperative attachment

% Initial state vector
x0 = [q_v0; omega0; zeros(9, 1)]; % Initial state with zero estimated inertia



%% Simulate the system
% ODE solver options
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-9);
[t, x] = ode45(@(t, x) dynamics(t, x, J_original, J1, L_dist, a, k_initial, k_final, zeta_initial, zeta_final, attachment_time, D), t_span, x0, options);

% Extract results
q_v = x(:, 1:4);
omega = x(:, 5:7);
h_hat = x(:, 8:end);

% Ensure the euler_angles array is the same length as the time vector
euler_angles = zeros(length(t), 3);
for i = 1:length(t)
    euler_angles(i, :) = quat2eul_single(q_v(i, :));
end

%% Calculate Lyapunov function
V = zeros(length(t), 1);
for i = 1:length(t)
    if t(i) < attachment_time
        V(i) = lyapunov_function(x(i, :)', J_original, D);
    else
        V(i) = lyapunov_function(x(i, :)', J1, D);
    end
end

%% Calculate control law over time
L_c_values = zeros(length(t), 3);
for i = 1:length(t)
    q_e = x(i, 1:4)'; % Error quaternion
    omega_val = x(i, 5:7)'; % Angular velocity
    h_hat_val = x(i, 8:end)'; % Estimated inertia
    if t(i) < attachment_time
        k = k_initial;
        n = zeta_initial;
    else
        k = k_final;
        n = zeta_final;
    end
    L_c_values(i, :) = control_law(q_e, omega_val, h_hat_val, k, n, a)';
end

%% Plot attitude angles (roll, pitch, yaw) in subplots
figure;
subplot(3,1,1);
plot(t, rad2deg(euler_angles(:,1)));
title('Roll Angle over Time');
xlabel('Time (s)');
ylabel('Angle (degrees)');

subplot(3,1,2);
plot(t, rad2deg(euler_angles(:,2)));
title('Pitch Angle over Time');
xlabel('Time (s)');
ylabel('Angle (degrees)');

subplot(3,1,3);
plot(t, rad2deg(euler_angles(:,3)));
title('Yaw Angle over Time');
xlabel('Time (s)');
ylabel('Angle (degrees)');

%% Plot angular velocities (omega_x, omega_y, omega_z) in subplots
figure;
subplot(3,1,1);
plot(t, rad2deg(omega(:,1)));
title('\omega_x over Time');
xlabel('Time (s)');
ylabel('Angular Velocity (deg/s)');

subplot(3,1,2);
plot(t, rad2deg(omega(:,2)));
title('\omega_y over Time');
xlabel('Time (s)');
ylabel('Angular Velocity (deg/s)');

subplot(3,1,3);
plot(t, rad2deg(omega(:,3)));
title('\omega_z over Time');
xlabel('Time (s)');
ylabel('Angular Velocity (deg/s)');

% Plot estimated inertia
figure;
plot(t, h_hat);
title('Estimated Inertia over Time');
xlabel('Time (s)');
ylabel('Inertia Estimates');
legend('h_1', 'h_2', 'h_3', 'h_4', 'h_5', 'h_6', 'h_7', 'h_8', 'h_9');

% Plot Lyapunov function
figure;
plot(t, V);
title('Lyapunov Function V over Time');
xlabel('Time (s)');
ylabel('V');

% Plot control law values over time
figure;
subplot(3,1,1);
plot(t, L_c_values(:,1));
title('Control Law L_c (Component 1) over Time');
xlabel('Time (s)');
ylabel('Control Law (Component 1)');

subplot(3,1,2);
plot(t, L_c_values(:,2));
title('Control Law L_c (Component 2) over Time');
xlabel('Time (s)');
ylabel('Control Law (Component 2)');

subplot(3,1,3);
plot(t, L_c_values(:,3));
title('Control Law L_c (Component 3) over Time');
xlabel('Time (s)');
ylabel('Control Law (Component 3)');

%% Function definitions

% Define dynamics equations
function dx = dynamics(t, x, J_original, J1, L_dist, a, k_initial, k_final, zeta_initial, zeta_final, attachment_time, D)
    q_e = x(1:4); % Error quaternion
    omega = x(5:7); % Angular velocity
    h_hat = x(8:end); % Estimated inertia
    
    % Quaternion kinematics
    q_dot = 0.5 * quatmultiply(q_e', [0; omega]');
    
    % Switch gains and inertia based on time
    if t < attachment_time
        k = k_initial;
        n = zeta_initial;
        J = J_original;
    else
        k = k_final;
        n = zeta_final;
        J = J1; % Change inertia after attachment
        if abs(t - attachment_time) < 1e-2
            omega = omega + [5; 5;5]; % Adjust this based on required disturbance
        end
    end
    
    % Angular velocity dynamics
    omega_dot = inv(J) * (-cross(omega, J * omega) + control_law(q_e, omega, h_hat, k, n, a) + L_dist);
    
    % Update inertia estimation
    Y = compute_Y(q_e, omega, a); % Define the Y matrix based on system dynamics
    h_hat_dot = D \ (Y' * (a * q_e(2:4) + omega)); % Using vector part of quaternion only
    
    dx = [q_dot'; omega_dot; h_hat_dot];
end

% Define the control law
function L_c = control_law(q_e, omega, h_hat, k, n, a)
    Y = compute_Y(q_e, omega, a); % Compute the Y matrix
    x1 = q_e(2:4); % Using vector part of quaternion only
    x2 = a * q_e(2:4) + omega; % Using vector part of quaternion only
    L_c = -0.5 * x1 - Y * h_hat - (k + n) * x2;
end

% Define the Y matrix (corrected as provided)
function Y = compute_Y(q_e, omega, a)
    q_e1 = q_e(1); q_e2 = q_e(2); q_e3 = q_e(3); q_e4 = q_e(4);
    omega1 = omega(1); omega2 = omega(2); omega3 = omega(3);
    
    Y = [ -a*q_e1, -a*q_e2,  a*q_e3, omega1*omega3, omega2*omega3, omega3^2, -omega1*omega2, -omega2^2, -omega2*omega3;
          -omega1*omega3, -omega2*omega3, -omega3*omega3, -a*q_e1, -a*q_e2, -a*q_e3, omega1^2, omega1*omega2, omega1*omega3;
           omega1*omega2,  omega2^2, omega2*omega3, -omega1^2, -omega1*omega2, -omega1*omega3, -a*q_e1, -a*q_e2, -a*q_e3 ];
end

% Define the Lyapunov function
function V = lyapunov_function(x, J, D)
    q_e = x(1:4); % Error quaternion
    omega = x(5:7); % Angular velocity
    h_hat = x(8:end); % Estimated inertia
    q_e0 = q_e(1);
    Delta_h = h_hat; 
    
    % Lyapunov function
    V = 0.5 * q_e(2:4)' * q_e(2:4) + 0.5 * (1 - q_e0)^2 + 0.5 * omega' * J * omega + 0.5 * Delta_h' * D * Delta_h;
end

%% Quaternion to Euler angles conversion function
function euler = quat2eul(q)
    % Convert quaternion to Euler angles (roll, pitch, yaw)
    euler = zeros(size(q, 1), 3);
    for i = 1:size(q, 1)
        euler(i, :) = quat2eul_single(q(i, :));
    end
end

% Convert a single quaternion to Euler angles
function euler = quat2eul_single(q)
    q0 = q(1);
    q1 = q(2);
    q2 = q(3);
    q3 = q(4);

    % Roll (x-axis rotation)
    sinr_cosp = 2 * (q0 * q1 + q2 * q3);
    cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2);
    roll = atan2(sinr_cosp, cosr_cosp);

    % Pitch (y-axis rotation)
    sinp = 2 * (q0 * q2 - q3 * q1);
    if abs(sinp) >= 1
        pitch = sign(sinp) * pi / 2; % use 90 degrees if out of range
    else
        pitch = asin(sinp);
    end

    % Yaw (z-axis rotation)
    siny_cosp = 2 * (q0 * q3 + q1 * q2);
    cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3);
    yaw = atan2(siny_cosp, cosy_cosp);

    euler = [roll, pitch, yaw];
end
