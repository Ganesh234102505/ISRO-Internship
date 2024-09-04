% MATLAB CODE FOR POLE PLACEMENT AND STATE FEEDBACK CONTROLLER DESIGN

clc; clear all; close all;

% DEFINE THE ORIGINAL SYSTEM MATRICES
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

% CHECK THE CONTROLLABILITY OF THE SYSTEM
controllability_matrix = ctrb(A, B);
rank_controllability = rank(controllability_matrix);

if rank_controllability == size(A, 1)
    disp('The system is controllable.');
else
    disp('The system is not controllable.');
end

% CHECK THE OBSERVABILITY OF THE SYSTEM
observability_matrix = obsv(A, C);
rank_observability = rank(observability_matrix);

if rank_observability == size(A, 1)
    disp('The system is observable.');
else
    disp('The system is not observable.');
end
disp(['Rank of the controllability matrix: ', num2str(rank_controllability)]);
disp(['Rank of the observability matrix: ', num2str(rank_observability)]);

% AUGMENT THE MATRICES
A_bar = [A zeros(6, 3); -C zeros(3, 3)];
B_bar = [B; zeros(3, 3)];

% DEFINE THE DESIRED CLOSED-LOOP POLES
desired_poles = [-1 -1.5 -2 -2.5 -3 -3.5 -4 -4.5 -5];

% COMPUTE THE STATE FEEDBACK GAIN MATRIX K USING THE PLACE FUNCTION
K = place(A_bar, B_bar, desired_poles);
disp('State feedback gain matrix K:');
disp(K);

% CHECK THE CLOSED-LOOP EIGENVALUES
eigenvalues = eig(A_bar - B_bar * K);
disp('Closed-loop eigenvalues:');
disp(eigenvalues);

% CLOSED-LOOP SYSTEM
A_cl = A_bar - B_bar * K;
B_cl = B_bar;
C_cl = eye(size(A_cl));
D_cl = zeros(size(B_cl));

% CREATE THE STATE-SPACE SYSTEM
sys_cl = ss(A_cl, B_cl, C_cl, D_cl);

