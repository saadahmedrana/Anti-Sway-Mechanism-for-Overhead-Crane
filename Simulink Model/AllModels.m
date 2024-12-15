% Define constants
m1 = 250;  
m2 = 55; 
combinedmass = m1+m2;

g = 9.81; % Acceleration due to gravity (m/s^2)
l = 2.5;
kf = 0.4;
% Length of the pendulum (adjust as needed)

% State-space matrices
A = [0, 1, 0, 0; 
     0, 0, (m1*g)/m2, 0; 
     0, 0, 0, 1; 
     0, 0, (-g*(m1+m2))/(l*m2), 0];

B = [0; 
     (1/m2); 
     0; 
     (-1/(l*m2))];

C = [1, 0, 1, 0];  % Output matrix
D = 0;             % No direct feedthrough

% Create state-space object
sys = ss(A, B, C, D);

% Calculate controllability and observability matrices
sc = ctrb(sys);  % Controllability matrix
so = obsv(sys);  % Observability matrix

% Check rank for controllability and observability
controllability_rank = rank(sc);
observability_rank = rank(so);

% Initial state of the system
x0 = [0; 0; 0; 0];

% Define LQR weights
Q = eye(4);      % Initialize Q as a 4x4 identity matrix
Q(1,1) = 0.75;   % Modify the (1,1) element of Q
R = 1/40;  
ref = 2;
% Define R

% Compute LQR gain
K_lqr = lqr(A, B, Q, R);

% Display results
fprintf('Controllability rank: %d\n', controllability_rank);
fprintf('Observability rank: %d\n', observability_rank);
disp('LQR gain matrix K_lqr:');
disp(K_lqr);

% Save variables to the workspace for Simulink
save('workspace_variables.mat', 'A', 'B', 'C', 'D', 'K_lqr', 'x0', 'm1', 'm2', 'g', 'l');

% Load variables into MATLAB workspace
load('workspace_variables.mat');
