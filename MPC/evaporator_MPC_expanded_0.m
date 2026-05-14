%% 1. INITIALIZATION & DISTURBANCES
clear variables; clc;

t0 = 0;
tfin = 10000;
assignin('base', 't0', t0);
assignin('base', 'tfin', tfin);
disp('Workspace variables t0 and tfin are ready.');

% Evaporator Parameters
rhoA     = 20; M = 20; Cpar = 4; Cp = 0.07; 
lambda   = 38.5; UA2 = 6.84; lambda_s = 36.6; 
par = [rhoA, M, Cpar, Cp, lambda, UA2, lambda_s];

% Measurement Noise variance
var_L = 0.01; var_X = 0.05; var_P = 1;  

% MPC regulator constraints
L2_nom = 1.0; % Assuming nominal is 1m
% Lower limit (0.3m absolute) -> deviation of -0.7m
L2_dev_min = 0.3 - L2_nom; 
% Upper limit (2.0m absolute) -> deviation of +1.0m
L2_dev_max = 2.0 - L2_nom;

% Disturbance variables
dist.F1 = 10.0; dist.X1 = 5.0; dist.T1 = 40.0; dist.T200 = 25.0; 

%% 2. STEADY-STATE OPTIMIZATION (fmincon)
% Decision variables: [P2, L2, X2, F2, F3, F200, P100]
xMin = [40, 0.3, 20, 0, 0, 1, 100]; 
xMax = [80, 2.0, 100, 20, 80, 380, 380]; 
xInitial = [50.5, 1.0, 35.0, 8.0, 50.0, 208.0, 194.7];

options = optimoptions('fmincon', 'Display', 'none', 'Algorithm', 'sqp');
[xOpt, fval] = fmincon(@(x) MaxProfitFunction(x, dist), xInitial, [], [], [], [], xMin, xMax, @(x) nonlinearconE(x, dist), options);

% Extract Optimized Setpoints (Nominal Operating Point)
P2_opt = xOpt(1); L2_opt = xOpt(2); X2_opt = xOpt(3);
F2_opt = xOpt(4); F3_opt = xOpt(5); F200_opt = xOpt(6); P100_opt = xOpt(7);

x_nom = [L2_opt; X2_opt; P2_opt]; 
u_full_nom = [dist.F1; F2_opt; F3_opt; dist.X1; F200_opt; dist.T1; dist.T200; P100_opt];
u_mv_nom = [F2_opt; F3_opt; F200_opt; P100_opt];

%% 3. OPEN-LOOP LINEARIZATION
model_name = 'Evaporator_dummy_linearized_0';
if ~bdIsLoaded(model_name), load_system(model_name); end

% Define xinit explicitly for the S-Function block to find it
xinit = x_nom; 
assignin('base', 'xinit', xinit); % Force it into the base workspace

[A_full, B_full, C_full, D_full] = linmod(model_name, x_nom, u_full_nom);

%% 4. MANUAL MPC DESIGN (ESTIMATOR & REGULATOR)
Ts = 0.5; % Sample Time

% --- 1. Define Matrices for the ESTIMATOR (8 inputs) ---
A = A_full;
B_for_estimator = B_full(:, [2, 3, 5, 8, 1, 4, 6, 7]); % 4 MVs + 4 Disturbances
C_est = eye(3); 
D_est = zeros(3, 8); 

sys_est_c = ss(A, B_for_estimator, C_est, D_est);
sys_est_d = c2d(sys_est_c, Ts);

% Variables for the Discrete State-Space Block:
Ad = sys_est_d.A;      % System dynamics
Bd_est = sys_est_d.B;  % 3x8 matrix for all inputs
Cd = sys_est_d.C;      % Output matrix (identity)
Dd_est = sys_est_d.D;  % 3x8 zero matrix
Cd = eye(3); 

% --- 2. Define Matrices for the REGULATOR (4 inputs) ---
B_for_regulator = B_full(:, [2, 3, 5, 8]); % Only the 4 Valves
C_reg = eye(3);
D_reg = zeros(3, 4);

sys_reg_c = ss(A, B_for_regulator, C_reg, D_reg);
sys_reg_d = c2d(sys_reg_c, Ts);

% Variable for the MATLAB Function Block:
Bd = sys_reg_d.B;      % 3x4 matrix for QP solver

% --- A. Kalman Filter Design (The Estimator) ---
% Q_est: Process Noise, R_est: Measurement Noise[cite: 1]
Q_est = eye(3) * 0.01; 
R_est = diag([var_L, var_X, var_P]); 
[K_filter, P_cov, ~] = dlqe(Ad, eye(3), Cd, Q_est, R_est); % Discrete Kalman Gain[cite: 1]

% --- B. QP Solver Preparation (The Regulator) ---
N = 10; % Prediction Horizon[cite: 1]
Q_weight = diag([10, 2, 0.5]); % Output weights[cite: 1]
R_weight = eye(4) * 0.1;       % Input rate weights[cite: 1]

% Constraints for quadprog (Converted to deviation variables)[cite: 1]
% u_min <= u_nom + delta_u <= u_max  =>  delta_u <= u_max - u_nom
u_lb = [0; 0; 1; 100] - u_mv_nom;
u_ub = [20; 80; 380; 380] - u_mv_nom;

% --- 1. Define Augmentation Matrices ---
[n_states, n_inputs] = size(Bd); 
[n_outputs, ~] = size(Cd);

% Augmented Ad: [Ad, 0; Cd, I]
A_aug = [Ad,               zeros(n_states, n_outputs);
         Cd * Ad,          eye(n_outputs)];

% Augmented Bd: [Bd; Cd * Bd]
B_aug = [Bd; 
         Cd * Bd];

% Augmented Cd: [zeros, I] (to focus on the integral of the error)
C_aug = [zeros(n_outputs, n_states), eye(n_outputs)];

% Q_aug size: (n_states + n_outputs) x (n_states + n_outputs)
Q_aug = diag([100, 1, 50, 10, 1, 10]);
% Note: High weights on the 4th, 5th, and 6th positions penalize the integral error

disp('Manual MPC Matrices Initialized.');

%% 5. HELPER FUNCTIONS (fmincon)[cite: 1, 2]
function [c, ceq] = nonlinearconE(x, dist)
    rhoA=20; M=20; Cpar=4; Cp=0.07; lambda=38.5; UA2=6.84;
    P2=x(1); L2=x(2); X2=x(3); F2=x(4); F3=x(5); F200=x(6); P100=x(7);
    F1=dist.F1; X1=dist.X1; T1=dist.T1; T200=dist.T200;
    
    T2 = 0.5616*P2 + 0.3126*X2 + 48.43;
    T3 = 0.507*P2 + 55.0;
    T100 = 0.1538*P100 + 90.0;
    Q100 = 0.16*(F1 + F3)*(T100 - T2);
    F4 = (Q100 - F1*Cp*(T2 - T1)) / lambda;
    Q200 = UA2*(T3 - T200) / (1 + UA2/(2*Cp*F200));
    F5 = Q200 / lambda;
    
    ceq(1) = (F1 - F2 - F4) / rhoA;
    ceq(2) = (F1*X1 - F2*X2) / M;
    ceq(3) = (F4 - F5) / Cpar;
    c= [];
end 

function P = MaxProfitFunction(x, dist)
    V=20; S=0.15; W=0.05; X2min=30; lambda_s=36.6;
    P2=x(1); X2=x(3); F2=x(4); F3=x(5); F200=x(6); P100=x(7);
    F1=dist.F1; T2 = 0.5616*P2 + 0.3126*X2 + 48.43;
    T100 = 0.1538*P100 + 90.0;
    Q100 = 0.16*(F1 + F3)*(T100 - T2);
    F100 = Q100 / lambda_s;
    
    profit = F2*V - S*F100 - W*F200;
    penalty = 50 * max(0, X2min - X2)^2;
    P = -profit + penalty;
end
