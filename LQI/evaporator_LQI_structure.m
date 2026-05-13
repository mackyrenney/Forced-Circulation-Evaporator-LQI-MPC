%% 1. INITIALIZATION & DISTURBANCES
clear variables; clc;

rhoA     = 20;
M        = 20;
Cpar     = 4; 
Cp       = 0.07;
lambda   = 38.5;
UA2      = 6.84;
lambda_s = 36.6; 
par = [rhoA, M, Cpar, Cp, lambda, UA2, lambda_s];

% Measurement Noise variance
var_L = 0.01; 
var_X = 0.05; 
var_P = 1;  

% disturbance variables
dist.F1   = 10.0; 
dist.X1   = 5.0;  
dist.T1   = 40.0; 
dist.T200 = 25.0; 

% Create "Dummy" versions of LQI variables so linmod doesn't crash
K_x = zeros(4,3); 
K_i = zeros(4,3); 
u_mv_nom = zeros(4,1); 
x_nom = zeros(3,1);
L2_opt = 1; X2_opt = 30; P2_opt = 50; 

%% STEADY-STATE OPTIMIZATION (fmincon)

% Decision variables: [P2, L2, X2, F2, F3, F200, P100]
xMin = [40,   0.3,  20,   0,   0,   1,    100]; 
xMax = [80,   2.0,  100,  20,  80, 380,  380]; 
xInitial = [50.5, 1.0, 35.0, 8.0, 50.0, 208.0, 194.7];

options = optimoptions('fmincon', 'Display', 'none', 'Algorithm', 'sqp');

[xOpt, fval] = fmincon(@(x) MaxProfitFunction(x, dist), xInitial, [], [], [], [], xMin, xMax, @(x) nonlinearconE(x, dist), options);

% Extract Optimized Setpoints (Nominal Operating Point)
P2_opt   = xOpt(1);
L2_opt   = xOpt(2);
X2_opt   = xOpt(3);
F2_opt   = xOpt(4);
F3_opt   = xOpt(5);
F200_opt = xOpt(6);
P100_opt = xOpt(7);

% Re-assign actual variables for Simulink and linmod
x_nom = [L2_opt; X2_opt; P2_opt]; 
xinit = x_nom; % This fixes your S-function xinit error
u_full_nom = [dist.F1 ; F2_opt; F3_opt; dist.X1; F200_opt; dist.T1; dist.T200; P100_opt];
u_mv_nom = u_full_nom([2, 3, 5, 8]);

fprintf('Steady-State Found. Profit: %.2f DKK/min\n', -fval);
t0 = 0;
tfin = 5000;

%% 3. OPEN-LOOP LINEARIZATION
% NOTE: Ensure 'Evaporator_Simulink_OpenLoop' has NO PID blocks.
% It should just be the S-Function block with inputs/outputs.
model_name = 'Evaporator_linearized_LQI_updated';
if ~bdIsLoaded(model_name), load_system(model_name); end
set_param(model_name, 'LoadExternalInput', 'off');

[A_full, B_full, C_full, D_full] = linmod(model_name, x_nom, u_full_nom);

% Extract B matrix for Manipulated Variables (MV) only
% MV Indices: 2(F2), 3(F3), 5(F200), 8(P100)
mv_idx = [2, 3, 5, 8];
A = A_full;
B = B_full(:, mv_idx);
C = eye(3); % Controlling all three states: L2, X2, P2
D = zeros(3, 4);

%% 4. LQI DESIGN (Optimal Gains)
% Augmented state vector: [L2, X2, P2, integral_L2, integral_X2, integral_P2]

%% 4.5 BRYSONS RULE 
% --- Bryson's Rule Tuning ---

dev_L2 = 0.4;    
dev_X2 = 3.0;    
dev_P2 = 20.0;   


dev_intL = 10.0;  
dev_intX = 10.0;  
dev_intP = 40.0;  


Q = diag([1/dev_L2^2, 1/dev_X2^2, 1/dev_P2^2, ...
          1/dev_intL^2, 1/dev_intX^2, 1/dev_intP^2]);

dev_F2 = 1.0;
dev_F3 = 1.0;
dev_F200 = 1.0;
dev_P100 = 10.0;

R = diag([1/dev_F2^2, 1/dev_F3^2, 1/dev_F200^2, 1/dev_P100^2]);


sys_plant = ss(A, B, C, D);
K_lqi = lqi(sys_plant, Q, R);

% Split K_lqi into State Feedback (K_x) and Integral Tracking (K_i)
K_x = K_lqi(:, 1:3); 
K_i = K_lqi(:, 4:6); 

% Nominal MV values to be used as Feed-Forward bias in Simulink
u_mv_nom = u_full_nom(mv_idx);

disp('LQI Gains Calculated. Ready for Simulink.');

%% 5. HELPER FUNCTIONS (fmincon)
function [c, ceq] = nonlinearconE(x, dist)
    rhoA=20; M=20; Cpar=4; Cp=0.07; lambda=38.5; UA2=6.84;
    P2=x(1); L2=x(2); X2=x(3); F2=x(4); F3=x(5); F200=x(6); P100=x(7);
    F1=dist.F1; X1=dist.X1; T1=dist.T1; T200=dist.T200;
    
    T2   = 0.5616*P2 + 0.3126*X2 + 48.43;
    T3   = 0.507*P2 + 55.0;
    T100 = 0.1538*P100 + 90.0;
    Q100 = 0.16*(F1 + F3)*(T100 - T2);
    F4   = (Q100 - F1*Cp*(T2 - T1)) / lambda;
    Q200 = UA2*(T3 - T200) / (1 + UA2/(2*Cp*F200));
    F5   = Q200 / lambda;
    
    ceq(1) = (F1 - F2 - F4) / rhoA;
    ceq(2) = (F1*X1 - F2*X2) / M;
    ceq(3) = (F4 - F5) / Cpar;
    c= [];
end 

function P = MaxProfitFunction(x, dist)
    UA2=6.84; lambda_s=36.6; V=20; S=0.15; W=0.05; X2min=30; Cp=0.07;
    P2=x(1); X2=x(3); F2=x(4); F3=x(5); F200=x(6); P100=x(7);
    F1=dist.F1; T1=dist.T1; T200=dist.T200;
    
    T2   = 0.5616*P2 + 0.3126*X2 + 48.43;
    T100 = 0.1538*P100 + 90.0;
    Q100 = 0.16*(F1 + F3)*(T100 - T2);
    F100 = Q100 / lambda_s;
    
    profit = F2*V - S*F100 - W*F200;
    penalty = 50 * max(0, X2min - X2)^2;
    P = -profit + penalty;
end

Co = ctrb(A, B);
rank_of_Co = rank(Co);

% Create the augmented system
A_aug = [A, zeros(3,3); -C, zeros(3,3)];
B_aug = [B; zeros(3,4)];

% Calculate the rank of the augmented system
r = rank(ctrb(A_aug, B_aug));
fprintf('The rank of the Augmented Controllability Matrix is: %d\n', r);

n_states = size(A_aug, 1);
r = rank(ctrb(A_aug, B_aug));

fprintf('System has %d states. Controllability Rank: %d\n', n_states, r);

if r == n_states
    fprintf('Result: FULLY CONTROLLABLE. Tuning weights should work.\n');
else
    fprintf('Result: UNCONTROLLABLE! Check your C matrix or reference wiring.\n');
end

Co_sub = ctrb(A(2,2), B(2,:));
fprintf('Rank of Concentration control: %d\n', rank(Co_sub));