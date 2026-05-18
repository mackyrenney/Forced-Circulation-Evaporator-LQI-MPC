**Apple Concentrate Evaporator: Multivariable Control Designs**
---
Multivariable Control Implementation using MatLab/Simulink. Several Implementation Procedures are provided:
  * Linear Quadratic Regulator w/ integrators (LQI)
  * Model Predictive Control (MPC)

**Usage**
---
This implementation uses MATLAB's Control System ToolBox, ran using MATLAB_R2025b and Simulink

**Process Model**
---
The evaporator process model consists of a dynamic nonlinear system of equations relating state variables, Outlet Concentration X2, Separator Level L2, and Pressure P2 with energy and mass algebraic equations. The control objective is to maintain stable operation by regulating the product concentration, separator level, and pressure, while ensuring reasonable control throttling, constraint conditions and consistent product quality.



**Model Piping and Instrumentation Diagram**
---
<img width="557" height="450" alt="Screenshot 2026-05-13 at 6 54 14 PM" src="https://github.com/user-attachments/assets/efb69506-48d7-459f-8dd3-cfa55bae1ae8" />

### Dynamic System of Equations

```matlab
% Algebraic model equations
T2   = 0.5616*P2 + 0.3126*X2 + 48.43;
T3   = 0.507*P2 + 55.0;
T100 = 0.1538*P100 + 90.0;
Q100 = 0.16*(F1 + F3)*(T100 - T2);
F100 = Q100 / lambda_s;
F4   = (Q100 - F1*Cp*(T2 - T1)) / lambda;
Q200 = UA2*(T3 - T200) / (1 + UA2/(2*Cp*F200));
T201 = T200 + Q200/(F200*Cp);
F5   = Q200 / lambda;

% Differential equations
dL2dt = (F1 - F2 - F4) / rhoA;
dX2dt = (F1*X1 - F2*X2) / M;
dP2dt = (F4 - F5) / C;

sys = [dL2dt dX2dt dP2dt];
```
**Nominal Steady State Conditions**
---
As a guide to explore the evaporator process, the following nominal conditions are chosen. During implmentation these conditions were later modified via optimization, where a new "optimized" steady state is calculated. Additionally, constant parameters are defined and remain unchanged throughout optimization and simulation. 
```matlab
% Nominal Process Variables
% Inputs
F1   = 10; % kg/min    (feed flowrate)
F2   = 2; % kg/min     (product flowrate)
F3   = 50; % kg/min    (circulating flowrate)
X1   = 5;% percent     (inlet composition)
F200 = 208; % kg/min   (cooling water flowrate)
T1   = 40; % C         (feed temperature)
T200 = 25; % C         (cooling water inlet temp.)
P100 = 194.7; % kPa    (steam pressure)

% States
L2 = 1; % m            (separator level)
X2 = 25; % percent     (product composition)
P2 = 50.5; % kPa       (operating pressure)

% Constant Parameters
rhoA = 20; % kg/m              (separator hold-up)
M = 20; % kg                   (evaporator liquid hold-up)
Cpar = 4; % kg/kPa             (constant for vapor mass to pressure conversion)
Cp = 0.07; % kW/K(kg/min)      (heat capacity of process liquid)
lambda   = 38.5; % kW/(kg/min) (latent heat of evaporation of process liquid)
UA2 = 6.84; % kW/K             (condenser overall heat transfer coefficient)
lambda_s = 36.6; % kW/(kg/min) (latent heat of steam)

```
**State, Manipulated, and Disturbance Variable Partition**
---
A quick DOF analysis allows us to see this system has 20 variables - 12 equations = 8 DOF. Since there are 3 designated state variables (L2, X2, and P2), we first design a square system with 3 manipulated inputs and 3 state variables. The remaining 5 independent variables will represent the mathematical flexibility within the model, +1 additional mv input and +4 additional disturbance inputs. 

Manipulated Inputs: F2, F3, F200, P100 <br>
Disturbance Inputs: F1, X1, T1, T200

[Resources] 
A sidequest for users playing with the simulation may also target different varieties of MV and DV. Specifically, choosing any of the other internal variables not described in the nominal process variables, denoted in the markdown below 
``` matlab
% Additional process variables
F4 = 8.0 % kg/min   (vapor flowrate)
F5 = 8.0 % kg/min   (condensate flowrate)
T2 = 84.6 % C       (product temperature)
T3 = 80.6 % C       (circulating temperature)
F100 = 9.3 % kg/min (steam flowrate)
T100 = 119.9 % C    (steam temperature)
Q100 = 339.0 % kW   (heater duty)
T201 = 46.1  % C    (cooling water outlet temp)
Q200 = 307.9 % kW   (condenser duty)
```
**System Constraints**
---
The safety constraints on P2, P100, F200, F3, L2 must be respected at all times, i.e. these are hard constraints. The constraint on the product purify X2 must be respected on average. Hence small violations in dynamic simulations of the constraint can be
accepted for a short time. Choose your products constraint in the interval X2min ∈[20; 35].
``` matlab
% Decision variables: [P2, L2, X2, F2, F3, F200, P100]
xMin = [40,   0.3,  20,   0,   0,   1,    100]; 
xMax = [80,   2.0,  100,  20,  80, 380,  380];
```

 
**Optimization Function**
---
The evaporator system with system constraints transforms into a constrained optimization problem when a profit function is applied. The cost–profit function was formulated to maximize the economic performance of the system while satisfying all process constraints. The profitability metric is deonted by quantity F2 of product outflow multiplied by the unit price per kilogram of apple concentrate (V). The costs are denoted by the steam cost (S) and the water cost (W) multiplied by their respective process values. The electrical energy consumption of the pumps (F2 and F3) is relatively small compared to the thermal energy demand of the evaporator. For a small-scale installation with a feed flow of 10 kg/min, the total pump power is estimated to be approximately 0.7 kW which is assumed as a negligible contribution. A penalty term is added regarding the extra quality constraint of X2min.
``` matlab
profit = F2*V - S*F100 - W*F200;
penalty = 50 * max(0, X2min - X2)^2;
P = -profit + penalty;
```
**LQI: Simulink Design**
---
The non-linear system is linearized around the optimized steady state using the "dummy" script.
``` matlab
model_name = 'Evaporator_linearized_LQI_updated';
if ~bdIsLoaded(model_name), load_system(model_name); end
set_param(model_name, 'LoadExternalInput', 'off');

[A_full, B_full, C_full, D_full] = linmod(model_name, x_nom, u_full_nom);
```
Then the system is augmented including state integrators, and tuned accordingly with state penalities Q and input penalties R.

$`
\mathbf{A_I} =\begin{bmatrix} \mathbf{A}&\mathbf{0} \\ -\mathbf{C}&\mathbf{0}\end{bmatrix},
\mathbf{B_I} =\begin{bmatrix} \mathbf{B_u} \\ \mathbf{0}\end{bmatrix},
\mathbf{C_I} =\begin{bmatrix} \mathbf{C}& \mathbf{0}\end{bmatrix},
\mathbf{Q_I} = \begin{bmatrix}\mathbf{Q} & 0 \\ 0 & \mathbf{Q_s} \end{bmatrix},
\mathbf{R_I} = \begin{bmatrix}\mathbf{R}\end{bmatrix}
`$

and stabilizing gain ($`K_x`$) and reference tracking gain ($`K_i`$) are calculated. 
``` matlab
sys_plant = ss(A, B, C, D);
K_lqi = lqi(sys_plant, Q, R);

% Split K_lqi into State Feedback (K_x) and Integral Tracking (K_i)
K_x = K_lqi(:, 1:3); 
K_i = K_lqi(:, 4:6); 
```

The following system dynamics considers additive band-limited white noise on measurement states and disturbance variables and follows the discrete-time state-space equations.
 
$`x_{k+1} = Ax_k + B_uu_k + B_dd_k`$

$`y_k = Cx_k + v_k`$
 
The resulting LQI model is presented and simulated in Simulink:
<img width="720" height="451" alt="Screenshot 2026-05-13 at 8 18 53 PM" src="https://github.com/user-attachments/assets/d320652d-25dc-49f4-ac3e-1999b73c8c99" />



**MPC: Simulink Design**
---
The model predictive control structure follows the same linearization scheme with the "dummy" Simulink script as the LQI. 

``` matlab
model_name = 'Evaporator_linearized_LQI_updated';
if ~bdIsLoaded(model_name), load_system(model_name); end
[A_full, B_full, C_full, D_full] = linmod(model_name, x_nom, u_full_nom);
```

The Kalman Estimator calculates the discrete kalman gain upon initialization:

``` matlab
% Q_est: Process Noise, R_est: Measurement Noise
[K_filter, P_cov, ~] = dlqe(Ad, eye(3), Cd, Q_est, R_est); % Discrete Kalman Gain
```

Then, the Kalman Filter Block, compares live plant data consisting of noisy measurements and disturbances with the prediction from the internal physical model, and filters the innovation by scaling the error by the discrete kalman gain. The resulting $`x_hat_aug`$ are clean estimated state vectors.

The regulator optimizes valve throttling using a QP solver in the MPC_regulator Simulink block.

The cost landscape for the QP problem is defined by the Hessian (H). The input penalties (R) penalize aggressive valve movements and the tracking cost (Bd_aug' * Q_aug * Bd_aug) calculates how much the state error will increase for given valve changes.

``` matlab
H_raw = 2 * (Bd_aug' * Q_aug * Bd_aug + R);
H = (H_raw + H_raw') / 2; % Force symmetry
```

The corrected x_hat_aug passes through the line gradient vector (f) for QP formulation. This gives the QP algorithm an optimal trajectory for ideal valve movements $\Delta u$ to reject disturbance and stabilize the evaporator. The 1-step free response (Ad_aug * x_hat_aug) shows what the next step will be without valve movement. The tracking penalty (Q_aug) penalizes variation in states based on weighted tuning. The input projection (Bd_aug') described how moving the valves changes the states. Therefore, Bd_aug' * Q_aug, represents ideal valve arrangements to fix predicted error.

``` matlab
f = 2 * Bd_aug' * Q_aug * (Ad_aug * x_hat_aug);
```

With actuator constraints:
``` matlab
% Constraints for quadprog (Converted to deviation variables)
% u_min <= u_nom + delta_u <= u_max  =>  delta_u <= u_max - u_nom
u_lb = [0; 0; 1; 100] - u_mv_nom;
u_ub = [20; 80; 380; 380] - u_mv_nom;
```
and state constraint on level:
``` matlab
A_ineq = [-B_L2; B_L2];
b_ineq = [-(L2_limit_min - x_hat_aug(1)); (L2_limit_max - x_hat_aug(1))];
```

Using Matlab's quadprog function 
``` matlab
[tmp, ~, exitflag] = quadprog(H, f, A_ineq, b_ineq, [], [], u_lb, u_ub, x0, opts);
```

The resulting MPC model is presented and simulated in Simulink:
<img width="915" height="619" alt="Screenshot 2026-05-13 at 8 21 56 PM" src="https://github.com/user-attachments/assets/87317ba0-24b8-47da-83e0-8f1e97f311b7" />








