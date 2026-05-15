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
F1   = 10; % kg/min
F2   = 2; % kg/min
F3   = 50; % kg/min
X1   = 5;% percent
F200 = 208; % kg/min
T1   = 40; % C
T200 = 25; % C
P100 = 194.7; % kPa

% States
L2 = 1; % m
X2 = 25; % percent
P2 = 50.5; % kPa

% Constant Parameters
rhoA = 20; % kg/m
M = 20; % kg
Cpar = 4; % kg/kPa
Cp = 0.07; % kW/K(kg/min)
lambda   = 38.5; % kW/(kg/min)
UA2 = 6.84; % kW/K
lambda_s = 36.6; % kW/(kg/min)

```
**State, Manipulated, and Disturbance Variable Partition**
---
A quick DOF analysis allows us to show our system has 20 variables - 12 equations = 8 DOF. Since there are 3 designated state variables (L2, X2, and P2), we first design a square system with 3 manipulated inputs and 3 state variables. The remaining 5 independent variables will represent the mathematical flexibility within our model, +1 additional mv input and +4 additional disturbance inputs. 

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
**Optimization Function**
---


**System Constraints**
---

**LQI: Simulink Design**
---
<img width="720" height="451" alt="Screenshot 2026-05-13 at 8 18 53 PM" src="https://github.com/user-attachments/assets/d320652d-25dc-49f4-ac3e-1999b73c8c99" />



**MPC: Simulink Design**
---
<img width="915" height="619" alt="Screenshot 2026-05-13 at 8 21 56 PM" src="https://github.com/user-attachments/assets/87317ba0-24b8-47da-83e0-8f1e97f311b7" />








