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
The evaporator process model consists of a dynamic nonlinear system of equations relating state variables, Outlet Concentration X2, Evaporator Level L2, and Pressure P2 with energy and mass algebraic equations. The control objective is to maintain stable operation by regulating the product concentration, separator level, and pressure, while ensuring reasonable control throttling, constraint conditions and consistent product quality.



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


**LQI: Simulink Design**
---
<img width="720" height="451" alt="Screenshot 2026-05-13 at 8 18 53 PM" src="https://github.com/user-attachments/assets/d320652d-25dc-49f4-ac3e-1999b73c8c99" />



**MPC: Simulink Design**
---
<img width="915" height="619" alt="Screenshot 2026-05-13 at 8 21 56 PM" src="https://github.com/user-attachments/assets/87317ba0-24b8-47da-83e0-8f1e97f311b7" />








