function [sys,x0,str,ts] = S_Function(t,x,u,flag,xinit,par)

switch flag
    case 0
        [sys,x0,str,ts] = mdlInitializeSizes(xinit);
    case 1
        sys = mdlDerivatives(t,x,u,par);
    case 3
        sys = mdlOutputs(t,x,u);
    case {2,4,9}
        sys = [];
    otherwise
        error(['Unhandled flag = ', num2str(flag)]);
end


function [sys,x0,str,ts] = mdlInitializeSizes(xinit)

sizes = simsizes;
sizes.NumContStates  = 3;   % L2, X2, P2
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 3;   % outputs: L2, X2, P2
sizes.NumInputs      = 8;   % F1, F2, F3, X1, F200, T1, T200, P100
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);

% Initial conditions passed from Simulink block
x0 = xinit;

str = [];
ts  = [0 0];


function sys = mdlDerivatives(t,x,u,par)

% Parameters
rhoA     = par(1);   % 20
M        = par(2);   % 20
C        = par(3);   % 4
Cp       = par(4);   % 0.07
lambda   = par(5);   % 38.5
UA2      = par(6);   % 6.84
lambda_s = par(7);   % 36.6

% Inputs from Simulink
F1   = u(1);
F2   = u(2);
F3   = u(3);
X1   = u(4);
F200 = u(5);
T1   = u(6);
T200 = u(7);
P100 = u(8);

% States
L2 = x(1);
X2 = x(2);
P2 = x(3);

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


function sys = mdlOutputs(t,x,u)

% Output only the states
sys = [x(1) x(2) x(3)];