clear;
clc;

% ----- Physical parameters: illustrative values -----
mu = 3.986004418e14;  % Earth's gravitational parameter [m^3/s^2]
Re = 6371e3;         % Approximate Earth radius [m]
h  = 400e3;          % Target orbital altitude [m]
r  = Re + h;        % Target orbital radius [m]

mc = 100;           % Chaser mass [kg]
n  = sqrt(mu/r^3);   % Target angular speed [rad/s]

% ----- Linearised plant -----
% States: [x; y; x_dot; y_dot]
A = [0       0    1     0;
     0       0    0     1;
     3*n^2   0    0     2*n;
     0       0   -2*n   0];

% Inputs: [Fx; Fy]
B = [0       0;
     0       0;
     1/mc    0;
     0       1/mc];

% Output all states for feedback
C = eye(4);
D = zeros(4,2);

% ----- Initial conditions -----
z0 = [10; 0; 0; 0];  % [m; m; m/s; m/s]

% ----- Simulation duration -----
tEnd = 1000;         % seconds

% Check controllability
Wc = ctrb(A,B);
rankWc = rank(Wc)       % Expected: 4

% Illustrative closed-loop poles [1/s]
poles = [-0.01 -0.012 -0.014 -0.016];

% State-feedback gain: 2-by-4 matrix
K = place(A,B,poles);

% Check achieved closed-loop poles
closedLoopPoles = eig(A-B*K)