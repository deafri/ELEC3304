%% SPACECRAFT DOCKING: PID PARAMETERS AND OBSERVER SETUP
% Plant: x_ddot = 3*n^2*x + 2*n*y_dot + Fx/mc
%        y_ddot = -2*n*x_dot + Fy/mc
% States: z = [x; y; x_dot; y_dot]
% PID errors: ex = x_ref - x; ey = y_ref - y.

%% Switch Between Non Linear Vs Linear
useNonlinearPlant = true;  % false = HCW; true = exact planar two-body gravity

%% 1. Physical parameters
mu = 3.986004418e14;  % Earth gravitational parameter [m^3/s^2]
Re = 6371e3;         % Approximate Earth radius [m]
h  = 400e3;          % Target orbital altitude [m]
r  = Re + h;        % Orbital radius [m]
mc = 3307;           % Chaser mass [kg]

n = sqrt(mu/r^3);   % Orbital angular speed [rad/s]
T_orbit = 2*pi/n;   % Orbital period [s]

%% 2. Actual plant initial conditions
x0  = 100;       % Relative radial displacement [m]
y0  = 100;       % Relative along-track displacement [m]
vx0 = 0;        % Relative radial coordinate rate [m/s]
vy0 = 0;        % Relative along-track coordinate rate [m/s]
z0 = [x0; y0; vx0; vy0];

%% 3. Desired relative positions
x_ref = 0;          % [m]
y_ref = 0;          % [m]

%% 4. Plant Gain-block parameters
g_x_position = 3*n^2;  % x contribution to x_ddot [1/s^2]
g_vy_to_ax   = 2*n;    % y_dot contribution to x_ddot [1/s]
g_vx_to_ay   = -2*n;   % x_dot contribution to y_ddot [1/s]
g_force     = 1/mc;    % Force-to-acceleration conversion [1/kg]

%% 5. Initial PD tuning
% Approximate design neglecting coupling; verify the complete model.
% wn is the desired response frequency, not the orbital angular speed n.
% Observer-based state feedback with integral action

zeta = 1;
wn   = 0.035;      % Desired second-order response [rad/s]
pI   = 0.030;     % Additional closed-loop pole at -0.005 [1/s]

kp = mc*(wn^2 + 2*zeta*wn*pI);
kd = mc*(2*zeta*wn + pI);

K = [kp + 3*mc*n^2,  0,    kd,       2*mc*n;
     0,              kp,  -2*mc*n,   kd];

Kix = mc*pI*wn^2;    
Kiy = Kix;

%% 6. Filter and initial states
Nf = 0.1;

I0x = 0;
I0y = 0;
D0x = 0;
D0y = 0;

%% 6. PID derivative filtering and internal initial states
% Continuous parallel PID:
% Gpid(s) = Kp + Ki/s + Kd*Nf*s/(s + Nf)
Nf = 0.1;               % Filter coefficient [1/s]
I0x = 0;
I0y = 0;
D0x = 0;
D0y = 0;


%% 7. Thrust limits

Fx_max = 400;           % [N]
Fy_max = 400;           % [N]
Fx_min = -Fx_max;
Fy_min = -Fy_max;

%% 8. Simulation settings

tEnd    = 2000;         % Stop time [s]
maxStep = 1;            % Maximum solver step [s]
relTol  = 1e-6;

%% 9. Example performance thresholds
% Assessment criteria for design 

position_tol = 0.10;    % Relative separation [m]
velocity_tol = 0.01;    % Relative coordinate speed [m/s]
settling_target = 500;  % [s]



%% 10. Plant matrices for the observer

A = [0       0    1     0;
     0       0    0     1;
     3*n^2   0    0     2*n;
     0       0   -2*n   0];

B = [0       0;
     0       0;
     1/mc    0;
     0       1/mc];

% Position-only measurement model: ym = Cm*z + Dm*u
Cm = [1 0 0 0;
      0 1 0 0];
Dm = zeros(2,2);

%% 11. Observability check

O = [Cm; Cm*A; Cm*A^2; Cm*A^3];
observability_rank = rank(O);
assert(observability_rank == 4, ...
    'The system must be fully observable for this observer design.');

%% 13. Observer gain
% Observer: zhat_dot = A*zhat + B*u + L*(ym - Cm*zhat)
% u = [Fx; Fy] is the actual applied force AFTER any saturation,
% but BEFORE the plant's 1/mc gains.
% ym = [x_measured; y_measured].
%
% Illustrative poles [1/s]; assess convergence and noise sensitivity.
p_obs = [-0.04 -0.05 -0.06 -0.07];

assert(exist('place','file') ~= 0, ...
    'Observer design requires place() from Control System Toolbox.');
L = place(A.', Cm.', p_obs).';

A_error = A - L*Cm;
observer_poles = eig(A_error);
assert(all(real(observer_poles) < 0), ...
    'Observer error dynamics must be asymptotically stable.');

%% 14. Observer initial conditions
% To show convergence 
xhat0  = 0;             % [m]
yhat0  = 0;             % [m]
vxhat0 = 0;             % [m/s]
vyhat0 = 0;             % [m/s]
zhat0 = [xhat0; yhat0; vxhat0; vyhat0];
Aobs = A_error;
Bobs = [B L];
Cobs = eye(4);
Dobs = zeros(4,4);

%% 15. Disturbance test parameters
dist_start_x = 500;     % Disturbance begins [s]
dist_end_x   = 1000;     % Used for the pulse test [s]
dist_Fx_x    = 10;     % Radial disturbance force [N]


dist_start_y = 500;     % Disturbance begins [s]
dist_end_y   = 500;     % Used for the pulse test [s]
dist_Fy    = 0;     % Radial disturbance force [N]

%% 16. Display summary
fprintf('\n--- Docking model parameters ---\n');
fprintf('Chaser mass:          %.2f kg\n', mc);
fprintf('Orbital altitude:     %.2f km\n', h/1000);
fprintf('Orbital radius:       %.2f km\n', r/1000);
fprintf('Orbital angular rate: %.6f rad/s\n', n);
fprintf('Orbital period:       %.2f min\n', T_orbit/60);
fprintf('\nActual initial state [x; y; vx; vy]:\n');
disp(z0);

fprintf('\nState-feedback gain K (rows: Fx, Fy; columns: x, y, vx, vy):\n');
disp(K);
fprintf('Integral gains: Kix = %.6f, Kiy = %.6f N/(m*s)\n', Kix, Kiy);
fprintf('Derivative filter coefficient: %.4f 1/s\n', Nf);
fprintf('Simulation stop time: %.1f s\n', tEnd);

fprintf('\n--- Observer setup ---\n');
fprintf('Observability rank: %d / 4\n', observability_rank);
fprintf('Observer gain L:\n');
disp(L);
fprintf('Requested observer poles:\n');
disp(p_obs.');
fprintf('Achieved observer poles:\n');
disp(observer_poles);
fprintf('Initial estimated state:\n');
disp(zhat0);



