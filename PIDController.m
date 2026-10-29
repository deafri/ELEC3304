%% SPACECRAFT DOCKING: PARAMETERS FOR SIMULINK
% Plant:
% x_ddot = 3*n^2*x + 2*n*y_dot + Fx/mc
% y_ddot = -2*n*x_dot + Fy/mc
%
% x: radial displacement relative to target [m]
% y: along-track displacement relative to target [m]
%
% Controller errors:
% ex = x_ref - x
% ey = y_ref - y

%% 1. Physical parameters
mu = 3.986004418e14;    % Earth gravitational parameter [m^3/s^2]
Re = 6371e3;           % Approximate Earth radius [m]
h  = 400e3;            % Example target orbital altitude [m]

r  = Re + h;          % Orbital radius from Earth's centre [m]
mc = 100;             % Example chaser mass [kg]

n = sqrt(mu/r^3);     % Target orbital angular speed [rad/s]
T_orbit = 2*pi/n;     % Target orbital period [s]

%% 2. Initial conditions for the four plant integrators
x0  = 10;             % Initial radial displacement [m]
y0  = 0;              % Initial along-track displacement [m]
vx0 = 0;              % Initial radial coordinate rate [m/s]
vy0 = 0;              % Initial along-track coordinate rate [m/s]

%% 3. Desired relative positions
x_ref = 0;            % Desired radial displacement [m]
y_ref = 0;            % Desired along-track displacement [m]

%% 4. Plant Gain-block parameters
g_x_position = 3*n^2; % x contribution to x_ddot [1/s^2]
g_vy_to_ax   = 2*n;   % y_dot contribution to x_ddot [1/s]
g_vx_to_ay   = -2*n;  % x_dot contribution to y_ddot [1/s]
g_force     = 1/mc;   % Force-to-acceleration gain [1/kg]

%% 5. Initial PD tuning
% Approximate design neglecting coupling between axes.
% These are starting gains, not an optimised final design.
%
% wn is the desired response frequency.
% n is the orbital angular speed; they are different quantities.

zeta = 1;             % Desired nominal damping ratio
wn   = 0.01;          % Desired nominal response frequency [rad/s]

Kpx = mc*(wn^2 + 3*n^2);   % Radial proportional gain [N/m]
Kpy = mc*wn^2;             % Along-track proportional gain [N/m]

Kdx = 2*mc*zeta*wn;        % Radial derivative gain [N*s/m]
Kdy = 2*mc*zeta*wn;        % Along-track derivative gain [N*s/m]

% Start with PD control. Add integral action only after testing.
Kix = 0;                  % Radial integral gain [N/(m*s)]
Kiy = 0;                  % Along-track integral gain [N/(m*s)]

%% 6. PID derivative filtering
% Continuous parallel PID:
% Gpid(s) = Kp + Ki/s + Kd*Nf*s/(s + Nf)

Nf = 0.1;                 % Derivative filter coefficient [1/s]

% PID internal initial states
I0x = 0;
I0y = 0;
D0x = 0;
D0y = 0;

% With zero filter initial states and nonzero initial errors,
% a brief derivative-filter startup transient can occur.

%% 7. Thrust limits
% Inf gives an initial unlimited-actuator test.
% Replace with actual positive force limits before assessing feasibility.

Fx_max = Inf;             % Maximum radial force magnitude [N]
Fy_max = Inf;             % Maximum along-track force magnitude [N]

Fx_min = -Fx_max;         % Minimum radial force [N]
Fy_min = -Fy_max;         % Minimum along-track force [N]

%% 8. Simulation settings
tEnd    = 1000;           % Stop time [s]
maxStep = 1;              % Maximum solver step [s]
relTol  = 1e-6;           % Relative solver tolerance

%% 9. Example performance thresholds
% Illustrative coursework criteria, not docking standards.
% These are for assessing results; they do not tune the controller.

position_tol = 0.10;      % Maximum relative separation [m]
velocity_tol = 0.01;      % Maximum relative coordinate speed [m/s]
settling_target = 600;    % Desired settling time [s]

%% 10. Basic checks
assert(mc > 0, 'Chaser mass must be positive.');
assert(r > Re, 'Orbital radius must exceed Earth radius.');
assert(wn > 0 && zeta > 0, 'wn and zeta must be positive.');
assert(Nf > 0, 'Derivative filter coefficient must be positive.');
assert(Fx_max > 0 && Fy_max > 0, ...
    'Thrust magnitude limits must be positive.');

%% 11. Display parameter summary
fprintf('\n--- Docking model parameters ---\n');
fprintf('Chaser mass:          %.2f kg\n', mc);
fprintf('Orbital altitude:     %.2f km\n', h/1000);
fprintf('Orbital radius:       %.2f km\n', r/1000);
fprintf('Orbital angular rate: %.6f rad/s\n', n);
fprintf('Orbital period:       %.2f min\n', T_orbit/60);

fprintf('\nInitial state [x; y; vx; vy]:\n');
disp([x0; y0; vx0; vy0]);

fprintf('Radial gains:      Kp = %.6f, Ki = %.6f, Kd = %.6f\n', ...
    Kpx, Kix, Kdx);
fprintf('Along-track gains: Kp = %.6f, Ki = %.6f, Kd = %.6f\n', ...
    Kpy, Kiy, Kdy);

fprintf('Derivative filter coefficient: %.4f 1/s\n', Nf);
fprintf('Simulation stop time: %.1f s\n\n', tEnd);