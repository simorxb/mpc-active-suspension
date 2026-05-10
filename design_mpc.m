%% State space model
% Quarter-car state matrices (4 states, 2 inputs, 2 outputs)
A0 = [ 0 1 0 0; [-ks -bs ks bs]/mb ; ...
       0 0 0 1; [ks bs -ks-kt -bs]/mw];
B0 = [ 0 0; 0 1e3/mb ; 0 0 ; [kt -1e3]/mw];

% Augment with a fast actuator lag fs_actual = 1/(tau*s+1) * fs_cmd.
% MPC Toolbox does not allow direct feedthrough from the MV to any output;
% the lag pushes the contribution of fs into a state, so D becomes zero.
% tau is chosen small enough that closed-loop performance is unaffected.

% Actuator time const (s)
tau = 0.005;

A = [ A0,         B0(:,2);
      zeros(1,4) -1/tau   ];
B = [ B0(:,1)     zeros(4,1);
      0           1/tau    ];

% Output matrices (xb, sd, ab)
C = [ 1 0 0 0 0;
      1 0 -1 0 0;
      [-ks -bs ks bs]/mb 1e3/mb ];
D = zeros(3,2);

qcar = ss(A,B,C,D);
qcar.StateName = ["body travel (m)";"body vel (m/s)";...
          "wheel travel (m)";"wheel vel (m/s)";...
          "fs actual (kN)"];
qcar.InputName = ["r";"fs"];
qcar.OutputName = ["xb";"sd";"ab"];

%% MPC setup
% Specify signal types
% Input 1: r  -> unmeasured disturbance (UD)
% Input 2: fs -> manipulated variable    (MV)
qcar = setmpcsignals(qcar, 'UD', 1, 'MV', 2);

% Sample time and horizons
Ts = 0.01;
p  = 40;
m  = 10;

% Manipulated variable: fs is scaled by 1e3 in B, so 1 unit of fs = 1000 N
MV = struct('Min', -5, 'Max', 5, 'ScaleFactor', 2);

% Output variables: no hard limits, just sensible scale factors (xb [m], sd [m], ab [m/s^2])
OV(1) = struct('Min', -Inf, 'Max', Inf, 'ScaleFactor', 0.05);
OV(2) = struct('Min', -Inf, 'Max', Inf, 'ScaleFactor', 0.05);
OV(3) = struct('Min', -Inf, 'Max', Inf, 'ScaleFactor', 1);

% Weights: drive both sd and ab to 0, penalise MV rate to avoid chatter
Weights = struct( ...
    'MV',     0, ...
    'MVRate', 0.1, ...
    'OV',     [0 3 5]);

% Build the controller
mpcobj = mpc(qcar, Ts, p, m, Weights, MV, OV);

%% Simulation
% Simulate with a road bump as unmeasured disturbance
% Simulation time (s)
Tsim  = 3;
% Simulation steps
steps = round(Tsim/Ts);

% Reference (we want xb = 0, sd = 0 and ab = 0)
ref      = zeros(steps, 3);

% Road profile: smooth half-cosine bump (1 - cos) of amplitude 0.05 m
r_signal = zeros(steps, 1);
N_bump   = round(0.25/Ts);
t_bump   = (0:N_bump-1)'*Ts;
r_signal(1:N_bump) = 0.025*(1-cos(8*pi*t_bump));

% Simulation options
simopt = mpcsimopt(mpcobj);
% Unmeasured disturbance
simopt.UnmeasuredDisturbance = r_signal;

% Simulate the controller
[y, t, u, xp] = sim(mpcobj, steps, ref, simopt);
% Filtered (applied) force
fs_actual = xp(:,5);

%% Open loop (passive suspension, fs = 0)
% Same disturbance into the augmented plant with the actuator off.
u_ol      = [r_signal, zeros(steps,1)];
[y_ol, t_ol, x_ol] = lsim(qcar, u_ol, (0:steps-1)'*Ts);

%% Plot
figure;

% Top-left: Body travel and road profile
ax1 = subplot(2,2,1);
plot(t_ol, y_ol(:,1), 'b-', 'LineWidth', 2);
hold on;
plot(t, xp(:,1), 'r-',  'LineWidth', 2);
plot(t, r_signal, 'g-', 'LineWidth', 2);
hold off;
legend('Passive (fs = 0)', 'MPC', 'Road profile', 'LineWidth', 1, 'FontSize', 12);
ylabel('x_b (m)', 'FontSize', 12);
xlabel('Time (s)', 'FontSize', 12);
title('Body travel', 'FontSize', 12);

% Top-right: Body acceleration (MPC vs passive open-loop)
ax2 = subplot(2,2,2);
plot(t,    y(:,3),    'r-', 'LineWidth', 2);
hold on;
plot(t_ol, y_ol(:,3), 'b-', 'LineWidth', 2);
hold off;
legend('MPC', 'Passive (fs = 0)', 'LineWidth', 1, 'FontSize', 12);
ylabel('a_b (m/s^2)', 'FontSize', 12);
xlabel('Time (s)', 'FontSize', 12);
title('Body acceleration', 'FontSize', 12);

% Bottom-left: Suspension deflection (MPC vs passive open-loop)
ax3 = subplot(2,2,3);
plot(t, y(:,2), 'r-', 'LineWidth', 2);
hold on;
plot(t_ol, y_ol(:,2), 'b-', 'LineWidth', 2);
hold off;
legend('MPC', 'Passive (fs = 0)', 'LineWidth', 1, 'FontSize', 12);
ylabel('s_d (m)', 'FontSize', 12);
xlabel('Time (s)', 'FontSize', 12);
title('Suspension deflection', 'FontSize', 12);

% Bottom-right: Control force (MPC vs passive open-loop)
ax4 = subplot(2,2,4);
plot(t, u,         'r-', 'LineWidth', 2);
hold on;
plot(t_ol, u_ol(:,2), 'b-', 'LineWidth', 2);
hold off;
legend('MPC', 'Passive (fs = 0)', 'LineWidth', 1, 'FontSize', 12);
ylabel('fs (kN)', 'FontSize', 12);
xlabel('Time (s)', 'FontSize', 12);
title('Control force', 'FontSize', 12);