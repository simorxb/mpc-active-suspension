% Quarter-car state matrices (4 states, 2 inputs, 2 outputs)
A0 = [ 0 1 0 0; [-ks -bs ks bs]/mb ; ...
       0 0 0 1; [ks bs -ks-kt -bs]/mw];
B0 = [ 0 0; 0 1e3/mb ; 0 0 ; [kt -1e3]/mw];

% Augment with a fast actuator lag fs_actual = 1/(tau*s+1) * fs_cmd.
% MPC Toolbox does not allow direct feedthrough from the MV to any output;
% the lag pushes the contribution of fs into a state, so D becomes zero.
% tau is chosen small enough that closed-loop performance is unaffected.
tau = 0.005;                                       % actuator time const (s)

A = [ A0,         B0(:,2);
      zeros(1,4) -1/tau   ];
B = [ B0(:,1)     zeros(4,1);
      0           1/tau    ];
C = [ 1 0 0 0 0;  % xb
      1 0 -1 0 0;                        % sd
      [-ks -bs ks bs]/mb 1e3/mb ];                 % ab
D = zeros(3,2);

qcar = ss(A,B,C,D);
qcar.StateName = ["body travel (m)";"body vel (m/s)";...
          "wheel travel (m)";"wheel vel (m/s)";...
          "fs actual (kN)"];
qcar.InputName = ["r";"fs"];
qcar.OutputName = ["xb";"sd";"ab"];

%% Specify signal types
% Input 1: r  -> unmeasured disturbance (UD)
% Input 2: fs -> manipulated variable    (MV)
qcar = setmpcsignals(qcar, 'UD', 1, 'MV', 2);

%% Sample time and horizons
Ts = 0.025;     % sampling time (s) - fast enough for the wheel-hop mode
p  = 20;        % prediction horizon (~0.5 s look-ahead)
m  = 10;         % control horizon

%% Manipulated variable: fs is scaled by 1e3 in B, so 1 unit of fs = 1000 N
MV = struct('Min', -5, 'Max', 5, 'ScaleFactor', 2);

%% Output variables: no hard limits, just sensible scale factors
OV(1) = struct('Min', -Inf, 'Max', Inf, 'ScaleFactor', 0.05);  % xb [m]
OV(2) = struct('Min', -Inf, 'Max', Inf, 'ScaleFactor', 0.05);  % sd [m]
OV(3) = struct('Min', -Inf, 'Max', Inf, 'ScaleFactor', 1);    % ab [m/s^2]

%% Weights: drive both sd and ab to 0, penalise MV rate to avoid chatter
Weights = struct( ...
    'MV',     0, ...
    'MVRate', 0.1, ...
    'OV',     [0 1 1]);

%% Build the controller
mpcobj = mpc(qcar, Ts, p, m, Weights, MV, OV);

%% Simulate with a road bump as unmeasured disturbance
Tsim  = 1;                                       % simulation time (s)
steps = round(Tsim/Ts);                          % simulation iterations

ref      = zeros(steps, 3);                      % we want xb = 0, sd = 0 and ab = 0

% road profile
r_signal = zeros(steps, 1);
r_signal(1:round(0.25/Ts)) = 0.025*(1-cos(8*pi*t(1:round(0.25/Ts))));

simopt = mpcsimopt(mpcobj);
simopt.UnmeasuredDisturbance = r_signal;

[y, t, u, xp] = sim(mpcobj, steps, ref, simopt);
fs_actual = xp(:,5);                             % filtered (applied) force

%% Plot (black background, 2x2 layout)
figure;

% Top-left: Body travel and road profile
ax1 = subplot(2,2,1);
plot(t, xp(:,1), 'b-',  'LineWidth', 2);
hold on;
plot(t, r_signal, 'g-', 'LineWidth', 2);
hold off;
legend('Body travel', 'Road profile', 'LineWidth', 1, 'FontSize', 12);
ylabel('x_b (m)', 'FontSize', 12);
xlabel('Time (s)', 'FontSize', 12);
title('Body travel', 'FontSize', 12);
%grid on;

% Top-right: Body acceleration
ax2 = subplot(2,2,2);
plot(t, y(:,2), 'b-', 'LineWidth', 2);
ylabel('a_b (m/s^2)', 'FontSize', 12);
xlabel('Time (s)', 'FontSize', 12);
title('Body acceleration', 'FontSize', 12);
%grid on;

% Bottom-left: Suspension deflection
ax3 = subplot(2,2,3);
plot(t, y(:,1), 'b-', 'LineWidth', 2);
ylabel('s_d (m)', 'FontSize', 12);
xlabel('Time (s)', 'FontSize', 12);
title('Suspension deflection', 'FontSize', 12);
%grid on;

% Bottom-right: Control force (command vs. filtered actuator force)
ax4 = subplot(2,2,4);
plot(t, u,         'b-', 'LineWidth', 2);
% legend('Command', 'LineWidth', 1, 'FontSize', 12);
ylabel('fs (kN)', 'FontSize', 12);
xlabel('Time (s)', 'FontSize', 12);
title('Control force', 'FontSize', 12);
%grid on;
