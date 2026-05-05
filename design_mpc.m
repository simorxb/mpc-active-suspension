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
C = [ 1 0 -1 0           0;                        % sd
      [-ks -bs ks bs]/mb 1e3/mb ];                 % ab
D = zeros(2,2);

qcar = ss(A,B,C,D);
qcar.StateName = ["body travel (m)";"body vel (m/s)";...
          "wheel travel (m)";"wheel vel (m/s)";...
          "fs actual (kN)"];
qcar.InputName = ["r";"fs"];
qcar.OutputName = ["sd";"ab"];

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
OV(1) = struct('Min', -Inf, 'Max', Inf, 'ScaleFactor', 0.05);  % sd [m]
OV(2) = struct('Min', -Inf, 'Max', Inf, 'ScaleFactor', 1);    % ab [m/s^2]

%% Weights: drive both sd and ab to 0, penalise MV rate to avoid chatter
Weights = struct( ...
    'MV',     0, ...
    'MVRate', 0.1, ...
    'OV',     [1 1]);

%% Build the controller
mpcobj = mpc(qcar, Ts, p, m, Weights, MV, OV);

%% Simulate with a road bump as unmeasured disturbance
Tsim  = 5;                                       % simulation time (s)
steps = round(Tsim/Ts);                          % simulation iterations

ref      = zeros(steps, 2);                      % we want sd = 0 and ab = 0
r_signal = zeros(steps, 1);                      % road profile
r_signal(round(1/Ts):round(1.1/Ts)) = 0.05;      % 5 cm bump for 100 ms

simopt = mpcsimopt(mpcobj);
simopt.UnmeasuredDisturbance = r_signal;

[y, t, u, xp] = sim(mpcobj, steps, ref, simopt);
fs_actual = xp(:,5);                             % filtered (applied) force

%% Plot (black background)
figure('Color', 'k');

% Suspension deflection
ax1 = subplot(3,1,1);
stairs(t, y(:,1), 'm-', 'LineWidth', 2);
hold on;
stairs(t, ref(:,1), 'c--', 'LineWidth', 2);
hold off;
legend('Feedback', 'Setpoint', 'TextColor', 'w', 'Color', 'k', 'EdgeColor', ...
    [0.5 0.5 0.5], 'LineWidth', 1, 'FontSize', 12);
ylabel('sd (m)', 'Color', 'w', 'FontSize', 12);
title('Suspension deflection', 'Color', 'w', 'FontSize', 12);
grid on;
ax1.Color = 'k'; ax1.GridColor = 'w'; ax1.GridAlpha = 0.3;
ax1.XColor = 'w'; ax1.YColor = 'w';

% Body acceleration
ax2 = subplot(3,1,2);
stairs(t, y(:,2), 'm-', 'LineWidth', 2);
hold on;
stairs(t, ref(:,2), 'c--', 'LineWidth', 2);
hold off;
legend('Feedback', 'Setpoint', 'TextColor', 'w', 'Color', 'k', 'EdgeColor', ...
    [0.5 0.5 0.5], 'LineWidth', 1, 'FontSize', 12);
ylabel('ab (m/s^2)', 'Color', 'w', 'FontSize', 12);
title('Body acceleration', 'Color', 'w', 'FontSize', 12);
grid on;
ax2.Color = 'k'; ax2.GridColor = 'w'; ax2.GridAlpha = 0.3;
ax2.XColor = 'w'; ax2.YColor = 'w';

% Control effort: command vs. filtered actuator force
ax3 = subplot(3,1,3);
stairs(t, u,         'g-',  'LineWidth', 2);
hold on;
stairs(t, fs_actual, 'm-', 'LineWidth', 2);
hold off;
legend('Command', 'Filtered', 'TextColor', 'w', 'Color', 'k', 'EdgeColor', ...
    [0.5 0.5 0.5], 'LineWidth', 1, 'FontSize', 12);
ylabel('fs (kN)', 'Color', 'w', 'FontSize', 12);
xlabel('Time (s)', 'Color', 'w', 'FontSize', 12);
title('Control effort', 'Color', 'w', 'FontSize', 12);
grid on;
ax3.Color = 'k'; ax3.GridColor = 'w'; ax3.GridAlpha = 0.3;
ax3.XColor = 'w'; ax3.YColor = 'w';
