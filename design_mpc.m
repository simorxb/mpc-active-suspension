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
    'OV',     [0 1 5]);

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
legend('Passive', 'MPC', 'Road profile', 'LineWidth', 1, 'FontSize', 12);
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

% Bottom-left: Suspension deflection
ax3 = subplot(2,2,3);
plot(t, y(:,2), 'r-', 'LineWidth', 2);
hold on;
plot(t_ol, y_ol(:,2), 'b-', 'LineWidth', 2);
hold off;
legend('MPC', 'Passive (fs = 0)', 'LineWidth', 1, 'FontSize', 12);
ylabel('s_d (m)', 'FontSize', 12);
xlabel('Time (s)', 'FontSize', 12);
title('Suspension deflection', 'FontSize', 12);

% Bottom-right: Control force (command vs. filtered actuator force)
ax4 = subplot(2,2,4);
plot(t, u,         'r-', 'LineWidth', 2);
hold on;
plot(t_ol, u_ol(:,2), 'b-', 'LineWidth', 2);
hold off;
legend('MPC', 'Passive (fs = 0)', 'LineWidth', 1, 'FontSize', 12);
ylabel('fs (kN)', 'FontSize', 12);
xlabel('Time (s)', 'FontSize', 12);
title('Control force', 'FontSize', 12);

%% Schematic of the quarter-car suspension model
figure('Color','w','Name','Quarter-car suspension', ...
       'NumberTitle','off','Position',[100 100 720 760]);
ax = axes; hold(ax,'on'); axis(ax,'equal'); axis(ax,'off');
xlim([-4 7]); ylim([-1.5 14]);

% Layout (figure units, arbitrary)
xc       = 1.5;                                   % vertical centerline
mb_w     = 4.0;  mb_h = 1.0;  mb_y = 9.5;         % body mass
mw_w     = 4.0;  mw_h = 0.8;  mw_y = 5.5;         % wheel mass
road_y   = 0;
spring_x = xc - mb_w/2 + 0.7;                     % suspension spring (left)
damp_x   = xc - 0.4;                              % damper (centre-left)
act_x    = xc + mb_w/2 - 0.7;                     % active actuator (right)

% --- Stylised car silhouette behind the schematic ---
drawCarSilhouette(xc, mb_y, mb_w, mb_h, mw_y, mw_h);

% --- Body mass m_b ---
rectangle('Position',[xc-mb_w/2 mb_y mb_w mb_h], ...
          'EdgeColor','k','LineWidth',2,'FaceColor','w');
text(xc, mb_y+mb_h/2, '$m_b$','Interpreter','latex', ...
     'FontSize',20,'HorizontalAlignment','center');
drawArrow(xc+mb_w/2+0.2, mb_y+mb_h/2, xc+mb_w/2+1.2, mb_y+mb_h/2);
text(xc+mb_w/2+1.35, mb_y+mb_h/2,'$x_b$','Interpreter','latex', ...
     'FontSize',16,'HorizontalAlignment','left','VerticalAlignment','middle');

% --- Wheel mass m_w ---
rectangle('Position',[xc-mw_w/2 mw_y mw_w mw_h], ...
          'EdgeColor','k','LineWidth',2,'FaceColor','w');
text(xc, mw_y+mw_h/2, '$m_w$','Interpreter','latex', ...
     'FontSize',20,'HorizontalAlignment','center');
drawArrow(xc+mw_w/2+0.2, mw_y+mw_h/2, xc+mw_w/2+1.2, mw_y+mw_h/2);
text(xc+mw_w/2+1.35, mw_y+mw_h/2,'$x_w$','Interpreter','latex', ...
     'FontSize',16,'HorizontalAlignment','left','VerticalAlignment','middle');

% --- Suspension spring k_s ---
drawSpring(spring_x, mw_y+mw_h, spring_x, mb_y, 8, 0.30);
text(spring_x-0.6,(mw_y+mw_h+mb_y)/2,'$k_s$','Interpreter','latex', ...
     'FontSize',18,'HorizontalAlignment','right');

% --- Suspension damper b_s ---
drawDamper(damp_x, mw_y+mw_h, damp_x, mb_y, 0.30);
text(damp_x-0.50,(mw_y+mw_h+mb_y)/2,'$b_s$','Interpreter','latex', ...
     'FontSize',18,'HorizontalAlignment','right');

% --- Active force actuator f_s ---
drawActuator(act_x, mw_y+mw_h, act_x, mb_y, 0.40);
text(act_x+0.55,(mw_y+mw_h+mb_y)/2,'$f_s$','Interpreter','latex', ...
     'FontSize',18,'HorizontalAlignment','left');

% --- Tire spring k_t ---
drawSpring(xc, road_y, xc, mw_y, 6, 0.30);
text(xc+0.55,(road_y+mw_y)/2,'$k_t$','Interpreter','latex', ...
     'FontSize',18,'HorizontalAlignment','left');

% --- Road (hatched ground) ---
plot([xc-2.6 xc+2.6],[road_y road_y],'k-','LineWidth',2);
hx = linspace(xc-2.4, xc+2.4, 13);
for i = 1:numel(hx)
    plot([hx(i) hx(i)-0.30],[road_y road_y-0.40],'k-','LineWidth',1);
end
% road profile arrow r
drawArrow(xc+2.9, road_y, xc+2.9, road_y+1.0);
text(xc+3.05, road_y+0.5,'$r$','Interpreter','latex', ...
     'FontSize',18,'HorizontalAlignment','left','VerticalAlignment','middle');

title(ax,'Quarter-car active suspension model','FontSize',14);
hold(ax,'off');

%% Local helper functions (must stay at end of script)
function drawSpring(x1, y1, x2, y2, n, w)
    % Zig-zag spring from (x1,y1) to (x2,y2) with n coils, lateral width w.
    L = hypot(x2-x1, y2-y1);
    if L == 0, return; end
    e = 0.12*L;                               % straight ends
    nseg = 2*n;
    ys_zig = linspace(e, L-e, nseg);
    xs_zig = w*(-1).^(1:nseg);
    xs = [0, xs_zig, 0];
    ys = [0, ys_zig, L];
    theta = atan2(y2-y1, x2-x1) - pi/2;       % rotate local y-axis to dir
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    pts = R*[xs; ys];
    plot(pts(1,:)+x1, pts(2,:)+y1, 'k-', 'LineWidth', 1.5);
end

function drawDamper(x1, y1, x2, y2, w)
    % Vertical damper symbol: cup attached to lower point, piston rod to upper.
    L = y2 - y1;
    % cup (open at the top)
    plot([x1-w x1-w x1+w x1+w], ...
         [y1+0.55*L y1 y1 y1+0.55*L], 'k-', 'LineWidth', 1.5);
    % piston rod from upper attachment
    plot([x2 x2], [y2 y1+0.30*L], 'k-', 'LineWidth', 1.5);
    % piston disc
    plot([x1-0.85*w x1+0.85*w], [y1+0.30*L y1+0.30*L], 'k-', 'LineWidth', 2);
end

function drawActuator(x1, y1, x2, y2, r)
    % Circle-with-arrow symbol for a controllable force actuator.
    yc = (y1+y2)/2;
    plot([x1 x1],[y1 yc-r],'k-','LineWidth',1.5);
    plot([x2 x2],[yc+r y2],'k-','LineWidth',1.5);
    th = linspace(0,2*pi,80);
    plot(x1+r*cos(th), yc+r*sin(th),'k-','LineWidth',1.5);
    a = 0.65*r;                                % diagonal arrow
    plot([x1-a x1+a],[yc-a yc+a],'k-','LineWidth',1.5);
    % arrowhead at upper-right end
    plot([x1+a x1+a-0.30*r],[yc+a yc+a-0.05*r],'k-','LineWidth',1.5);
    plot([x1+a x1+a-0.05*r],[yc+a yc+a-0.30*r],'k-','LineWidth',1.5);
end

function drawArrow(x1, y1, x2, y2)
    % Simple arrow from (x1,y1) to (x2,y2).
    plot([x1 x2],[y1 y2],'k-','LineWidth',1.2);
    L = hypot(x2-x1, y2-y1);
    if L == 0, return; end
    h = 0.18;                                  % head size
    ux = (x2-x1)/L; uy = (y2-y1)/L;            % unit along arrow
    px = -uy;       py = ux;                   % perpendicular
    plot([x2 x2-h*ux+0.5*h*px], [y2 y2-h*uy+0.5*h*py],'k-','LineWidth',1.2);
    plot([x2 x2-h*ux-0.5*h*px], [y2 y2-h*uy-0.5*h*py],'k-','LineWidth',1.2);
end

function drawCarSilhouette(xc, mb_y, mb_w, mb_h, mw_y, mw_h)
    % Stylised side-profile car body drawn behind the schematic, in pale blue,
    % so the active-suspension diagram clearly belongs to a vehicle corner.
    car_color = [0.15 0.40 0.80];

    % Outer body profile (hood -> windshield -> roof -> rear -> bumper)
    bx = xc + [-3.2 -2.6 -1.6 -0.2  1.6  2.4  3.0  3.2  3.2 -3.2];
    by = mb_y + [mb_h+0.05 mb_h+0.05 mb_h+1.20 mb_h+2.00 ...
                 mb_h+2.00 mb_h+1.30 mb_h+0.30 mb_h+0.05 ...
                 -0.20 -0.20];
    patch(bx, by, car_color, 'FaceAlpha', 0.14, ...
          'EdgeColor', car_color, 'EdgeAlpha', 0.55, 'LineWidth', 1.2);

    % Side windows hint
    plot([-1.4  1.4]+xc, [mb_h+1.85 mb_h+1.85]+mb_y, ...
         'Color', car_color, 'LineWidth', 1.0);
    plot([-0.1 -0.1]+xc, [mb_h+1.20 mb_h+1.95]+mb_y, ...
         'Color', car_color, 'LineWidth', 1.0);

    % Wheel arch around the wheel mass + tyre / rim hint
    th  = linspace(0, pi, 50);
    arc_r  = 0.55*mw_w;
    arc_cx = xc;
    arc_cy = mw_y + mw_h/2;
    plot(arc_cx + arc_r*cos(th), arc_cy + arc_r*sin(th), ...
         'Color', car_color, 'LineWidth', 1.2);
    th2 = linspace(0, 2*pi, 60);
    tyre_r = 0.42*mw_w;
    plot(arc_cx + tyre_r*cos(th2), arc_cy + tyre_r*sin(th2), ...
         'Color', car_color, 'LineWidth', 1.0, 'LineStyle', ':');
end