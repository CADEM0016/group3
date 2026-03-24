function fus = fuselageStructures(adp)
% FUSELAGESTRUCTURES
% Preliminary fuselage structural design model using ADP inputs.
%
% Inputs:
%   adp  - Unconventional.ADP() object
% Outputs:
%   fus  - struct containing load cases, sizing results, stresses, masses
%  OPTIONS
makePlots = true;
verbose   = true;
%  CONSTANTS
%% ==========================================================
g = 9.81;    % gravity [m/s^2]

%% ==========================================================
%  INPUTS FROM ADP
%% ==========================================================
Lf         = adp.FuselageLength;      % [m]
r_fus      = adp.CabinRadius;         % [m]
Span       = adp.Span;                % [m]
Sw         = adp.WingArea;            % [m^2]
m_MTOW     = adp.MTOM;                % [kg]
m_OEM      = adp.OEM;                 % [kg]

xw         = adp.WingStation;         % [m]
xt         = adp.TailStation;         % [m]

m_payload  = adp.PayloadMass;         % [kg]
m_crew     = adp.CrewMass;            % [kg]

xPayload1  = adp.PayloadStart;        % [m]
xPayload2  = adp.PayloadEnd;          % [m]
xCG_rest   = adp.RestMassCG;          % [m]

p_diff     = adp.PressureDiff;        % [Pa]

sigma_allow = adp.SigmaAllow;         % [Pa]
E           = adp.YoungsModulus;      % [Pa]
G           = adp.ShearModulus;       % [Pa]
rho_al      = adp.MaterialDensity;    % [kg/m^3]

t_min_gauge = adp.MinGauge;           % [m]
frame_spacing = adp.FrameSpacing;     % [m]
N_stringers   = adp.NumStringers;     % [-]

Kg        = adp.GustKg;               % [-]
wg_up     = adp.GustSpeedUp;          % [m/s]
wg_dn     = adp.GustSpeedDown;        % [m/s]
V         = adp.CruiseEAS;            % [m/s]

%% ==========================================================
%  LOCAL / FALLBACK ASSUMPTIONS
%% ==========================================================
if isempty(m_OEM)
    m_OEM = 197395;
end

% Fuel mass not yet stored in ADP; keep local for now
m_fuel = 161456;   % [kg]

% Gust air density and lift-curve slope
rho = 1.225;       % [kg/m^3]
a   = 5.5;         % [1/rad]

% Half-wing resultant at 50% of semi-span => b/4
yResFracSemi = 0.5;

%% ==========================================================
%  GEOMETRY / DISCRETISATION
%% ==========================================================
N  = 401;
x  = linspace(0, Lf, N);
dx = x(2) - x(1);

nearestIndex = @(x0) find(abs(x - x0) == min(abs(x - x0)), 1, 'first');

idx_CGrest = nearestIndex(xCG_rest);
idx_wing   = nearestIndex(xw);
idx_tail   = nearestIndex(xt);

%% ==========================================================
%  MASS BREAKDOWN
%% ==========================================================
m_rest = m_OEM + m_fuel + m_crew;   % all mass except payload [kg]

if verbose
    fprintf('Mass check:\n');
    fprintf('MTOW           = %.0f kg\n', m_MTOW);
    fprintf('Payload        = %.0f kg\n', m_payload);
    fprintf('Remaining mass = %.0f kg\n', m_rest);
    fprintf('Payload + rest = %.0f kg\n\n', m_payload + m_rest);
end

%% ==========================================================
%  WEIGHTS (1g REFERENCE)
%% ==========================================================
W_payload = m_payload * g;    % [N]
W_rest    = m_rest    * g;    % [N]
W_total   = m_MTOW    * g;    % [N]

% Distributed payload load
q_payload = W_payload / (xPayload2 - xPayload1);   % [N/m]

%% ==========================================================
%  GUST LOAD FACTORS
%% ==========================================================
WS = W_total / Sw;   % wing loading [N/m^2]

dn_up = rho * V * a * Kg * wg_up / (2 * WS);
dn_dn = rho * V * a * Kg * wg_dn / (2 * WS);

n_gust_up = 1.0 + dn_up;
n_gust_dn = 1.0 + dn_dn;

%% ==========================================================
%  PRESSURIZATION / SECTION INPUTS
%% ==========================================================
% Skin thickness from hoop stress
t_skin_pressure = p_diff * r_fus / sigma_allow;   % [m]

% Practical minimum gauge
t_skin = max(t_skin_pressure, t_min_gauge);

% Thin-walled circular tube properties using skin only
I_fus = pi * r_fus^3 * t_skin;   % [m^4]
Z_fus = I_fus / r_fus;           % [m^3]

%% ==========================================================
%  RUN SYMMETRIC LOAD CASES
%% ==========================================================
runCase = @(n) computeVM_ModelA( ...
    x, dx, n, ...
    q_payload, xPayload1, xPayload2, ...
    W_rest, idx_CGrest, xCG_rest, ...
    xw, idx_wing, ...
    xt, idx_tail);

case1g     = runCase(1.0);
case25g    = runCase(2.5);
caseM1g    = runCase(-1.0);
caseGustUp = runCase(n_gust_up);
caseGustDn = runCase(n_gust_dn);

%% ==========================================================
%  ASYMMETRIC WING LOAD CASE: 100:80 of 2.5g CASE
%% ==========================================================
asym = computeAsymmetricTorqueCase( ...
    x, ...
    idx_wing, idx_tail, ...
    case25g.Lw, ...
    Span, ...
    yResFracSemi, ...
    1.0, ...
    0.8);

%% ==========================================================
%  PRESSURIZATION STRESSES
%% ==========================================================
sigma_hoop       = p_diff * r_fus / t_skin;        % [Pa]
sigma_long_press = p_diff * r_fus / (2 * t_skin);  % [Pa]

%% ==========================================================
%  BENDING STRESS DISTRIBUTIONS
%% ==========================================================
sigma_b_1g     = case1g.My     / Z_fus;   % [Pa]
sigma_b_25g    = case25g.My    / Z_fus;   % [Pa]
sigma_b_M1g    = caseM1g.My    / Z_fus;   % [Pa]
sigma_b_gustUp = caseGustUp.My / Z_fus;   % [Pa]
sigma_b_gustDn = caseGustDn.My / Z_fus;   % [Pa]

%% ==========================================================
%  COMBINED LONGITUDINAL STRESS
%% ==========================================================
sigma_top_1g     = sigma_long_press + sigma_b_1g;
sigma_bot_1g     = sigma_long_press - sigma_b_1g;

sigma_top_25g    = sigma_long_press + sigma_b_25g;
sigma_bot_25g    = sigma_long_press - sigma_b_25g;

sigma_top_M1g    = sigma_long_press + sigma_b_M1g;
sigma_bot_M1g    = sigma_long_press - sigma_b_M1g;

sigma_top_gustUp = sigma_long_press + sigma_b_gustUp;
sigma_bot_gustUp = sigma_long_press - sigma_b_gustUp;

sigma_top_gustDn = sigma_long_press + sigma_b_gustDn;
sigma_bot_gustDn = sigma_long_press - sigma_b_gustDn;

%% ==========================================================
%  MAX MOMENTS / MAX BENDING STRESSES
%% ==========================================================
Mmax_1g     = max(abs(case1g.My));
Mmax_25g    = max(abs(case25g.My));
Mmax_M1g    = max(abs(caseM1g.My));
Mmax_gustUp = max(abs(caseGustUp.My));
Mmax_gustDn = max(abs(caseGustDn.My));

sigma_bmax_1g     = max(abs(sigma_b_1g));
sigma_bmax_25g    = max(abs(sigma_b_25g));
sigma_bmax_M1g    = max(abs(sigma_b_M1g));
sigma_bmax_gustUp = max(abs(sigma_b_gustUp));
sigma_bmax_gustDn = max(abs(sigma_b_gustDn));

%% ==========================================================
%  STRINGER SIZING FROM BENDING
%% ==========================================================
Mcrit = max([Mmax_1g, Mmax_25g, Mmax_M1g, Mmax_gustUp, Mmax_gustDn]);

t_eq = Mcrit / (pi * r_fus^2 * sigma_allow);      % [m]
A_req = 2 * pi * r_fus * t_eq;                    % [m^2]
A_skin = 2 * pi * r_fus * t_skin;                 % [m^2]

A_stringers_total = max(A_req - A_skin, 0.0);     % [m^2]
A_per_stringer = A_stringers_total / N_stringers; % [m^2]
F_stringer = sigma_allow * A_per_stringer;        % [N]

%% ==========================================================
%  FRAME SPACING AND MASS ESTIMATION
%% ==========================================================
N_frames = round(Lf / frame_spacing);

% Stringer pitch
stringer_pitch = 2*pi*r_fus / N_stringers;

% Frame geometry assumption
A_frame = 0.0015;         % frame cross-sectional area [m^2]

frame_length = 2*pi*r_fus;
V_frame = frame_length * A_frame;
V_frames_total = N_frames * V_frame;

m_frames = rho_al * V_frames_total;

%% ==========================================================
%  MASS ESTIMATION
%% ==========================================================
% Skin mass
A_skin_surface = 2*pi*r_fus*Lf;
V_skin = A_skin_surface * t_skin;
m_skin = rho_al * V_skin;

% Stringer mass
V_stringers = A_stringers_total * Lf;
m_stringers = rho_al * V_stringers;

% Total fuselage structural mass
m_fuselage_total = m_skin + m_stringers + m_frames;

%% ==========================================================
%  EI AND GJ ESTIMATES
%% ==========================================================
I_eq = pi * r_fus^3 * t_eq;
J_skin = 2 * pi * r_fus^3 * t_skin;

EI_const = E * I_eq;
GJ_const = G * J_skin;

%% ==========================================================
%  PLOTS
%% ==========================================================
if makePlots
    figure;
    plot(x, case1g.Vz,      'LineWidth', 1.5); hold on;
    plot(x, case25g.Vz,     'LineWidth', 1.5);
    plot(x, caseM1g.Vz,     'LineWidth', 1.5);
    plot(x, caseGustUp.Vz,  'LineWidth', 1.5);
    plot(x, caseGustDn.Vz,  'LineWidth', 1.5);
    grid on;
    xlabel('x [m]');
    ylabel('V_z [N]');
    title('Fuselage Shear Force Distribution');
    legend('1g', '2.5g', '-1g', 'Gust Up', 'Gust Down', 'Location', 'best');
    set(gcf, 'Color', 'w');

    figure;
    plot(x, case1g.My,      'LineWidth', 1.5); hold on;
    plot(x, case25g.My,     'LineWidth', 1.5);
    plot(x, caseM1g.My,     'LineWidth', 1.5);
    plot(x, caseGustUp.My,  'LineWidth', 1.5);
    plot(x, caseGustDn.My,  'LineWidth', 1.5);
    grid on;
    xlabel('x [m]');
    ylabel('M_y [N m]');
    title('Fuselage Bending Moment Distribution');
    legend('1g', '2.5g', '-1g', 'Gust Up', 'Gust Down', 'Location', 'best');
    set(gcf, 'Color', 'w');

    figure;
    stem(x, asym.T_applied, 'filled', 'LineWidth', 1.4);
    grid on;
    xlabel('x [m]');
    ylabel('Applied Torque [N m]');
    title('Applied Torque Stations - Asymmetric Wing Load Case');
    set(gcf, 'Color', 'w');

    figure;
    plot(x, asym.T_internal, 'LineWidth', 1.8);
    grid on;
    xlabel('x [m]');
    ylabel('T_x [N m]');
    title('Internal Torque Distribution - Asymmetric Wing Load Case');
    set(gcf, 'Color', 'w');

    figure;
    plot(x, sigma_top_25g/1e6, 'LineWidth', 1.8); hold on;
    plot(x, sigma_bot_25g/1e6, 'LineWidth', 1.8);
    yline(sigma_long_press/1e6, '--', 'LineWidth', 1.3);
    grid on;
    xlabel('x [m]');
    ylabel('Stress [MPa]');
    title('Combined Longitudinal Stress Distribution (2.5g Case)');
    legend('Top skin', 'Bottom skin', 'Pressure longitudinal stress', 'Location', 'best');
    set(gcf, 'Color', 'w');
end

%% ==========================================================
%  PRINT RESULTS
%% ==========================================================
if verbose
    fprintf('\n================ TRIM RESULTS ================\n');
    fprintf('Assumed non-payload CG location = %.2f m\n', xCG_rest);
    fprintf('Payload region                  = %.2f m to %.2f m\n\n', xPayload1, xPayload2);

    fprintf('1g case:\n');
    fprintf('  Total weight  = %.3f MN\n', case1g.Wtot / 1e6);
    fprintf('  Aircraft CG   = %.3f m\n', case1g.xcg);
    fprintf('  Tail load Lt  = %.3f MN\n', case1g.Lt / 1e6);
    fprintf('  Wing lift Lw  = %.3f MN\n\n', case1g.Lw / 1e6);

    fprintf('2.5g case:\n');
    fprintf('  Total weight  = %.3f MN\n', case25g.Wtot / 1e6);
    fprintf('  Aircraft CG   = %.3f m\n', case25g.xcg);
    fprintf('  Tail load Lt  = %.3f MN\n', case25g.Lt / 1e6);
    fprintf('  Wing lift Lw  = %.3f MN\n\n', case25g.Lw / 1e6);

    fprintf('-1g case:\n');
    fprintf('  Total weight  = %.3f MN\n', caseM1g.Wtot / 1e6);
    fprintf('  Aircraft CG   = %.3f m\n', caseM1g.xcg);
    fprintf('  Tail load Lt  = %.3f MN\n', caseM1g.Lt / 1e6);
    fprintf('  Wing lift Lw  = %.3f MN\n\n', caseM1g.Lw / 1e6);

    fprintf('Upward gust case:\n');
    fprintf('  Load factor n = %.3f\n', n_gust_up);
    fprintf('  Total weight  = %.3f MN\n', caseGustUp.Wtot / 1e6);
    fprintf('  Aircraft CG   = %.3f m\n', caseGustUp.xcg);
    fprintf('  Tail load Lt  = %.3f MN\n', caseGustUp.Lt / 1e6);
    fprintf('  Wing lift Lw  = %.3f MN\n\n', caseGustUp.Lw / 1e6);

    fprintf('Downward gust case:\n');
    fprintf('  Load factor n = %.3f\n', n_gust_dn);
    fprintf('  Total weight  = %.3f MN\n', caseGustDn.Wtot / 1e6);
    fprintf('  Aircraft CG   = %.3f m\n', caseGustDn.xcg);
    fprintf('  Tail load Lt  = %.3f MN\n', caseGustDn.Lt / 1e6);
    fprintf('  Wing lift Lw  = %.3f MN\n\n', caseGustDn.Lw / 1e6);

    fprintf('============= ASYMMETRIC 100:80 CASE =============\n');
    fprintf('Reference case                    = symmetric 2.5g\n');
    fprintf('Total wing lift from 2.5g case    = %.3f MN\n', case25g.Lw / 1e6);
    fprintf('Symmetric half-wing lift          = %.3f MN\n', asym.L_half / 1e6);
    fprintf('Left wing lift                    = %.3f MN\n', asym.L_left / 1e6);
    fprintf('Right wing lift                   = %.3f MN\n', asym.L_right / 1e6);
    fprintf('Lift difference (L_left-L_right)  = %.3f MN\n', asym.deltaL / 1e6);
    fprintf('Half-wing resultant arm           = %.3f m\n', asym.y_res);
    fprintf('Applied fuselage torque           = %.3f MNm\n', asym.T_wing / 1e6);
    fprintf('Reaction torque station           = %.2f m\n', xt);
    fprintf('Max internal torque magnitude     = %.3f MNm\n', max(abs(asym.T_internal))/1e6);

    fprintf('\n================ PRESSURIZATION / SECTION RESULTS ================\n');
    fprintf('Pressure differential p            = %.3f kPa\n', p_diff / 1e3);
    fprintf('Fuselage radius r                  = %.3f m\n', r_fus);
    fprintf('Allowable stress                   = %.3f MPa\n', sigma_allow / 1e6);
    fprintf('Thickness from pressure only       = %.3f mm\n', t_skin_pressure * 1e3);
    fprintf('Adopted skin thickness             = %.3f mm\n\n', t_skin * 1e3);

    fprintf('Pressurization stresses:\n');
    fprintf('  Hoop stress                      = %.3f MPa\n', sigma_hoop / 1e6);
    fprintf('  Longitudinal pressure stress     = %.3f MPa\n\n', sigma_long_press / 1e6);

    fprintf('Maximum absolute bending moment:\n');
    fprintf('  1g                               = %.3f MNm\n', Mmax_1g / 1e6);
    fprintf('  2.5g                             = %.3f MNm\n', Mmax_25g / 1e6);
    fprintf('  -1g                              = %.3f MNm\n', Mmax_M1g / 1e6);
    fprintf('  Gust Up                          = %.3f MNm\n', Mmax_gustUp / 1e6);
    fprintf('  Gust Down                        = %.3f MNm\n\n', Mmax_gustDn / 1e6);

    fprintf('Maximum absolute bending stress (skin-only section modulus):\n');
    fprintf('  1g                               = %.3f MPa\n', sigma_bmax_1g / 1e6);
    fprintf('  2.5g                             = %.3f MPa\n', sigma_bmax_25g / 1e6);
    fprintf('  -1g                              = %.3f MPa\n', sigma_bmax_M1g / 1e6);
    fprintf('  Gust Up                          = %.3f MPa\n', sigma_bmax_gustUp / 1e6);
    fprintf('  Gust Down                        = %.3f MPa\n\n', sigma_bmax_gustDn / 1e6);

    fprintf('Combined longitudinal stress extrema (2.5g case):\n');
    fprintf('  Top skin min/max                 = %.3f / %.3f MPa\n', min(sigma_top_25g)/1e6, max(sigma_top_25g)/1e6);
    fprintf('  Bottom skin min/max              = %.3f / %.3f MPa\n\n', min(sigma_bot_25g)/1e6, max(sigma_bot_25g)/1e6);

    fprintf('================ STRINGER SIZING RESULTS ================\n');
    fprintf('Critical bending moment            = %.3f MNm\n', Mcrit / 1e6);
    fprintf('Equivalent thickness required      = %.3f mm\n', t_eq * 1e3);
    fprintf('Skin longitudinal area             = %.6f m^2\n', A_skin);
    fprintf('Total required longitudinal area   = %.6f m^2\n', A_req);
    fprintf('Total stringer area required       = %.6f m^2\n', A_stringers_total);
    fprintf('Number of stringers assumed        = %d\n', N_stringers);
    fprintf('Area per stringer                  = %.6f m^2\n', A_per_stringer);
    fprintf('Estimated load per stringer        = %.3f kN\n\n', F_stringer / 1e3);

    fprintf('================ FRAME STRUCTURE RESULTS ================\n');
    fprintf('Frame spacing                      = %.3f m\n', frame_spacing);
    fprintf('Number of frames                   = %d\n', N_frames);
    fprintf('Stringer pitch                     = %.3f m\n', stringer_pitch);
    fprintf('Frame cross-sectional area         = %.6f m^2\n', A_frame);
    fprintf('Total frame volume                 = %.3f m^3\n', V_frames_total);
    fprintf('Estimated frame mass               = %.1f kg\n\n', m_frames);

    fprintf('================ MASS ESTIMATION ================\n');
    fprintf('Skin mass                          = %.1f kg\n', m_skin);
    fprintf('Stringer mass                      = %.1f kg\n', m_stringers);
    fprintf('Frame mass                         = %.1f kg\n', m_frames);
    fprintf('-----------------------------------------------\n');
    fprintf('Total fuselage structural mass     = %.1f kg\n\n', m_fuselage_total);

    fprintf('================ STIFFNESS RESULTS ================\n');
    fprintf('EI (constant estimate)             = %.3e N m^2\n', EI_const);
    fprintf('GJ (constant estimate)             = %.3e N m^2\n', GJ_const);
end

%% ==========================================================
%  OUTPUT STRUCT
%% ==========================================================
fus.case1g     = case1g;
fus.case25g    = case25g;
fus.caseM1g    = caseM1g;
fus.caseGustUp = caseGustUp;
fus.caseGustDn = caseGustDn;
fus.asym       = asym;

fus.t_skin_pressure = t_skin_pressure;
fus.t_skin          = t_skin;
fus.t_eq            = t_eq;

fus.sigma_hoop       = sigma_hoop;
fus.sigma_long_press = sigma_long_press;

fus.sigma_top_25g = sigma_top_25g;
fus.sigma_bot_25g = sigma_bot_25g;

fus.Mcrit = Mcrit;
fus.A_stringers_total = A_stringers_total;
fus.A_per_stringer    = A_per_stringer;
fus.N_stringers       = N_stringers;
fus.F_stringer        = F_stringer;

fus.frame_spacing = frame_spacing;
fus.N_frames      = N_frames;
fus.stringer_pitch = stringer_pitch;

fus.m_skin           = m_skin;
fus.m_stringers      = m_stringers;
fus.m_frames         = m_frames;
fus.m_fuselage_total = m_fuselage_total;

fus.EI = EI_const;
fus.GJ = GJ_const;

end

%% ==========================================================
%  LOCAL FUNCTIONS
%% ==========================================================
function out = computeVM_ModelA(x, dx, n, ...
    q_payload, xP1, xP2, ...
    W_rest, idx_CGrest, xCG_rest, ...
    xw, idx_wing, ...
    xt, idx_tail)

N = numel(x);

qz = zeros(1, N);

% Payload distributed load
payloadMask = (x >= xP1) & (x <= xP2);
qz(payloadMask) = n * q_payload;

% Point loads
Pz = zeros(1, N);

% Lumped remaining weight at assumed CG location
Pz(idx_CGrest) = Pz(idx_CGrest) + n * W_rest;

% Total weight
W_payload_total = trapz(x, qz);
Wtot = W_payload_total + sum(Pz);

% Aircraft CG
x_payload_cg = 0.5 * (xP1 + xP2);

moment_payload = W_payload_total * x_payload_cg;
moment_rest    = (n * W_rest) * xCG_rest;

xcg = (moment_payload + moment_rest) / Wtot;

% Trim equations
A = xw - xcg;
B = xt - xcg;

Lt = (Wtot * A) / (B - A);
Lw = Wtot + Lt;

% Apply wing / tail forces
Pz(idx_tail) = Pz(idx_tail) + Lt;   % tail load downward positive
Pz(idx_wing) = Pz(idx_wing) - Lw;   % wing lift upward negative

% Shear force and bending moment
Vz = zeros(1, N);
My = zeros(1, N);

for j = 2:N
    Vz(j) = Vz(j-1) - qz(j-1)*dx - Pz(j-1);
    My(j) = My(j-1) + Vz(j-1)*dx;
end

out.Vz   = Vz;
out.My   = My;
out.Lt   = Lt;
out.Lw   = Lw;
out.Wtot = Wtot;
out.xcg  = xcg;
end

function out = computeAsymmetricTorqueCase(x, idx_wing, idx_react, ...
    Lw_total_ref, Span, yResFracSemi, leftFactor, rightFactor)

N = numel(x);

L_half  = Lw_total_ref / 2.0;

L_left  = leftFactor  * L_half;
L_right = rightFactor * L_half;

deltaL = L_left - L_right;

% Resultant arm of half-wing lift
y_res = yResFracSemi * (Span / 2.0);

% Applied torque at wing station
T_wing = deltaL * y_res;

% Apply balanced torques to fuselage
T_applied = zeros(1, N);
T_applied(idx_wing)  = T_applied(idx_wing)  + T_wing;
T_applied(idx_react) = T_applied(idx_react) - T_wing;

% Internal torque diagram
T_internal = zeros(1, N);
for j = 2:N
    T_internal(j) = T_internal(j-1) - T_applied(j-1);
end

out.L_half     = L_half;
out.L_left     = L_left;
out.L_right    = L_right;
out.deltaL     = deltaL;
out.y_res      = y_res;
out.T_wing     = T_wing;
out.T_applied  = T_applied;
out.T_internal = T_internal;
end