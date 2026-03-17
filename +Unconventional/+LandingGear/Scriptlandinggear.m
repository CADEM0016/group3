%% =========================================================
%  LANDING GEAR CLASS I / II / II.5 SIZING TOOL
%  MSc Group Design Project – CADEM0016, University of Bristol
%  Reference: Handbook, Specification, Landing Gear Lecture Slides
%  Author: Landing Gear Engineer
%  Date:   March 2026
%% =========================================================
clc; clear; close all;

fprintf('=======================================================\n');
fprintf(' LANDING GEAR CLASS I / II / II.5 SIZING TOOL\n');
fprintf(' CADEM0016 – MSc Group Design Project\n');
fprintf('=======================================================\n\n');

%% =========================================================
%  SECTION 1: AIRCRAFT TOP-LEVEL INPUTS
%  (Edit these to match your MDO hyperparameters)
%% =========================================================
MTOM   = 497870;     % [kg]  Maximum Take-Off Mass
MLM_ratio = 0.90;    % [-]   MLM/MTOM ratio (Class I: 0.85–0.95 for widebody)
OEM    = 145000;     % [kg]  Operational Empty Mass (initial estimate)
n_legs = 4;          % [-]   Number of main landing gear legs (typical widebody: 2 or 4)

% Geometry (initial estimates – update from fuselage/stability disciplines)
h_cg    = 3.15;      % [m]   CG height above ground (compressed gear)
l_fus   = 76.0;      % [m]   Total fuselage length
x_nose  = 0.05;      % [-]   NLG position as fraction of fuselage length from nose
x_mg_frac = 0.52;    % [-]   MLG longitudinal position as fraction of fuselage length
x_cg_frac = 0.45;    % [-]   CG position as fraction of fuselage length

% Engine / fuselage geometry
D_fan   = 3.20;      % [m]   Engine fan diameter
h_fus_bottom = 3.15; % [m]   Height of fuselage bottom above ground (compressed gear)
R_fus   = 3.15;      % [m]   Fuselage outer radius at main gear attachment

% Tail geometry (for tail-strike)
x_tail_contact = 0.98;  % [-]  Fraction of fuselage length where tail contacts ground
h_tail_contact = 1.50;  % [m]  Height of tail contact point above gear pivot (add gear height)

% ICAO code target
icao_code = 'F';     % landing: up to F (65–80 m span); taxi: up to E (52–65 m)

% Material for physics-based estimate
sigma_y = 1000e6;    % [Pa]  Yield strength (high-strength steel, e.g. 300M)
E_steel = 200e9;     % [Pa]  Young's modulus
rho_steel = 7850;    % [kg/m³] Steel density

g = 9.81;            % [m/s²]

%% =========================================================
%  SECTION 2: CLASS I – STATISTICAL / EMPIRICAL ESTIMATES
%  (MDO Development – initial sizing)
%% =========================================================
fprintf('--- SECTION 2: CLASS I ESTIMATES ---\n\n');

% 2.1 Maximum Landing Mass
MLM = MLM_ratio * MTOM;
fprintf('Class I: MLM = %.0f kg  (MLM/MTOM = %.2f)\n', MLM, MLM_ratio);

% 2.2 Total Landing Gear Mass (Raymer Table 15.2: ~3.3–5.7% MTOW for transports)
LG_mass_frac = 0.04;     % [-]  ~4% of MTOM
M_LG_total   = LG_mass_frac * MTOM;
M_LG_main    = 0.90 * M_LG_total;   % 90% on main gear
M_LG_nose    = 0.10 * M_LG_total;   % 10% on nose gear
fprintf('Class I: Total LG mass  = %.0f kg (%.1f%% MTOM)\n', M_LG_total, LG_mass_frac*100);
fprintf('         Main LG mass   = %.0f kg\n', M_LG_main);
fprintf('         Nose LG mass   = %.0f kg\n\n', M_LG_nose);

% 2.3 Number of Wheels (Class I – empirical comparison)
% Rule of thumb: ~20–30 t per main-gear tyre for large civil aircraft
MTOW_kg_per_tyre_target = 25000; % [kg] ~25t per tyre (A380: ~28.75t)
n_MLG_wheels_CI = ceil(0.90 * MTOM / MTOW_kg_per_tyre_target);
% Enforce 2 wheels per axle (civil certification requirement)
if mod(n_MLG_wheels_CI, 2) ~= 0
    n_MLG_wheels_CI = n_MLG_wheels_CI + 1;
end
n_NLG_wheels = 2;  % Standard dual-wheel nose gear
fprintf('Class I: Recommended main gear wheels = %d (+ %d nose)\n', ...
        n_MLG_wheels_CI, n_NLG_wheels);
fprintf('         Mass per MLG tyre = %.1f t\n\n', 0.9*MTOM/n_MLG_wheels_CI/1000);

%% =========================================================
%  SECTION 3: CLASS II – SEMI-EMPIRICAL ESTIMATES
%  (MDO Development – refined, physics-informed)
%% =========================================================
fprintf('--- SECTION 3: CLASS II ESTIMATES ---\n\n');

% 3.1 Tyre Sizing (Raymer Table 11.1 – power-law empirical fit)
%     D [in] = A * (W_wheel [lbs])^B  for Type VII (transport tyres)
%     A = 0.89, B = 0.361  (Raymer)
W_wheel_lbs = (0.90 * MTOM * 2.2046) / n_MLG_wheels_CI;  % [lbs] load per tyre
A_tyre = 0.89; B_tyre = 0.361;
D_tyre_in = A_tyre * W_wheel_lbs^B_tyre;   % [in]
W_tyre_in = 0.35 * D_tyre_in;              % [in] approximate width (Raymer ~0.35*D)
D_tyre_m  = D_tyre_in * 0.0254;            % [m]
W_tyre_m  = W_tyre_in * 0.0254;            % [m]
R_tyre    = D_tyre_m / 2;                  % [m]
fprintf('Class II: Tyre diameter = %.2f m (%.1f in)\n', D_tyre_m, D_tyre_in);
fprintf('          Tyre width    = %.2f m (%.1f in)\n', W_tyre_m, W_tyre_in);
fprintf('          Tyre radius   = %.3f m\n\n', R_tyre);

% 3.2 Tyre Deflection (slide p.15: narrowbody and above = 6 inches)
x_t = 6 * 0.0254;   % [m]  Tyre deflection under load = 0.152 m

% 3.3 Load Distribution & Gear Placement
%     MAIN: 85–95% of static load; NOSE: 5–15%
main_load_share = 0.90;
nose_load_share = 0.10;
F_static_main = main_load_share * MTOM * g;   % [N] total static force on all MLG
F_static_nose = nose_load_share * MTOM * g;   % [N]
F_per_leg     = F_static_main / n_legs;        % [N] per MLG leg
fprintf('Class II: Static main gear total load = %.0f kN\n', F_static_main/1e3);
fprintf('          Static nose gear total load  = %.0f kN\n', F_static_nose/1e3);
fprintf('          Load per MLG leg             = %.0f kN\n\n', F_per_leg/1e3);

% 3.4 Longitudinal Placement
x_mg   = x_mg_frac * l_fus;   % [m] MLG position from nose
x_nose_pos = x_nose * l_fus;  % [m] NLG position from nose
x_cg   = x_cg_frac * l_fus;   % [m] CG position from nose

% Nose gear load check (moment balance about main gear)
% F_nose * (x_mg - x_nose_pos) = MTOM*g*(x_mg - x_cg)
dist_mg_cg   = x_mg - x_cg;           % [m] positive = CG fwd of MLG
dist_mg_nlg  = x_mg - x_nose_pos;     % [m] wheelbase

if dist_mg_nlg <= 0
    warning('NLG must be FORWARD of MLG. Check x_nose_pos and x_mg_frac.');
end

F_nose_check = MTOM * g * dist_mg_cg / dist_mg_nlg;   % [N]
nose_share_check = F_nose_check / (MTOM * g) * 100;    % [%]

fprintf('Class II: Longitudinal placement:\n');
fprintf('          NLG position  = %.2f m from nose\n', x_nose_pos);
fprintf('          MLG position  = %.2f m from nose\n', x_mg);
fprintf('          CG position   = %.2f m from nose\n', x_cg);
fprintf('          Wheelbase     = %.2f m\n', dist_mg_nlg);
fprintf('          NLG load (static) = %.0f kN  (%.1f%% of MTOM)\n\n', ...
        F_nose_check/1e3, nose_share_check);

% 3.5 Tail-Strike Angle (slide p.7 & p.8: < 15°, aim 8–12°)
%     θ_ts = arctan(h_tail / dist_mg_to_tail)
x_tail   = x_tail_contact * l_fus;            % [m] tail contact point from nose
dist_mg_tail = x_tail - x_mg;                 % [m]
h_gear_approx = R_fus + R_tyre;               % [m] rough gear length
h_tail_above_ground = h_tail_contact + h_gear_approx;  % [m]
theta_ts = atand(h_tail_above_ground / dist_mg_tail);  % [deg]

fprintf('Class II: Tail-strike angle = %.1f deg  (target: 8–15 deg)\n', theta_ts);
if theta_ts < 8
    fprintf('          WARNING: tail-strike risk is HIGH (< 8 deg)\n');
elseif theta_ts > 15
    fprintf('          WARNING: rotation may be limited (> 15 deg)\n');
else
    fprintf('          OK: within safe range\n');
end

% 3.6 Tip-Back Angle (slide p.7 & p.8: β > θ_ts, β < 25°)
%     β = arctan((x_mg - x_cg_AFT) / h_cg)
%     Use worst-case AFT CG: assume 5% fwd shift from aft limit
x_cg_aft = (x_cg_frac + 0.03) * l_fus;    % [m] AFT CG limit
beta_tipback = atand((x_mg - x_cg_aft) / h_cg);   % [deg]

fprintf('          Tip-back angle β = %.1f deg  (must be > %.1f and < 25 deg)\n', ...
        beta_tipback, theta_ts);
if beta_tipback < theta_ts
    fprintf('          WARNING: Tip-back angle less than tail-strike angle!\n');
elseif beta_tipback > 25
    fprintf('          WARNING: Large elevator input required for rotation.\n');
else
    fprintf('          OK: tip-back angle is acceptable\n');
end
fprintf('\n');

% 3.7 Lateral Placement – Turnover Angle (slide p.9: θ < 63°)
%     α = arctan(T/(2B)),  Y = D*sin(α),  θ = arctan(E/Y)
%     T = track width, B = wheelbase, D = NLG-to-MLG horizontal dist, E = CG height

% ICAO F: track 14–16 m (taxi: E means track 9–14 m)
% Set track to satisfy BOTH: turnover < 63° AND ICAO taxi E constraint
% Iterative loop to find minimum track width satisfying turnover requirement
B_wb = dist_mg_nlg;           % [m] wheelbase
D_ng_mg = dist_mg_cg;         % [m] CG to MLG distance (longitudinal)
E_cg = h_cg;                  % [m] CG height

fprintf('Class II: Lateral placement (turnover constraint):\n');
T_min_icao = 9.0;    % [m] ICAO E minimum
T_max_icao = 14.0;   % [m] ICAO E maximum (taxi)
theta_turnover = 100;
T_track = T_min_icao;
while theta_turnover > 63 && T_track <= T_max_icao + 2
    alpha_t  = atand((T_track/2) / B_wb);
    Y_t      = D_ng_mg * sind(alpha_t);
    theta_turnover = atand(E_cg / Y_t);
    T_track  = T_track + 0.1;
end
T_track = T_track - 0.1;  % back one step

fprintf('          Required track width T = %.2f m  (turnover angle = %.1f deg)\n', ...
        T_track, theta_turnover);
fprintf('          ICAO E max track = %.1f m  |  ICAO F max track = 16.0 m\n', T_max_icao);
if T_track > 16.0
    fprintf('          WARNING: Track exceeds ICAO F limit! Redesign needed.\n');
elseif T_track > 14.0
    fprintf('          INFO: ICAO F (taxi) required (track > 14 m)\n');
else
    fprintf('          OK: within ICAO E taxi constraint\n');
end
fprintf('\n');

% 3.8 Required Landing Gear Length – Engine Clearance (slide p.10)
%     Clearance = 0.23 * D_fan between nacelle bottom and ground
h_engine_clearance = 0.23 * D_fan;   % [m]
% Engine mounted below wing, approximate engine centreline height above ground
% h_nacelle_bottom = h_gear - (engine_hang_below_wing)
% Assuming engine bottom = gear height - (fuselage_bottom + engine_offset)
% Simplified: gear length sets ground clearance
h_gear_engine = h_fus_bottom + h_engine_clearance;  % [m] minimum gear length for clearance
fprintf('Class II: Gear length from engine clearance = %.3f m\n', h_gear_engine);
fprintf('          (engine clearance required = %.3f m  = 0.23 x %.2f m fan)\n\n', ...
        h_engine_clearance, D_fan);

%% =========================================================
%  SECTION 4: CLASS II.5 – PHYSICS-BASED (MDO Refinement)
%% =========================================================
fprintf('--- SECTION 4: CLASS II.5 PHYSICS-BASED ESTIMATES ---\n\n');

% ---- Sweep over λ (landing reaction factor) ----
lambda_vec = 1.1 : 0.05 : 1.8;   % typical range (large civil: 1.1–1.3)

% Certification descent rate: 10 ft/s
V_z = 10 * 0.3048;    % [m/s] = 3.048 m/s

fprintf('4.1  Shock Absorber Stroke Length  (Class II.5)\n');
fprintf('     Energy equation: 0.5*m*Vz^2 = lambda*m*g*(0.75*xs + 0.5*xt)\n');
fprintf('     Vz = %.3f m/s  |  xt = %.4f m  |  MLM = %.0f kg\n\n', V_z, x_t, MLM);
fprintf('     lambda |  xs [m]  | xs+xt [m] | F_peak [kN/leg] | M_leg_tube [kg]\n');
fprintf('     -------|----------|-----------|-----------------|----------------\n');

xs_vec   = zeros(size(lambda_vec));
Fpeak_vec = zeros(size(lambda_vec));
M_tube_vec = zeros(size(lambda_vec));

for ii = 1:length(lambda_vec)
    lam = lambda_vec(ii);
    
    % Stroke from energy balance (slide p.15):
    % 0.5*V_z^2 = lambda*g*(0.75*xs + 0.5*xt)
    % → xs = (V_z^2/(2*lambda*g) - 0.5*xt) / 0.75
    xs = (V_z^2 / (2*lam*g) - 0.5*x_t) / 0.75;
    if xs < 0.05
        xs = 0.05;  % physical minimum
    end
    xs_vec(ii) = xs;
    
    % Peak landing force per leg (lambda * static weight per leg)
    F_peak_leg = lam * (MLM * g / n_legs);   % [N]
    Fpeak_vec(ii) = F_peak_leg;
    
    %% Physics-based tube mass (main fitting as hollow cylinder)
    % Main fitting length = xs + 10% stroke (slide p.18)
    L_tube = xs * 1.10;
    
    % Slider diameter from static gas pressure (slide p.18)
    % P_static = 1500 psi = 10.34 MPa
    P_static = 1500 * 6894.76;   % [Pa]
    F_static_leg = (MLM * g * main_load_share) / n_legs;   % [N]
    A_slider = F_static_leg / P_static;                    % [m²]
    d_slider = 2 * sqrt(A_slider / pi);                    % [m]
    
    % Wall thickness: size main fitting (outer cylinder) for Euler buckling
    % Pcr = pi^2 * E * I / (K*L)^2  with K=2 (cantilever), I=pi/64*(Do^4-Di^4)
    % Use t = outer_wall so that Pcr > F_peak_leg with safety factor 2.0
    SF = 2.0;
    D_outer = d_slider * 1.6;   % outer diameter ~1.6x slider (annular gap for oil)
    K_col   = 2.0;              % effective length factor (fixed-free)
    % Pcr = SF * F_peak:  pi^2*E*I/(K*L)^2 = SF*F
    % I_required = SF * F * (K*L)^2 / (pi^2 * E)
    I_req   = SF * F_peak_leg * (K_col * L_tube)^2 / (pi^2 * E_steel);
    % For hollow cylinder: I = pi/64*(Do^4 - Di^4)  → Di from I_req
    Di4 = D_outer^4 - 64*I_req/pi;
    if Di4 < 0
        Di = 0;
    else
        Di = Di4^(1/4);
    end
    t_wall  = (D_outer - Di) / 2;
    if t_wall < 0.002, t_wall = 0.002; end   % minimum 2 mm
    
    % Also check compressive stress
    A_tube   = pi/4 * (D_outer^2 - (D_outer - 2*t_wall)^2);
    sigma_c  = F_peak_leg / A_tube;
    if sigma_c > sigma_y / SF
        % Increase t_wall
        A_req_comp = SF * F_peak_leg / sigma_y;
        t_wall_comp = (D_outer - sqrt(D_outer^2 - 4*A_req_comp/pi)) / 2;
        if t_wall_comp > t_wall, t_wall = t_wall_comp; end
    end
    
    % Volume and mass of main fitting (one leg)
    V_tube   = pi/4 * (D_outer^2 - (D_outer - 2*t_wall)^2) * L_tube;
    M_tube_one_leg = rho_steel * V_tube;   % [kg]
    % Include slider piston mass (~30% of fitting) + fittings/brackets (~20%)
    M_leg_total = M_tube_one_leg * (1 + 0.30 + 0.20);
    M_tube_vec(ii) = M_leg_total;
    
    fprintf('     %5.2f  |  %7.4f |  %8.4f |  %14.1f  |  %11.1f\n', ...
        lam, xs, xs+x_t, F_peak_leg/1e3, M_leg_total);
end
fprintf('\n');

% Select design point λ (large civil: 1.1–1.3)
lambda_design = 1.2;
idx_d = find(abs(lambda_vec - lambda_design) < 0.01, 1);
xs_design     = xs_vec(idx_d);
Fpeak_design  = Fpeak_vec(idx_d);
M_leg_design  = M_tube_vec(idx_d);

fprintf('  >> Design choice: lambda = %.2f\n', lambda_design);
fprintf('     Stroke length xs        = %.4f m (%.2f in)\n', xs_design, xs_design/0.0254);
fprintf('     Peak force per MLG leg  = %.1f kN\n', Fpeak_design/1e3);

% 4.2 Slider diameter at design point
P_static = 1500 * 6894.76;
F_static_leg = (MLM * g * main_load_share) / n_legs;
A_slider_d = F_static_leg / P_static;
d_slider_d  = 2 * sqrt(A_slider_d / pi);
fprintf('     Slider (piston) diameter = %.4f m (%.2f in)\n', ...
        d_slider_d, d_slider_d/0.0254);

% 4.3 Physics-based total mass
M_LG_main_phys  = n_legs * M_leg_design;
M_LG_nose_phys  = 0.10/0.90 * M_LG_main_phys;  % maintain 90/10 ratio
M_LG_total_phys = M_LG_main_phys + M_LG_nose_phys;
fprintf('     Physics-based main gear mass = %.0f kg  (%.1f per leg)\n', ...
        M_LG_main_phys, M_leg_design);
fprintf('     Physics-based nose gear mass = %.0f kg\n', M_LG_nose_phys);
fprintf('     Physics-based TOTAL LG mass  = %.0f kg  (%.2f%% MTOM)\n\n', ...
        M_LG_total_phys, M_LG_total_phys/MTOM*100);

% 4.4 Raymer mass cross-check (from Schmidt "Design of Aircraft Landing Gear" p.865)
%     Accounts for λ  (transport jet version)
%     W_MLG = 0.0117 * lambda * W_land_lbs^0.95 * L_gear_in^0.43 * N_main_wheel^0.525
%     W_NLG = 0.048 * W_land_lbs^0.67 * L_nlg_in^0.43 * N_nose_wheel^0.525
L_gear_in = (h_gear_engine + xs_design) / 0.0254;   % [in]
L_nlg_in  = L_gear_in * 0.80;                         % NLG ~80% of MLG
W_land_lbs = MLM * 2.2046;
N_main_wheel = n_MLG_wheels_CI;
N_nose_wheel = n_NLG_wheels;

W_MLG_Raymer_lbs = 0.0117 * lambda_design * W_land_lbs^0.95 * L_gear_in^0.43 * N_main_wheel^0.525;
W_NLG_Raymer_lbs = 0.048  * W_land_lbs^0.67 * L_nlg_in^0.43  * N_nose_wheel^0.525;
M_MLG_Raymer = W_MLG_Raymer_lbs / 2.2046;
M_NLG_Raymer = W_NLG_Raymer_lbs / 2.2046;
M_LG_Raymer  = M_MLG_Raymer + M_NLG_Raymer;

fprintf('  >> Raymer cross-check (λ-corrected, Schmidt p.865):\n');
fprintf('     Main gear mass = %.0f kg\n', M_MLG_Raymer);
fprintf('     Nose gear mass = %.0f kg\n', M_NLG_Raymer);
fprintf('     Total LG mass  = %.0f kg  (%.2f%% MTOM)\n\n', M_LG_Raymer, M_LG_Raymer/MTOM*100);

%% =========================================================
%  SECTION 5: SUMMARY TABLE
%% =========================================================
fprintf('=======================================================\n');
fprintf(' RESULTS SUMMARY\n');
fprintf('=======================================================\n');
fprintf(' Method          | Total LG [kg] | Main [kg] | Nose [kg]\n');
fprintf(' ----------------|---------------|-----------|----------\n');
fprintf(' Class I  (4%%)   | %13.0f | %9.0f | %8.0f\n', M_LG_total, M_LG_main, M_LG_nose);
fprintf(' Class II.5 tube | %13.0f | %9.0f | %8.0f\n', M_LG_total_phys, M_LG_main_phys, M_LG_nose_phys);
fprintf(' Raymer (λ=%.1f) | %13.0f | %9.0f | %8.0f\n', lambda_design, M_LG_Raymer, M_MLG_Raymer, M_NLG_Raymer);
fprintf('\n');
fprintf(' Geometry:\n');
fprintf('   NLG position          = %.2f m from nose\n', x_nose_pos);
fprintf('   MLG position          = %.2f m from nose\n', x_mg);
fprintf('   Track width (min)     = %.2f m\n', T_track);
fprintf('   Gear length (engine)  = %.3f m\n', h_gear_engine);
fprintf('   Stroke length (λ=%.1f) = %.4f m\n', lambda_design, xs_design);
fprintf('   Tyre diameter         = %.3f m  (%.1f in)\n', D_tyre_m, D_tyre_in);
fprintf('   Slider diameter       = %.4f m  (%.2f in)\n', d_slider_d, d_slider_d/0.0254);
fprintf('\n');
fprintf(' Geometric constraints:\n');
fprintf('   Tail-strike angle     = %.1f deg  (limit: <15, aim 8-12)\n', theta_ts);
fprintf('   Tip-back angle β      = %.1f deg  (must be >%.1f and <25)\n', beta_tipback, theta_ts);
fprintf('   Turnover angle θ      = %.1f deg  (limit: <63)\n', theta_turnover);
fprintf('=======================================================\n\n');

%% =========================================================
%  SECTION 6: PLOTS
%% =========================================================

% --- Plot 1: Stroke length and peak force vs lambda ---
figure('Name','Stroke & Force vs Landing Reaction Factor','NumberTitle','off');
tiledlayout(2,1);

nexttile;
plot(lambda_vec, xs_vec*100, 'b-o','LineWidth',2,'MarkerSize',6);
hold on;
xline(lambda_design,'r--','LineWidth',1.5);
xlabel('\lambda (Landing Reaction Factor)');
ylabel('Stroke Length x_s [cm]');
title('Shock Absorber Stroke Length vs \lambda');
grid on; legend('x_s', sprintf('Design \\lambda=%.1f',lambda_design),'Location','NE');

nexttile;
plot(lambda_vec, Fpeak_vec/1e3, 'r-s','LineWidth',2,'MarkerSize',6);
hold on;
xline(lambda_design,'r--','LineWidth',1.5);
xlabel('\lambda (Landing Reaction Factor)');
ylabel('Peak Force per MLG Leg [kN]');
title('Peak Landing Force per Main Gear Leg vs \lambda');
grid on;

% --- Plot 2: LG mass comparison ---
figure('Name','Landing Gear Mass Comparison','NumberTitle','off');
methods = {'Class I (4%)', 'Class II.5 tube', 'Raymer (\lambda-corrected)'};
masses  = [M_LG_total, M_LG_total_phys, M_LG_Raymer];
bar(masses/1000,'FaceColor',[0.2 0.4 0.8]);
set(gca,'XTickLabel',methods,'FontSize',10);
ylabel('Total Landing Gear Mass [tonnes]');
title('Landing Gear Mass: Class I vs II.5 vs Raymer');
grid on;
for k = 1:3
    text(k, masses(k)/1000+0.3, sprintf('%.1f t', masses(k)/1000), ...
         'HorizontalAlignment','center','FontSize',9);
end

% --- Plot 3: Landing gear mass vs lambda (Class II.5) ---
figure('Name','LG Mass vs Lambda','NumberTitle','off');
M_total_lambda = n_legs * M_tube_vec + (0.10/0.90)*(n_legs * M_tube_vec);
plot(lambda_vec, M_total_lambda/1000, 'g-^','LineWidth',2,'MarkerSize',6);
hold on;
xline(lambda_design,'r--','LineWidth',1.5);
xlabel('\lambda (Landing Reaction Factor)');
ylabel('Total LG Mass [tonnes]');
title('Total Landing Gear Mass vs \lambda (Class II.5 Physics)');
grid on; legend('LG mass',sprintf('Design \\lambda=%.1f',lambda_design),'Location','NE');

% --- Plot 4: Landing gear geometry schematic (side view) ---
figure('Name','Landing Gear Geometry – Side View','NumberTitle','off');
hold on;
% Ground line
plot([0, l_fus], [0, 0], 'k-','LineWidth',2);
% Fuselage centreline
plot([0, l_fus], [R_fus, R_fus], 'b--','LineWidth',1);
% NLG
plot([x_nose_pos, x_nose_pos], [0, h_gear_engine], 'b-','LineWidth',3);
plot(x_nose_pos, h_gear_engine, 'bs','MarkerSize',8,'MarkerFaceColor','b');
% MLG
plot([x_mg, x_mg], [0, h_gear_engine], 'r-','LineWidth',4);
plot(x_mg, h_gear_engine, 'rs','MarkerSize',10,'MarkerFaceColor','r');
% CG
plot(x_cg, h_cg, 'kp','MarkerSize',14,'MarkerFaceColor','yellow');
% Annotations
text(x_nose_pos, -1.0, 'NLG','HorizontalAlignment','center','FontSize',10,'Color','b');
text(x_mg,        -1.0, 'MLG','HorizontalAlignment','center','FontSize',10,'Color','r');
text(x_cg,  h_cg+1.0, 'CG','HorizontalAlignment','center','FontSize',10,'Color','k');
xlabel('Position along fuselage [m]');
ylabel('Height above ground [m]');
title('Landing Gear Longitudinal Placement');
axis([0 l_fus -3 h_fus_bottom+5]);
grid on; legend('Ground','Fuselage c/l','NLG','NLG attach','MLG','MLG attach','CG','Location','NW');

fprintf('Plots generated. Check Figure windows.\n');
fprintf('Script complete.\n');