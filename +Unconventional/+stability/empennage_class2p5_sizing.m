%% ============================================================
% EMPENNAGE_CLASS2P5_SIZING.M
% Widebody cargo aircraft empennage, control-surface and CG sizing tool
%
% PURPOSE
%   Progresses a Class I tail-volume sizing into a Class II / Class II.5-
%   style empennage sizing workflow using explicit sizing cases.
%
% WHAT IT DOES
%   1) Builds a Class I baseline using tail volume coefficients and
%      Snorri/Gudmundsson style tail-arm optimisation.
%   2) Builds a CG envelope from OWE, payload loading paths and fuel.
%   3) Sizes the horizontal tail against explicit criteria:
%        - forward-CG stall authority
%        - low-speed trim at forward CG with full thrust
%        - take-off rotation / nose lift about the main gear
%        - high-speed trim at aft CG
%        - static-margin requirement
%        - tail incidence / pitch-rate stall margin
%   4) Sizes the vertical tail against explicit criteria:
%        - OEI take-off yawing moment balance
%        - crosswind landing / take-off yaw control
%        - minimum directional stability target
%   5) Sizes elevator and rudder from required control effectiveness.
%   6) Adds empennage mass penalties, updates OWE + OWE CG and iterates
%      to convergence.
%   7) Produces sensitivity studies around the final design point.
%
% IMPORTANT
%   - This is intended to be a robust project-level sizing script, not a
%     certification-grade flight mechanics model.
%   - The aerodynamic derivative model is deliberately simplified.
%   - Replace the editable aero / propulsion placeholders with your own
%     AVL / CFD / handbook-calibrated values when available.
%
% UNITS
%   SI throughout: m, kg, N, rad
%% ============================================================

clear; clc; close all;

%% ============================================================
% 0) USER INPUTS - AIRCRAFT, MASS, MISSION, AERO, GEAR
%% ============================================================

% -----------------------------
% Aircraft / wing geometry
% -----------------------------
ac.name      = 'Widebody cargo concept';
ac.L_fus     = 65.0;     % fuselage length [m]
ac.D_fus     = 6.40;     % representative fuselage diameter [m]
ac.z_cg_ref  = 3.2;      % representative CG height above datum [m]
ac.n_ult     = 3.75;     % conceptual ultimate load factor
ac.isNovel   = false;    % set true for non-conventional layouts

wing.S       = 820.0;    % wing reference area [m^2]
wing.b       = 72.0;     % wing span [m]
wing.cMGC    = 12.0;     % mean geometric chord [m]
wing.xLEMAC  = 27.0;     % MAC leading edge x [m]
wing.sweepLE = 32.0;     % wing leading-edge sweep [deg]
wing.dihedral= 4.0;      % wing dihedral [deg]
wing.AR      = wing.b^2 / wing.S;
wing.x_qc    = wing.xLEMAC + 0.25*wing.cMGC;
wing.h_ac_wb = 0.24;     % wing-body aerodynamic centre as fraction MAC
wing.Cm0_wb  = -0.045;   % wing-body pitching moment coefficient about ac
wing.CL0     = 0.25;     % approx clean lift coefficient at alpha = 0
wing.alpha0  = deg2rad(-2.0);
wing.alpha_stall_clean = deg2rad(13.0);
wing.alpha_stall_land  = deg2rad(15.0);
wing.CLmax_to = 2.10;
wing.CLmax_ld = 2.65;

% -----------------------------
% Class I tail volume inputs
% -----------------------------
tailCfg.VHT0      = 1.00;
tailCfg.VVT0      = 0.09;
tailCfg.AR_HT     = 4.0;
tailCfg.AR_VT     = 1.6;
tailCfg.lambdaHT  = 0.35;
tailCfg.lambdaVT  = 0.35;
tailCfg.sweepLE_HT= 32.0;
tailCfg.sweepLE_VT= 38.0;
tailCfg.i_t       = deg2rad(-4.0);  % tail setting angle [rad]
tailCfg.i_v       = 0.0;
tailCfg.alpha_stall_HT = deg2rad(16.0);
tailCfg.alpha_stall_VT = deg2rad(18.0);
tailCfg.cec_max   = 0.45;  % max elevator chord ratio
% Raymer-style typical range roughly 0.25 to 0.50 of tail chord
% for elevators / rudders.
tailCfg.crc_max   = 0.45;  % max rudder chord ratio

% Snorri / Gudmundsson Method 3 fuselage radii for Class I arm estimate
aft.R1 = 3.20;
aft.R2 = 1.10;

% -----------------------------
% Mass model and payload/fuel
% -----------------------------
mass.OWE_fixed   = 255000;  % OWE excluding empennage mass carried by this script [kg]
mass.x_OWE_fixed = 31.0;    % x-location of fixed OWE CG [m]
mass.z_OWE_fixed = 3.2;     % z-location of fixed OWE CG [m]

fuel.m_fuel      = 112000;  % usable fuel mass [kg]
fuel.x_fuel      = wing.x_qc;
fuel.m_landing   = 28000;   % landing fuel for MLW / approach cases [kg]

cargo.N          = 12;
cargo.m_each     = 8000;
cargo.x_start    = 9.0;
cargo.pitch      = 3.2;
cargo.x          = cargo.x_start + ((1:cargo.N) - 0.5) * cargo.pitch;

W.MZFW           = mass.OWE_fixed + cargo.N * cargo.m_each;
W.MTOW_guess     = W.MZFW + fuel.m_fuel;
W.MLW_guess      = 0.92 * W.MTOW_guess;

% -----------------------------
% Flight CG limits / targets
% These are the target usable flight limits; the loading cloud must fit.
% -----------------------------
req.fwd_flight_pctMAC = 15.0;
req.aft_flight_pctMAC = 35.0;
req.SM_target         = 0.08;   % target static margin at aft CG [fraction MAC]
req.CNb_target        = 0.070;  % minimum directional stability target [1/rad]
req.beta_crosswind    = deg2rad(15.0);
req.beta_OEI_allow    = deg2rad(5.0);
req.q_pitch_max       = deg2rad(8.0); % representative high pitch rate [rad/s]

% -----------------------------
% Landing gear geometry
% -----------------------------
gear.x_nose = 7.0;
gear.x_main = 35.0;
gear.z_main = 0.0;
gear.min_nose_frac_takeoff = 0.025;
gear.max_nose_load = 0.20 * W.MTOW_guess;
gear.max_main_load = 0.90 * W.MTOW_guess;

% -----------------------------
% Propulsion / OEI / thrust-line data
% -----------------------------
prop.n_engines       = 4;
prop.T_sl_total      = 1.46e6;   % total static thrust [N] placeholder
prop.T_balked_total  = 1.20e6;   % full thrust for low-speed trim / balked landing [N]
prop.T_cruise_total  = 2.80e5;   % representative cruise thrust [N]
prop.y_engine_outbd  = 22.0;     % outer engine CG spanwise location [m]
prop.z_thrust_line   = 1.5;      % thrust line height above datum [m]
prop.k_OEI           = 1.05;     % OEI yawing-moment margin factor
prop.oneEngineOutThrust = 0.25 * prop.T_sl_total; % thrust lost when one engine fails [N]

% -----------------------------
% Mission / flight condition definitions
% -----------------------------
perf.Mach_cruise     = 0.82;
perf.alt_cruise      = 10668;    % 35,000 ft [m]
perf.V_app_margin    = 1.20;     % 1.2 VS0
perf.V_rot_factor    = 0.90;     % 0.9 VS1 criterion
perf.V_high_margin   = 10 * 0.514444; % +10 kt [m/s]
perf.rho_sl          = 1.225;

% -----------------------------
% Simplified aerodynamic placeholders (edit as your aero model improves)
% -----------------------------
aero.eta_t      = 0.92;   % horizontal tail efficiency
aero.eta_v      = 0.95;   % vertical tail efficiency
aero.deps_dalpha= 0.45;   % downwash gradient at tail
aero.dsigma_dbeta = 0.10; % sidewash gradient at fin
aero.CNb_fuse   = -0.020; % destabilising fuselage contribution [1/rad]
aero.CYbeta_side= 0.95;   % side-force derivative of fuselage/side area [1/rad]
aero.alpha_t0   = 0.0;    % zero-offset at tail [rad]
aero.beta_v0    = 0.0;    % zero-offset at fin [rad]
aero.k_trimDrag = 1.08;   % multiplier for trim drag estimate

% Fuselage side area estimate for crosswind yawing moment
side.S_side = 0.70 * ac.L_fus * ac.D_fus;  % rough side projected area [m^2]
side.x_centroid = 0.42 * ac.L_fus;         % representative side-area centroid [m]

% -----------------------------
% Search and convergence settings
% -----------------------------
opt.maxIter           = 15;
opt.massTol           = 2.0;      % kg
opt.cgTol             = 1e-4;     % m
opt.armSearch         = linspace(0.65, 1, 50);
opt.ShtSearchMult     = linspace(0.30, 4.00, 300);
opt.SvtSearchMult     = linspace(0.30, 4.00, 300);
opt.tailArmPenaltyExp = 1.00;

%% ============================================================
% 1) CLASS I BASELINE - TAIL ARM + INITIAL AREAS
%% ============================================================

class1.lT = sqrt( 2 * wing.S * (tailCfg.VHT0 * wing.cMGC + tailCfg.VVT0 * wing.b) / ...
                 (pi * (aft.R1 + aft.R2)) );
class1.Sht = tailCfg.VHT0 * wing.S * wing.cMGC / class1.lT;
class1.Svt = tailCfg.VVT0 * wing.S * wing.b    / class1.lT;

fprintf('\n============================================================\n');
fprintf('CLASS I BASELINE\n');
fprintf('------------------------------------------------------------\n');
fprintf('Optimum common Class I tail arm lT0     = %8.3f m\n', class1.lT);
fprintf('Class I horizontal tail area Sht0       = %8.3f m^2\n', class1.Sht);
fprintf('Class I vertical tail area Svt0         = %8.3f m^2\n', class1.Svt);

%% ============================================================
% 2) CONVERGED CLASS II / II.5 SIZING LOOP
%% ============================================================

m_HT  = 0.0;
m_VT  = 0.0;
m_e   = 0.0;
m_r   = 0.0;
convHist = [];

for iter = 1:opt.maxIter
    fprintf('\n================ ITERATION %d / %d ================\n', iter, opt.maxIter);
    drawnow;
    % --------------------------------------------------------
    % 2.1) Update OWE including current empennage estimate
    % --------------------------------------------------------
    m_tail_tot = m_HT + m_VT + m_e + m_r;
    x_tail_ref = wing.x_qc + class1.lT; % temporary before re-sizing this iter

    [massNow, cgNow] = update_mass_and_cg(mass, m_tail_tot, x_tail_ref, ac.z_cg_ref);

    % Build loading cloud / CG envelope with current mass model.
    CG = build_cg_envelope(massNow, wing, cargo, fuel, W, req, gear);

    % --------------------------------------------------------
    % 2.2) Find minimum-mass tail arm / tail area combination
    % --------------------------------------------------------
    best = struct();
    best.totalMass = inf;
    best.pass = false;

    a_w = finite_wing_lift_curve_slope(wing.AR, wing.sweepLE, perf.Mach_cruise);
    a_t = finite_wing_lift_curve_slope(tailCfg.AR_HT, tailCfg.sweepLE_HT, 0.2);
    a_v = finite_wing_lift_curve_slope(tailCfg.AR_VT, tailCfg.sweepLE_VT, 0.2);

    for armScale = opt.armSearch
        lT = armScale * class1.lT;

        HT = size_horizontal_tail(lT, class1, wing, tailCfg, massNow, CG, gear, prop, perf, req, aero, opt, a_w, a_t);
        if ~HT.pass
            fprintf('HT failed at lT = %.3f m\n', lT);
            continue;
        end
        VT = size_vertical_tail(lT, class1, wing, tailCfg, massNow, CG, prop, perf, req, aero, side, opt, a_v);
        if ~VT.pass
            fprintf('VT failed at lT = %.3f m\n', lT);
            continue;
        end

        % Masses and objective.
        masses = estimate_empennage_masses(HT, VT, massNow, ac);
        totalMass = masses.total * (lT / class1.lT) ^ opt.tailArmPenaltyExp;

        if totalMass < best.totalMass
            best.totalMass = totalMass;
            best.pass = true;
            best.lT = lT;
            best.HT = HT;
            best.VT = VT;
            best.masses = masses;
        end
    end

    if ~best.pass
        error('No empennage solution found. Widen search ranges or relax placeholder assumptions.');
    end

    % --------------------------------------------------------
    % 2.3) Update mass and CG with selected empennage
    % --------------------------------------------------------
    x_tail_mass = wing.x_qc + best.lT;
    [massNew, cgNew] = update_mass_and_cg(mass, best.masses.total, x_tail_mass, ac.z_cg_ref);

    convHist = [convHist; ...
        iter, best.lT, best.HT.S, best.VT.S, best.masses.mHT, best.masses.mVT, ...
        best.masses.mElev, best.masses.mRudd, massNew.OWE, cgNew.x_OWE]; %#ok<AGROW>

    dMass = abs(massNew.OWE - massNow.OWE);
    dCG   = abs(cgNew.x_OWE - cgNow.x_OWE);

    m_HT = best.masses.mHT;
    m_VT = best.masses.mVT;
    m_e  = best.masses.mElev;
    m_r  = best.masses.mRudd;

    if dMass < opt.massTol && dCG < opt.cgTol
        final = best;
        final.mass = massNew;
        final.cg   = cgNew;
        break;
    end

    if iter == opt.maxIter
        final = best;
        final.mass = massNew;
        final.cg   = cgNew;
        warning('Empennage sizing stopped at maxIter before strict convergence.');
    end
end

% Rebuild final CG envelope with converged masses.
CG = build_cg_envelope(final.mass, wing, cargo, fuel, W, req, gear);
final.CG = CG;

%% ============================================================
% 3) FINAL REPORT TO CONSOLE
%% ============================================================

fprintf('\n============================================================\n');
fprintf('FINAL CLASS II / II.5 EMPENNAGE SOLUTION\n');
fprintf('------------------------------------------------------------\n');
fprintf('Converged tail arm lT                     = %8.3f m\n', final.lT);
fprintf('HT area Sht                              = %8.3f m^2\n', final.HT.S);
fprintf('VT area Svt                              = %8.3f m^2\n', final.VT.S);
fprintf('HT span                                  = %8.3f m\n', final.HT.b);
fprintf('VT height                                = %8.3f m\n', final.VT.b);
fprintf('Elevator chord ratio ce/c                = %8.3f\n', final.HT.cec);
fprintf('Rudder chord ratio cr/c                  = %8.3f\n', final.VT.crc);
fprintf('Horizontal tail volume VHT               = %8.3f\n', final.HT.VHT);
fprintf('Vertical tail volume VVT                 = %8.3f\n', final.VT.VVT);
fprintf('OWE including empennage                  = %8.1f kg\n', final.mass.OWE);
fprintf('OWE CG x                                 = %8.3f m\n', final.cg.x_OWE);
fprintf('Forward CG limit (active)                = %8.3f %%MAC\n', CG.fwd_pctMAC_active);
fprintf('Aft CG limit (active)                    = %8.3f %%MAC\n', CG.aft_pctMAC_active);

fprintf('\nHT sizing drivers:\n');
print_case_summary(final.HT.caseTable);

fprintf('\nVT sizing drivers:\n');
print_case_summary(final.VT.caseTable);

if ac.isNovel
    fprintf('\nNovel layout flag is ON.\n');
    fprintf('For the report, explicitly discuss deep-stall / wake blanketing,\n');
    fprintf('control coupling, and any directional-stability penalties or gains.\n');
else
    fprintf('\nConventional layout flag is ON.\n');
    fprintf('Report discussion should state that no additional unconventional\n');
    fprintf('stability/control penalty was assumed beyond the normal sizing cases.\n');
end

%% ============================================================
% 4) PLOTS - PLANFORMS, CG, CONVERGENCE, SENSITIVITY
%% ============================================================

% Build final planforms.
HTplot = make_trapezoid(final.HT.S, final.HT.b, tailCfg.lambdaHT);
VTplot = make_trapezoid(final.VT.S, final.VT.b, tailCfg.lambdaVT);

HTplot.cMGC = mean_geom_chord(HTplot.c_root, HTplot.lambda);
HTplot.yMGC = y_mgc(HTplot.b, HTplot.lambda);
HTplot.x_qc = wing.x_qc + final.lT;
HTplot.x_apex = HTplot.x_qc - (HTplot.yMGC * tand(tailCfg.sweepLE_HT) + 0.25 * HTplot.cMGC);
[HTplot.x_poly, HTplot.y_poly] = tail_planform_polygon(HTplot.x_apex, HTplot.c_root, HTplot.c_tip, HTplot.b, tailCfg.sweepLE_HT);

VTplot.cMGC = mean_geom_chord(VTplot.c_root, VTplot.lambda);
VTplot.zMGC = y_mgc(VTplot.b, VTplot.lambda);
VTplot.x_qc = wing.x_qc + final.lT;
VTplot.x_apex = VTplot.x_qc - (VTplot.zMGC * tand(tailCfg.sweepLE_VT) + 0.25 * VTplot.cMGC);
[VTplot.x_poly, VTplot.z_poly] = fin_planform_polygon(VTplot.x_apex, VTplot.c_root, VTplot.c_tip, VTplot.b, tailCfg.sweepLE_VT);

figure('Name','Horizontal Tail Planform');
plot(HTplot.x_poly, HTplot.y_poly, 'k-', 'LineWidth', 1.5); hold on;
fill(HTplot.x_poly, HTplot.y_poly, [0.85 0.90 1.00], 'EdgeColor', 'k');
axis equal; grid on;
xlabel('x from nose datum [m]');
ylabel('y [m]');
title('Horizontal Tail Planform (Final Class II / II.5 Sizing)');
xline(wing.x_qc, '--', 'Wing c/4');
xline(HTplot.x_qc, '--', 'HT c/4');
set(gca, 'FontSize', 14);

figure('Name','Vertical Tail Planform');
plot(VTplot.x_poly, VTplot.z_poly, 'k-', 'LineWidth', 1.5); hold on;
fill(VTplot.x_poly, VTplot.z_poly, [0.90 0.95 0.85], 'EdgeColor', 'k');
axis equal; grid on;
xlabel('x from nose datum [m]');
ylabel('z [m]');
title('Vertical Tail Planform (Final Class II / II.5 Sizing)');
xline(wing.x_qc, '--', 'Wing c/4');
xline(VTplot.x_qc, '--', 'VT c/4');
set(gca, 'FontSize', 14);

figure('Name','CG Envelope and Loading Cloud'); hold on; grid on; box on;
plot(CG.pathF.pctMAC, CG.pathF.W/1000, 'b-o', 'LineWidth', 1.2, 'MarkerSize', 4, 'DisplayName', 'Loading path: nose to tail');
plot(CG.pathA.pctMAC, CG.pathA.W/1000, 'g-s', 'LineWidth', 1.2, 'MarkerSize', 4, 'DisplayName', 'Loading path: tail to nose');
xline(req.fwd_flight_pctMAC, 'r--', 'DisplayName', 'Target forward flight limit');
xline(req.aft_flight_pctMAC, 'r--', 'DisplayName', 'Target aft flight limit');
xline(CG.aft_TO_pctMAC, 'm--', 'DisplayName', 'Aft take-off ground limit');
plot(CG.fwd_struct_pctMAC, CG.Wgrid/1000, 'c-', 'LineWidth', 1.3, 'DisplayName', 'Max nose gear limit');
plot(CG.aft_struct_pctMAC, CG.Wgrid/1000, 'y-', 'LineWidth', 1.3, 'DisplayName', 'Max main gear limit');
yline(CG.MZFW/1000, '--', 'MZFW');
yline(CG.MLW/1000, '--', 'MLW');
yline(CG.MTOW/1000, '--', 'MTOW');
xlabel('CG location [% MAC]');
ylabel('Aircraft weight [t]');
title('Final CG Envelope / Loading Cloud');
legend('Location','best');
set(gca, 'FontSize', 14);

figure('Name','Empennage Convergence History');
subplot(2,1,1);
plot(convHist(:,1), convHist(:,2), '-o', 'LineWidth', 1.2); grid on;
ylabel('Tail arm lT [m]');
title('Empennage Convergence');
subplot(2,1,2);
plot(convHist(:,1), convHist(:,9), '-o', 'LineWidth', 1.2); hold on;
plot(convHist(:,1), convHist(:,10), '-s', 'LineWidth', 1.2);
grid on;
xlabel('Iteration');
ylabel('OWE / x_{CG}');
legend('OWE [kg]', 'OWE x_{CG} [m]', 'Location', 'best');

%% ------------------------------------------------------------
% 4.1) Sensitivity studies around final design point
% FEDR support: sensitivity around final design point, CG range at final
% design point, plus critical-case control sizing.
%% ------------------------------------------------------------

% Study A: HT mass sensitivity to static margin target and required CG range.
SM_vec = linspace(max(0.04, req.SM_target - 0.04), req.SM_target + 0.05, 21);
CGaft_vec = linspace(req.aft_flight_pctMAC - 5, req.aft_flight_pctMAC + 5, 21);
Mht_map = nan(numel(CGaft_vec), numel(SM_vec));

for i = 1:numel(CGaft_vec)
    reqTmp = req;
    reqTmp.aft_flight_pctMAC = CGaft_vec(i);
    for j = 1:numel(SM_vec)
        reqTmp.SM_target = SM_vec(j);
        HTtmp = size_horizontal_tail(final.lT, class1, wing, tailCfg, final.mass, ...
            build_cg_envelope(final.mass, wing, cargo, fuel, W, reqTmp, gear), ...
            gear, prop, perf, reqTmp, aero, opt, a_w, a_t);
        if HTtmp.pass
            tmpMasses = estimate_empennage_masses(HTtmp, final.VT, final.mass, ac);
            Mht_map(i,j) = tmpMasses.mHT + tmpMasses.mElev;
        end
    end
end

figure('Name','HT mass sensitivity');
imagesc(SM_vec, CGaft_vec, Mht_map);
set(gca,'YDir','normal'); colorbar;
xlabel('Target static margin at aft CG [-]');
ylabel('Aft CG limit [% MAC]');
title('Horizontal Tail + Elevator Mass Sensitivity [kg]');

% Study B: VT mass sensitivity to OEI thrust loss and crosswind requirement.
OEI_vec = linspace(0.8, 1.2, 21) * prop.oneEngineOutThrust;
Beta_vec = deg2rad(linspace(10, 20, 21));
Mvt_map = nan(numel(Beta_vec), numel(OEI_vec));

for i = 1:numel(Beta_vec)
    reqTmp = req;
    reqTmp.beta_crosswind = Beta_vec(i);
    for j = 1:numel(OEI_vec)
        propTmp = prop;
        propTmp.oneEngineOutThrust = OEI_vec(j);
        VTtmp = size_vertical_tail(final.lT, class1, wing, tailCfg, final.mass, final.CG, ...
            propTmp, perf, reqTmp, aero, side, opt, a_v);
        if VTtmp.pass
            tmpMasses = estimate_empennage_masses(final.HT, VTtmp, final.mass, ac);
            Mvt_map(i,j) = tmpMasses.mVT + tmpMasses.mRudd;
        end
    end
end

figure('Name','VT mass sensitivity');
imagesc(OEI_vec/1000, rad2deg(Beta_vec), Mvt_map);
set(gca,'YDir','normal'); colorbar;
xlabel('OEI thrust loss [kN]');
ylabel('Crosswind sizing yaw angle beta [deg]');
title('Vertical Tail + Rudder Mass Sensitivity [kg]');

% Study C: local finite-difference sensitivities for tornado chart.
baseMetrics.mHT = final.masses.mHT + final.masses.mElev;
baseMetrics.mVT = final.masses.mVT + final.masses.mRudd;

pertNames = {'aft CG limit', 'static margin', 'OEI thrust loss', 'crosswind beta'};
pertPct = [0.05, 0.10, 0.10, 0.10];
HTsens = zeros(size(pertPct));
VTsens = zeros(size(pertPct));

for k = 1:numel(pertPct)
    reqP = req; propP = prop;
    switch k
        case 1
            reqP.aft_flight_pctMAC = req.aft_flight_pctMAC * (1 + pertPct(k));
        case 2
            reqP.SM_target = req.SM_target * (1 + pertPct(k));
        case 3
            propP.oneEngineOutThrust = prop.oneEngineOutThrust * (1 + pertPct(k));
        case 4
            reqP.beta_crosswind = req.beta_crosswind * (1 + pertPct(k));
    end

    CGP = build_cg_envelope(final.mass, wing, cargo, fuel, W, reqP, gear);
    HTP = size_horizontal_tail(final.lT, class1, wing, tailCfg, final.mass, CGP, gear, propP, perf, reqP, aero, opt, a_w, a_t);
    VTP = size_vertical_tail(final.lT, class1, wing, tailCfg, final.mass, CGP, propP, perf, reqP, aero, side, opt, a_v);

    if HTP.pass
        mP = estimate_empennage_masses(HTP, final.VT, final.mass, ac);
        HTsens(k) = 100 * ((mP.mHT + mP.mElev) - baseMetrics.mHT) / baseMetrics.mHT;
    end
    if VTP.pass
        mP = estimate_empennage_masses(final.HT, VTP, final.mass, ac);
        VTsens(k) = 100 * ((mP.mVT + mP.mRudd) - baseMetrics.mVT) / baseMetrics.mVT;
    end
end

figure('Name','Local sensitivities');
barh(categorical(pertNames), [HTsens(:), VTsens(:)]);
grid on;
xlabel('Mass change from + perturbation [%]');
title('Local Sensitivities Around the Final Design Point');
legend('HT + elevator mass', 'VT + rudder mass', 'Location', 'best');

%% ============================================================
% 5) EXPORT RESULTS STRUCTURE TO WORKSPACE
%% ============================================================

results.final      = final;
results.convHist   = convHist;
results.class1     = class1;
results.inputs.ac  = ac;
results.inputs.wing= wing;
results.inputs.tailCfg = tailCfg;
results.inputs.req = req;
assignin('base', 'results_empennage', results);

fprintf('\nWorkspace variable written: results_empennage\n');

%% ============================================================
% LOCAL FUNCTIONS
%% ============================================================

function [massOut, cgOut] = update_mass_and_cg(mass, mTail, xTail, zTail)
    massOut = mass;
    massOut.OWE = mass.OWE_fixed + mTail;

    xsum = mass.OWE_fixed * mass.x_OWE_fixed + mTail * xTail;
    zsum = mass.OWE_fixed * mass.z_OWE_fixed + mTail * zTail;

    cgOut.x_OWE = xsum / massOut.OWE;
    cgOut.z_OWE = zsum / massOut.OWE;

    % make CG available in massOut too, because downstream code uses massNow.x_OWE
    massOut.x_OWE = cgOut.x_OWE;
    massOut.z_OWE = cgOut.z_OWE;
end

function CG = build_cg_envelope(massNow, wing, cargo, fuel, W, req, gear)
    W.MZFW = massNow.OWE + cargo.N * cargo.m_each;
    W.MTOW = W.MZFW + fuel.m_fuel;
    W.MLW  = massNow.OWE + cargo.N * cargo.m_each + fuel.m_landing;

    order_fwd = 1:cargo.N;
    order_aft = cargo.N:-1:1;

    [WF, xF] = loading_path(massNow.OWE, massNow.x_OWE, cargo.m_each, cargo.x(order_fwd), fuel.m_fuel, fuel.x_fuel);
    [WA, xA] = loading_path(massNow.OWE, massNow.x_OWE, cargo.m_each, cargo.x(order_aft), fuel.m_fuel, fuel.x_fuel);

    pathF.W = WF; pathF.xcg = xF; pathF.pctMAC = x_to_pctMAC(xF, wing.xLEMAC, wing.cMGC);
    pathA.W = WA; pathA.xcg = xA; pathA.pctMAC = x_to_pctMAC(xA, wing.xLEMAC, wing.cMGC);

    Wgrid = linspace(massNow.OWE, W.MTOW, 250);
    x_aft_TO = gear.x_main - gear.min_nose_frac_takeoff * (gear.x_main - gear.x_nose);
    aft_TO_pctMAC = x_to_pctMAC(x_aft_TO, wing.xLEMAC, wing.cMGC);

    x_fwd_struct = gear.x_main - gear.max_nose_load * (gear.x_main - gear.x_nose) ./ Wgrid;
    x_aft_struct = gear.x_nose + gear.max_main_load * (gear.x_main - gear.x_nose) ./ Wgrid;

    fwd_struct_pctMAC = x_to_pctMAC(x_fwd_struct, wing.xLEMAC, wing.cMGC);
    aft_struct_pctMAC = x_to_pctMAC(x_aft_struct, wing.xLEMAC, wing.cMGC);

    fwd_loading = min([pathF.pctMAC(:); pathA.pctMAC(:)]);
    aft_loading = max([pathF.pctMAC(:); pathA.pctMAC(:)]);

    fwd_active = max([req.fwd_flight_pctMAC, max(fwd_struct_pctMAC)]);
    aft_active = min([req.aft_flight_pctMAC, aft_TO_pctMAC, min(aft_struct_pctMAC)]);

    CG.pathF = pathF;
    CG.pathA = pathA;
    CG.Wgrid = Wgrid;
    CG.fwd_struct_pctMAC = fwd_struct_pctMAC;
    CG.aft_struct_pctMAC = aft_struct_pctMAC;
    CG.aft_TO_pctMAC = aft_TO_pctMAC;
    CG.fwd_pctMAC_loading = fwd_loading;
    CG.aft_pctMAC_loading = aft_loading;
    CG.fwd_pctMAC_active = fwd_active;
    CG.aft_pctMAC_active = aft_active;
    CG.h_fwd = fwd_loading / 100;
    CG.h_aft = aft_loading / 100;
    CG.MZFW = W.MZFW;
    CG.MTOW = W.MTOW;
    CG.MLW  = W.MLW;
end
function HT = size_horizontal_tail(lT, class1, wing, tailCfg, massNow, CG, gear, prop, perf, req, aero, opt, a_w, a_t)
% SIZE_HORIZONTAL_TAIL
% Class II / II.5 horizontal tail sizing against:
%   1) forward CG stall authority
%   2) low-speed trim at forward CG
%   3) take-off rotation / nose lift
%   4) high-speed trim at aft CG
%   5) aft-CG static margin
%
% Notes:
% - Sign conventions are handled explicitly using moments about CG.
% - Positive aerodynamic pitching moment coefficient is nose-up.
% - Conventional aft tail: upward tail lift gives nose-DOWN moment, so
%   M_tail_about_cg = -Lh * (x_ht - x_cg).
% - Elevator authority is screened using a simple linear tail model.
%
% Returns:
%   HT.pass        : true if feasible solution found
%   HT.S           : selected horizontal tail area [m^2]
%   HT.b           : span [m]
%   HT.c_root      : root chord [m]
%   HT.c_tip       : tip chord [m]
%   HT.cMGC        : mean geometric chord [m]
%   HT.cec         : required elevator chord ratio c_e/c
%   HT.tau         : required flap effectiveness parameter
%   HT.VHT         : horizontal tail volume coefficient
%   HT.caseTable   : per-case pass/fail summary
%   HT.maxAlphaTail: max effective tail alpha incl. pitch-rate term [rad]
%   HT.SM          : achieved static margin at aft CG [-]

    Sgrid = class1.Sht * opt.ShtSearchMult;

    % Maximum elevator effectiveness available from chosen max c_e/c
    tauMax = flap_effectiveness_from_chord_ratio(tailCfg.cec_max);

    % Default return
    HT = struct();
    HT.pass = false;

    bestS = inf;
    best = struct();

    % Cruise atmosphere
    rhoCruise = isa_density(perf.alt_cruise);
    aSound    = isa_speed_of_sound(perf.alt_cruise);
    Vcr       = perf.Mach_cruise * aSound;

    % Use active CG envelope clipped by required flight limits
    hFwdUse = max(CG.fwd_pctMAC_loading, req.fwd_flight_pctMAC) / 100;
    hAftUse = min(CG.aft_pctMAC_loading, req.aft_flight_pctMAC) / 100;

    % Placeholder design masses used by the original script
    WfwdLand  = massNow.OWE + 12 * 8000 + 28000;
    WaftCruise = massNow.OWE + 12 * 8000 + 112000;

    % Reference stall speeds
    Vs0 = sqrt(2 * WfwdLand   * 9.81 / (perf.rho_sl * wing.S * wing.CLmax_ld));
    Vs1 = sqrt(2 * WaftCruise * 9.81 / (perf.rho_sl * wing.S * wing.CLmax_to));

    % Diagnostics
    nFail_stall      = 0;
    nFail_trimFwd    = 0;
    nFail_rotation   = 0;
    nFail_trimAft    = 0;
    nFail_tailStall  = 0;
    nFail_static     = 0;
    nFail_cec        = 0;
    nTotal           = 0;

    % Candidate cases
    cases(1) = make_pitch_case( ...
        'Forward CG stall authority', ...
        perf.rho_sl, Vs0, hFwdUse, wing.alpha_stall_land, ...
        prop.T_balked_total, false, false);

    cases(2) = make_pitch_case( ...
        'Low-speed trim forward CG', ...
        perf.rho_sl, perf.V_app_margin * Vs0, hFwdUse, ...
        wing.alpha_stall_land - deg2rad(2), ...
        prop.T_balked_total, false, false);

    cases(3) = make_pitch_case( ...
        'Take-off rotation / nose lift', ...
        perf.rho_sl, perf.V_rot_factor * Vs1, hFwdUse, ...
        deg2rad(8), 0.90 * prop.T_sl_total, true, false);

    cases(4) = make_pitch_case( ...
        'High-speed trim aft CG', ...
        rhoCruise, Vcr + perf.V_high_margin, hAftUse, ...
        deg2rad(2), prop.T_cruise_total, false, false);

    cases(5) = make_pitch_case( ...
        'Static margin at aft CG', ...
        rhoCruise, Vcr, hAftUse, deg2rad(2), ...
        prop.T_cruise_total, false, true);

    cases(1).W = WfwdLand;
    cases(2).W = WfwdLand;
    cases(3).W = WaftCruise;
    cases(4).W = WaftCruise;
    cases(5).W = WaftCruise;

    for i = 1:numel(Sgrid)
        nTotal = nTotal + 1;

        Sht = Sgrid(i);
        geom = make_trapezoid(Sht, sqrt(tailCfg.AR_HT * Sht), tailCfg.lambdaHT);
        geom.cMGC = mean_geom_chord(geom.c_root, geom.lambda);

        VHT = lT * Sht / (wing.S * wing.cMGC);

        % Tail / CG geometry
        x_ac_w  = wing.xLEMAC + wing.h_ac_wb * wing.cMGC;
        x_cg_fwd = wing.xLEMAC + hFwdUse * wing.cMGC;
        x_cg_aft = wing.xLEMAC + hAftUse * wing.cMGC;
        x_ht    = wing.x_qc + lT;

        % Static margin screen at aft CG
        h_n = wing.h_ac_wb + aero.eta_t * (a_t / a_w) * ...
              (1 - aero.deps_dalpha) * (Sht / wing.S) * (lT / wing.cMGC);

        SM = h_n - hAftUse;
        if SM < req.SM_target
            nFail_static = nFail_static + 1;
            continue;
        end

        passAll = true;
        failBucket = "";
        caseTable = repmat(struct( ...
            'name','', 'required',0, 'available',0, 'pass',false), numel(cases), 1);

        maxTauReq    = 0;
        maxAlphaTail = -inf;

        for k = 1:numel(cases)
            q = 0.5 * cases(k).rho * cases(k).V^2;

            % Aircraft wing CL required in this condition
            CL = (cases(k).W * 9.81) / (q * wing.S);

            % Wing alpha from linear lift curve
            alpha_w = (CL - wing.CL0) / max(a_w, 1e-9) + wing.alpha0;

            % Use forward or aft CG consistently for each case
            if cases(k).h <= 0.5 * (hFwdUse + hAftUse)
                x_cg = x_cg_fwd;
            else
                x_cg = x_cg_aft;
            end

            % Placeholder zCG
            zcg = 3.2;

            % Wing-body-thrust pitching moment coefficient about CG
            CmT  = thrust_pitching_moment(cases(k).T, prop.z_thrust_line, zcg, q, wing.S, wing.cMGC);
            CmCG = wing.Cm0_wb + (cases(k).h - wing.h_ac_wb) * CL + CmT;

            % Tail effective angle:
            % alpha_t = alpha_w + i_t - eps + alpha_t0
            eps     = aero.deps_dalpha * alpha_w;
            alpha_t = aero.alpha_t0 + alpha_w + tailCfg.i_t - eps;

            % Pitch-rate augmented tail alpha screen
            % Conceptual tail-stall screen:
            % use a reduced pitch-rate increment factor to avoid over-penalising long tail arms
            k_q = 0.30;
            alphaTailEff = alpha_t + k_q * req.q_pitch_max * lT / max(cases(k).V, 1);
            maxAlphaTail = max(maxAlphaTail, alphaTailEff);

            % -------------------------------------------------------------
            % Rotation case: compare available moment about main gear
            % -------------------------------------------------------------
            if cases(k).isRotation
                CLrot = min(0.85 * wing.CLmax_to, CL);
                Lw    = q * wing.S * CLrot;

                % Approximate required nose-up moment about main gear
                Mreq = max(0, ...
                    cases(k).W * 9.81 * (gear.x_main - x_cg) ...
                  - Lw * (gear.x_main - x_ac_w) ...
                  - cases(k).T * (zcg - gear.z_main));

                % Maximum available tail coefficient with max elevator
                CLh_max = abs(a_t * (alpha_t + tauMax * deg2rad(25)));

                % Max tail lift magnitude
                LhMax = q * Sht * CLh_max * aero.eta_t;

                % Tail is aft of main gear; use positive magnitude here
                Mavail = LhMax * max(x_ht - gear.x_main, 0);

                caseTable(k).name      = cases(k).name;
                caseTable(k).required  = Mreq;
                caseTable(k).available = Mavail;
                caseTable(k).pass      = (Mavail >= Mreq);

                if ~caseTable(k).pass
                    passAll = false;
                    failBucket = "rotation";
                    break;
                end

                continue;
            end

            % -------------------------------------------------------------
            % Static margin case
            % -------------------------------------------------------------
            if cases(k).isStaticMargin
                caseTable(k).name      = cases(k).name;
                caseTable(k).required  = req.SM_target;
                caseTable(k).available = SM;
                caseTable(k).pass      = (SM >= req.SM_target);

                if ~caseTable(k).pass
                    passAll = false;
                    failBucket = "static";
                    break;
                end

                continue;
            end

            % -------------------------------------------------------------
            % Trim / stall / control-authority cases
            %
            % Required tail lift coefficient from pitch-moment balance:
            % 0 = CmCG + Cm_tail
            % Cm_tail = -(eta_t * Sht/S * lT/c) * CLh
            % => CLh_req = CmCG / [eta_t * Sht/S * lT/c]
            %
            % Then:
            % CLh = a_t * (alpha_t + tau * delta_e)
            % => delta_e_req = (CLh_req/a_t - alpha_t) / tau
            % -------------------------------------------------------------
            tailLeverTerm = aero.eta_t * (Sht / wing.S) * (lT / wing.cMGC);

            CLh_req = CmCG / max(tailLeverTerm, 1e-9);
            de_req  = (CLh_req / max(a_t, 1e-9) - alpha_t) / max(tauMax, 1e-9);

            % Equivalent tau required if delta_e limit is 25 deg
            tau_req = abs(CLh_req / max(a_t, 1e-9) - alpha_t) / deg2rad(25);
            maxTauReq = max(maxTauReq, tau_req);

            caseTable(k).name      = cases(k).name;
            caseTable(k).required  = rad2deg(abs(de_req));
            caseTable(k).available = 25.0;
            caseTable(k).pass      = (abs(de_req) <= deg2rad(25));

            if ~caseTable(k).pass
                passAll = false;

                switch k
                    case 1
                        failBucket = "stall";
                    case 2
                        failBucket = "trimFwd";
                    case 4
                        failBucket = "trimAft";
                    otherwise
                        failBucket = "trim";
                end

                break;
            end
        end

        % Tail stall margin
        alphaTailLimit = tailCfg.alpha_stall_HT - deg2rad(0.5);
        if passAll && (maxAlphaTail > alphaTailLimit)
            passAll = false;
            failBucket = "tailStall";
        end

        if ~passAll
            switch failBucket
                case "stall"
                    nFail_stall = nFail_stall + 1;
                case "trimFwd"
                    nFail_trimFwd = nFail_trimFwd + 1;
                case "rotation"
                    nFail_rotation = nFail_rotation + 1;
                case "trimAft"
                    nFail_trimAft = nFail_trimAft + 1;
                case "tailStall"
                    nFail_tailStall = nFail_tailStall + 1;
                case "static"
                    nFail_static = nFail_static + 1;
            end
            continue;
        end

        % Required elevator chord ratio
        cec_req = required_chord_ratio_from_tau(maxTauReq);
        if isnan(cec_req) || cec_req > tailCfg.cec_max
            nFail_cec = nFail_cec + 1;
            continue;
        end

        % Keep smallest feasible tail
        if Sht < bestS
            bestS = Sht;

            best = struct();
            best.S            = Sht;
            best.b            = geom.b;
            best.c_root       = geom.c_root;
            best.c_tip        = geom.c_tip;
            best.cMGC         = geom.cMGC;
            best.cec          = max(0.25, cec_req);
            best.tau          = maxTauReq;
            best.VHT          = VHT;
            best.caseTable    = caseTable;
            best.maxAlphaTail = maxAlphaTail;
            best.SM           = SM;
            best.pass         = true;
        end
    end

    if bestS < inf
        HT = best;
    else
        HT.pass = false;
        HT.failReason = 'No HT candidate satisfied all pitch / trim / rotation / stall constraints.';

        fprintf('\nHT summary at lT = %.3f m\n', lT);
        fprintf('  total checked      = %d\n', nTotal);
        fprintf('  fail stall         = %d\n', nFail_stall);
        fprintf('  fail trim fwd      = %d\n', nFail_trimFwd);
        fprintf('  fail rotation      = %d\n', nFail_rotation);
        fprintf('  fail trim aft      = %d\n', nFail_trimAft);
        fprintf('  fail tail stall    = %d\n', nFail_tailStall);
        fprintf('  fail static margin = %d\n', nFail_static);
        fprintf('  fail c_e/c         = %d\n', nFail_cec);
    end
end
function VT = size_vertical_tail(lT, class1, wing, tailCfg, massNow, CG, prop, perf, req, aero, side, opt, a_v)
    Sgrid = class1.Svt * opt.SvtSearchMult;

    tauMax = flap_effectiveness_from_chord_ratio(tailCfg.crc_max);
    VT.pass = false;
    bestS = inf;

    Wto = massNow.OWE + 12 * 8000 + 112000;
    Vs1 = sqrt(2 * Wto * 9.81 / (perf.rho_sl * wing.S * wing.CLmax_to));
    V2 = 1.20 * Vs1;

    rhoCruise = isa_density(perf.alt_cruise);
    aSound = isa_speed_of_sound(perf.alt_cruise);
    Vcr = perf.Mach_cruise * aSound;

    cases(1) = make_yaw_case('OEI take-off', perf.rho_sl, V2, prop.oneEngineOutThrust, req.beta_OEI_allow, true, false, false);
    cases(2) = make_yaw_case('Crosswind take-off/landing', perf.rho_sl, 1.15 * Vs1, 0.0, req.beta_crosswind, false, true, false);
    cases(3) = make_yaw_case('Minimum directional stability', rhoCruise, Vcr, 0.0, deg2rad(2), false, false, true);

    for i = 1:numel(Sgrid)
        Svt = Sgrid(i);
        VVT = lT * Svt / (wing.S * wing.b);
        geom = make_trapezoid(Svt, sqrt(tailCfg.AR_VT * Svt), tailCfg.lambdaVT);

        passAll = true;
        caseTable = repmat(struct('name','', 'required',0, 'available',0, 'pass',false), numel(cases), 1);
        maxTauReq = 0;

        for k = 1:numel(cases)
            q = 0.5 * cases(k).rho * cases(k).V^2;
            lv_b = lT / wing.b;
            sv_s = Svt / wing.S;

            Cn_beta_fin = aero.eta_v * a_v * (1 - aero.dsigma_dbeta) * sv_s * lv_b;
            Cn_beta_total = Cn_beta_fin + aero.CNb_fuse;
            Cn_dr_max = aero.eta_v * a_v * tauMax * sv_s * lv_b;

            if cases(k).isDirectionalStability
                caseTable(k).name = cases(k).name;
                caseTable(k).required = req.CNb_target;
                caseTable(k).available = Cn_beta_total;
                caseTable(k).pass = Cn_beta_total >= req.CNb_target;
                passAll = passAll && caseTable(k).pass;
                continue;
            end

            if cases(k).isOEI
                Cn_eng = prop.k_OEI * cases(k).Tfailed * prop.y_engine_outbd / max(q * wing.S * wing.b, 1e-6);
                beta_avail = cases(k).beta;
                Cn_avail = aero.eta_v * a_v * (beta_avail + tauMax * deg2rad(25)) * sv_s * lv_b;
                caseTable(k).name = cases(k).name;
                caseTable(k).required = Cn_eng;
                caseTable(k).available = Cn_avail;
                caseTable(k).pass = Cn_avail >= Cn_eng;
                passAll = passAll && caseTable(k).pass;
                continue;
            end

            if cases(k).isCrosswind
                xcg = wing.xLEMAC + CG.h_aft * wing.cMGC;
                l_side = abs(side.x_centroid - xcg);
                Cn_cross = (side.S_side / wing.S) * (l_side / wing.b) * aero.CYbeta_side * cases(k).beta;
                dr_req = abs(Cn_cross / max(Cn_dr_max, 1e-6));
                tau_req = abs(Cn_cross / max(aero.eta_v * a_v * sv_s * lv_b, 1e-6)) / deg2rad(25);
                maxTauReq = max(maxTauReq, tau_req);
                caseTable(k).name = cases(k).name;
                caseTable(k).required = rad2deg(dr_req);
                caseTable(k).available = 25.0;
                caseTable(k).pass = dr_req <= deg2rad(25);
                passAll = passAll && caseTable(k).pass;
                continue;
            end
        end

        if ~passAll
            continue;
        end

        crc_req = required_chord_ratio_from_tau(maxTauReq);
        if isnan(crc_req) || crc_req > tailCfg.crc_max
            continue;
        end

        if Svt < bestS
            bestS = Svt;
            best.S = Svt;
            best.b = geom.b;
            best.c_root = geom.c_root;
            best.c_tip = geom.c_tip;
            best.crc = max(0.25, crc_req);
            best.VVT = VVT;
            best.caseTable = caseTable;
            best.pass = true;
        end
    end

    if bestS < inf
        VT = best;
    else
        VT.pass = false;
    end
end

function masses = estimate_empennage_masses(HT, VT, massNow, ac)
    % Simplified Class II.5-like mass model.
    % Area terms capture skin / ribs / secondary structure.
    % Load terms scale with aircraft size and tail aspect ratio.
    Wref = max(massNow.OWE, 1.0);

    kHT = 18.5;
    kVT = 21.0;
    kE  = 7.0;
    kR  = 8.0;

    mHT = kHT * HT.S * (1 + 0.10 * abs(HT.VHT - 1.0)) * (Wref / 3e5)^0.12;
    mVT = kVT * VT.S * (1 + 0.20 * abs(VT.VVT - 0.09) / 0.09) * (Wref / 3e5)^0.12;
    mElev = kE * HT.cec * HT.S;
    mRudd = kR * VT.crc * VT.S;

    masses.mHT = mHT;
    masses.mVT = mVT;
    masses.mElev = mElev;
    masses.mRudd = mRudd;
    masses.total = mHT + mVT + mElev + mRudd;
end

function a = finite_wing_lift_curve_slope(AR, sweepLE_deg, Mach)
    beta = sqrt(max(1 - Mach^2, 0.2^2));
    sweep = deg2rad(sweepLE_deg);
    a0 = 2 * pi;
    a = a0 * AR / (2 + sqrt(4 + (AR * beta / 0.95)^2 * (1 + tan(sweep)^2 / beta^2)));
end

function out = make_pitch_case(name, rho, V, h, alpha, T, isRotation, isStaticMargin)
    out.name = name;
    out.rho = rho;
    out.V = V;
    out.h = h;
    out.alpha = alpha;
    out.T = T;
    out.isRotation = isRotation;
    out.isStaticMargin = isStaticMargin;

    % Default weights by case; overwritten where needed in caller.
    if contains(lower(name), 'stall') || contains(lower(name), 'low-speed')
        out.W = 255000 + 12 * 8000 + 28000;
    else
        out.W = 255000 + 12 * 8000 + 112000;
    end
end

function out = make_yaw_case(name, rho, V, Tfailed, beta, isOEI, isCrosswind, isDirectionalStability)
    out.name = name;
    out.rho = rho;
    out.V = V;
    out.Tfailed = Tfailed;
    out.beta = beta;
    out.isOEI = isOEI;
    out.isCrosswind = isCrosswind;
    out.isDirectionalStability = isDirectionalStability;
end

function CmT = thrust_pitching_moment(T, zThrust, zcg, q, S, cbar)
    CmT = T * (zThrust - zcg) / max(q * S * cbar, 1e-6);
end

function tau = flap_effectiveness_from_chord_ratio(cc)
    cc = max(min(cc, 0.70), 0.01);
    tau = 1.129 * cc^0.4044 - 0.1772;
    tau = max(tau, 0.0);
end

function cc = required_chord_ratio_from_tau(tauReq)
    if tauReq <= 0
        cc = 0.25;
        return;
    end
    cgrid = linspace(0.10, 0.60, 1000);
    tauGrid = arrayfun(@flap_effectiveness_from_chord_ratio, cgrid);
    idx = find(tauGrid >= tauReq, 1, 'first');
    if isempty(idx)
        cc = NaN;
    else
        cc = cgrid(idx);
    end
end

function rho = isa_density(h)
    T0 = 288.15;
    p0 = 101325;
    L = 0.0065;
    R = 287.05287;
    g = 9.80665;

    if h <= 11000
        T = T0 - L * h;
        p = p0 * (T / T0)^(g / (R * L));
    else
        T = 216.65;
        p11 = p0 * (T / T0)^(g / (R * L));
        p = p11 * exp(-g * (h - 11000) / (R * T));
    end
    rho = p / (R * T);
end

function a = isa_speed_of_sound(h)
    gamma = 1.4;
    R = 287.05287;
    if h <= 11000
        T = 288.15 - 0.0065 * h;
    else
        T = 216.65;
    end
    a = sqrt(gamma * R * T);
end

function T = make_trapezoid(S, b, lambda)
    T.S = S;
    T.b = b;
    T.lambda = lambda;
    T.c_root = 2 * S / (b * (1 + lambda));
    T.c_tip = lambda * T.c_root;
end

function cbar = mean_geom_chord(c_root, lambda)
    cbar = (2/3) * c_root * ((1 + lambda + lambda^2) / (1 + lambda));
end

function ybar = y_mgc(b, lambda)
    ybar = (b/6) * ((1 + 2 * lambda) / (1 + lambda));
end

function [xpoly, ypoly] = tail_planform_polygon(x_apex, c_root, c_tip, b, sweepLE_deg)
    ysemi = b / 2;
    xLE_tip = x_apex + ysemi * tand(sweepLE_deg);
    xpoly = [x_apex, xLE_tip, xLE_tip + c_tip, x_apex + c_root, x_apex + c_root, xLE_tip + c_tip, xLE_tip, x_apex, x_apex];
    ypoly = [0, ysemi, ysemi, 0, 0, -ysemi, -ysemi, 0, 0];
end

function [xpoly, zpoly] = fin_planform_polygon(x_apex, c_root, c_tip, h, sweepLE_deg)
    xLE_tip = x_apex + h * tand(sweepLE_deg);
    xpoly = [x_apex, xLE_tip, xLE_tip + c_tip, x_apex + c_root, x_apex];
    zpoly = [0, h, h, 0, 0];
end

function [Wvec, xcgvec] = loading_path(OWE, xOWE, m_each, x_payload, m_fuel, x_fuel)
    W = OWE;
    M = OWE * xOWE;
    Wvec = W;
    xcgvec = M / W;

    for i = 1:numel(x_payload)
        W = W + m_each;
        M = M + m_each * x_payload(i);
        Wvec(end+1,1) = W; %#ok<AGROW>
        xcgvec(end+1,1) = M / W; %#ok<AGROW>
    end

    W = W + m_fuel;
    M = M + m_fuel * x_fuel;
    Wvec(end+1,1) = W;
    xcgvec(end+1,1) = M / W;
end

function pct = x_to_pctMAC(x, xLEMAC, cMGC)
    pct = 100 * (x - xLEMAC) / cMGC;
end

function print_case_summary(caseTable)
    for i = 1:numel(caseTable)
        if isfield(caseTable(i), 'name') && ~isempty(caseTable(i).name)
            fprintf('  %-34s | required = %10.4g | available = %10.4g | %s\n', ...
                caseTable(i).name, caseTable(i).required, caseTable(i).available, tf(caseTable(i).pass));
        end
    end
end

function s = tf(flag)
    if flag
        s = 'PASS';
    else
        s = 'FAIL';
    end
end
