function FT = FoldingWingtip(loc, G, S_25g, S_1g, S_n1g, W_25g, D_25g, MB_25g)
% Folding wingtip structural analysis i.e hinge pin, rib, and outer panel sizing.

ih  = G.i_hinge;
N   = G.N;
dy  = G.dy;

% Geometry at hinge station
h_h  = G.h_wb(ih);          % m   wingbox height at hinge
w_h  = G.w_wb(ih);          % m   wingbox width at hinge
Ae_h = G.A_enc(ih);         % m²  enclosed area at hinge
c_h  = G.chord(ih);         % m   chord at hinge station
y_h  = G.y_hinge;           % m   hinge position from CL

% Hinge loads — 2.5g governs flight; 1.5g governs fold
Q_h  = abs(S_25g.Q_hinge);  % N
M_h  = abs(S_25g.M_hinge);  % Nm
T_h  = abs(S_25g.T_hinge);  % Nm

Q_h_fold = abs(S_1g.Q_hinge) * loc.n_limit_fold;   % N   fold load case
M_h_fold = abs(S_1g.M_hinge) * loc.n_limit_fold;   % Nm

% Hinge pin sizing — double-shear, titanium
% Shear in pin: V_pin = Q_hinge / 2 (double shear)
% A_pin = V_pin / tau_all   =>   d_pin from A_pin = pi*d²/4
V_pin  = Q_h / 2;
A_pin  = V_pin / loc.hinge_tau;
d_pin  = sqrt(4 * A_pin / pi);

% Bearing check: sigma_bearing = Q_h / (d_pin * t_bearing)
% t_bearing ≥ Q_h / (d_pin * hinge_sig)
t_bearing = Q_h / (d_pin * loc.hinge_sig);
t_bearing = max(t_bearing, loc.hinge_t_min);

% Hinge rib sizing - carries bending moment as a torque box
% Rib acts as a shear frame: Q_rib = M_hinge / w_wb (moment resolved to
% shear couple across rib depth h_wb)
Q_rib    = M_h / w_h;
t_rib    = 1.5 * Q_rib / (h_h * loc.Al.tau_all);
t_rib    = max(t_rib, loc.Al.t_min);
m_rib    = loc.Al.rho * 2 * (h_h * w_h) * t_rib;   % kg  top+bottom rib plates

% Outer panel mass breakdown
% Stations 1..ih are tip→hinge (outer panel, index 1 = tip)
m_skin_outer = 2 * sum(MB_25g.m_skin_dist(1:ih));
m_cap_outer  = 2 * sum(MB_25g.m_total_dist(1:ih)) - m_skin_outer;
m_outer_prim = 2 * sum(MB_25g.m_total_dist(1:ih));

m_lock       = loc.f_lock_mech  * m_outer_prim;   % kg  lock mechanism
m_actuator   = loc.f_actuator   * m_outer_prim;   % kg  fold actuator
m_hinge_assy = max(loc.f_hinge_mech * m_outer_prim, loc.m_hinge_min);

m_outer_total = m_outer_prim + m_lock + m_actuator + m_hinge_assy;

% Stiffness discontinuity at hinge
% EI and GJ ratio inner/outer panel at hinge station
EI_inner = D_25g.EI(ih + 1);   % Nm²  just inboard of hinge
EI_outer = D_25g.EI(ih);       % Nm²  just outboard of hinge
GJ_inner = D_25g.GJ(ih + 1);
GJ_outer = D_25g.GJ(ih);

EI_ratio = EI_inner / max(EI_outer, 1e3);
GJ_ratio = GJ_inner / max(GJ_outer, 1e3);

% Tip deflection estimate (cantilever, 2.5g)
% delta_tip = integral of M/(EI) dz over outer panel, trapezoidal
% Outer panel runs stations 1..ih (tip to hinge)
delta_tip = 0;
for i = 1 : ih - 1
    M_avg  = 0.5 * (abs(S_25g.M(i)) + abs(S_25g.M(i+1)));
    EI_avg = 0.5 * (D_25g.EI(i)    + D_25g.EI(i+1));
    if EI_avg > 1e3
        delta_tip = delta_tip + M_avg / EI_avg * dy;
    end
end
% outer panel span
b_tip = loc.b_tip_fold;

% Folded position check — Code E compliance
span_taxi_check = 2 * y_h;   % m   folded span = 2 × hinge position
code_E_margin   = 65.0 - span_taxi_check;   % m   positive = compliant

% Weight penalty vs fixed wing
m_fixed_equiv = MB_25g.m_outer_primary;   % outer panel primary if fixed
m_fold_penalty = m_hinge_assy + m_lock + m_actuator + m_rib;   % kg

% Package outputs
FT.y_hinge        = y_h;
FT.b_tip          = b_tip;
FT.span_taxi      = span_taxi_check;
FT.code_E_margin  = code_E_margin;

FT.Q_hinge_25g    = Q_h;
FT.M_hinge_25g    = M_h;
FT.T_hinge_25g    = T_h;
FT.Q_hinge_fold   = Q_h_fold;
FT.M_hinge_fold   = M_h_fold;

FT.d_pin          = d_pin;
FT.t_bearing      = t_bearing;
FT.t_rib          = t_rib;
FT.m_rib          = m_rib;

FT.m_outer_prim   = m_outer_prim;
FT.m_lock         = m_lock;
FT.m_actuator     = m_actuator;
FT.m_hinge_assy   = m_hinge_assy;
FT.m_outer_total  = m_outer_total;
FT.m_fold_penalty = m_fold_penalty;

FT.EI_inner       = EI_inner;
FT.EI_outer       = EI_outer;
FT.GJ_inner       = GJ_inner;
FT.GJ_outer       = GJ_outer;
FT.EI_ratio       = EI_ratio;
FT.GJ_ratio       = GJ_ratio;

FT.delta_tip_25g  = delta_tip;

fprintf('\n--- Folding Wingtip Analysis ---\n');
fprintf('  Hinge position         y = %.2f m from CL\n',   y_h);
fprintf('  Outer panel span       %.3f m each side\n',     b_tip);
fprintf('  Folded taxi span       %.2f m  (Code E margin %.2f m)\n', span_taxi_check, code_E_margin);

fprintf('\n  Hinge loads (2.5g flight):\n');
fprintf('    Q_hinge  %8.3f MN\n',  Q_h/1e6);
fprintf('    M_hinge  %8.3f MNm\n', M_h/1e6);
fprintf('    T_hinge  %8.3f MNm\n', T_h/1e6);
fprintf('  Hinge loads (fold case  n=%.1f):\n', loc.n_limit_fold);
fprintf('    Q_hinge  %8.3f MN\n',  Q_h_fold/1e6);
fprintf('    M_hinge  %8.3f MNm\n', M_h_fold/1e6);

fprintf('\n  Hinge pin (Ti, double-shear):\n');
fprintf('    Diameter           %.1f mm\n',  d_pin*1e3);
fprintf('    Bearing thickness  %.1f mm\n',  t_bearing*1e3);
fprintf('  Hinge rib thickness  %.2f mm\n',  t_rib*1e3);
fprintf('  Hinge rib mass       %.0f kg\n',  m_rib);

fprintf('\n  Outer panel mass breakdown:\n');
fprintf('    Primary (box)      %6.0f kg\n', m_outer_prim);
fprintf('    Lock mechanism     %6.0f kg  (%.0f%% outer prim)\n', m_lock,     loc.f_lock_mech*100);
fprintf('    Fold actuator      %6.0f kg  (%.0f%% outer prim)\n', m_actuator, loc.f_actuator*100);
fprintf('    Hinge assembly     %6.0f kg  (%.0f%% outer prim)\n', m_hinge_assy, loc.f_hinge_mech*100);
fprintf('    OUTER TOTAL        %6.0f kg\n', m_outer_total);
fprintf('  Fold mechanism penalty vs fixed wing  %+.0f kg\n', m_fold_penalty);

fprintf('\n  Stiffness discontinuity at hinge:\n');
fprintf('    EI inner / outer   %.3e / %.3e  (ratio %.1f×)\n', EI_inner, EI_outer, EI_ratio);
fprintf('    GJ inner / outer   %.3e / %.3e  (ratio %.1f×)\n', GJ_inner, GJ_outer, GJ_ratio);

fprintf('\n  Outer panel tip deflection (2.5g)  %.3f m\n', delta_tip);

if code_E_margin >= 0
    fprintf('  Code E compliance   PASS  (%.2f m margin)\n', code_E_margin);
else
    fprintf('  Code E compliance   FAIL  (%.2f m over)\n', abs(code_E_margin));
end

end
