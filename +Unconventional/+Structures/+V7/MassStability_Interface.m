%% MassStability_Interface — Wing Structures outputs for Role 5
% ── Wing mass contribution to OEM breakdown ───────────────────────────
m_wing_Al  = WS.m_wing_total;     % kg  Al 7075-T6  — use for OEM tracking
m_wing_CF  = WS.m_wing_CF;        % kg  CFRP        — comparison
m_primary  = WS.m_wing_primary;   % kg  box primary only
m_sec      = WS.m_secondary;      % kg  secondary structure
m_hinge    = WS.m_hinge;          % kg  fold hinge mechanism

% ── Centre-of-mass envelope ───────────────────────────────────────────
y_CG_wing  = WS.y_CG_wing;        % m   wing spanwise CG from CL
%  Add to CoM balance with fuselage, payload, fuel, gear contributions

% ── Roll moment of inertia ────────────────────────────────────────────
I_roll     = WS.I_roll_wing;      % kg·m²  both semi-wings
%  Sum with fuselage, fuel, payload inertia for full aircraft I_roll

% ── Geometry for CG range analysis ───────────────────────────────────
MAC        = WS.MAC;              % m   mean aerodynamic chord (CG % MAC ref)
c_root     = WS.c_root;          % m
y_hinge    = WS.y_hinge;         % m   fold hinge spanwise position

% ── Print summary ─────────────────────────────────────────────────────
fprintf('\n--- Wing Structures → Mass & Stability handoff ---\n');
fprintf('  Wing mass (Al, 2.5g)         %8.0f kg  (%.2f%% MTOM)\n', m_wing_Al, WS.m_frac_MTOM*100);
fprintf('  Wing mass (CF, 2.5g)         %8.0f kg\n', m_wing_CF);
fprintf('  Primary / Secondary / Hinge  %6.0f / %6.0f / %6.0f kg\n', m_primary, m_sec, m_hinge);
fprintf('  Spanwise CG                  %8.2f m from CL\n', y_CG_wing);
fprintf('  I_roll (both wings)          %.4e kg·m²\n', I_roll);
fprintf('  MAC                          %8.2f m\n', MAC);
