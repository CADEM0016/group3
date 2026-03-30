function WS = WingStructuresExport(G, S_25g, S_1g, S_n1g, W_25g, W_25g_CF, D_25g, D_25g_CF, MB_25g, MB_25g_CF, E_Al, E_CF)
% No physics computed here — all values copied from pipeline structs.

% Mass & Stability
WS.m_wing_total   = MB_25g.m_total;
WS.m_wing_CF      = MB_25g_CF.m_total;
WS.m_wing_primary = MB_25g.m_primary;
WS.m_secondary    = MB_25g.m_secondary;
WS.m_hinge        = MB_25g.m_hinge;
WS.m_frac_MTOM    = MB_25g.m_frac_MTOM;
WS.y_CG_wing      = MB_25g.y_CG_wing;      % m   spanwise wing CG from CL
WS.I_roll_wing    = MB_25g.I_roll_wing;    % kg·m²  both semi-wings

% Aerodynamics
WS.e_oswald       = G.e_oswald;
WS.AR             = G.AR;
WS.MAC            = G.MAC;
WS.y              = G.y;
WS.chord          = G.chord;
WS.x_ac           = G.x_ac;
WS.x_fa           = G.x_fa;
WS.e_ac_fa        = G.e_ac_fa;

% Fuselage Structures — root and hinge loads
WS.Q_root         = S_25g.Q_root;
WS.M_root         = S_25g.M_root;
WS.T_root         = S_25g.T_root;
WS.Q_root_n1g     = S_n1g.Q_root;
WS.M_root_n1g     = S_n1g.M_root;
WS.Q_hinge        = S_25g.Q_hinge;
WS.M_hinge        = S_25g.M_hinge;
WS.T_hinge        = S_25g.T_hinge;

% Aeroelastics — stiffness distributions and key station scalars
WS.EI             = D_25g.EI;
WS.GJ             = D_25g.GJ;
WS.EI_CF          = D_25g_CF.EI;
WS.GJ_CF          = D_25g_CF.GJ;
WS.EI_root        = D_25g.EI_root;
WS.GJ_root        = D_25g.GJ_root;
WS.EI_hinge       = D_25g.EI_hinge;
WS.GJ_hinge       = D_25g.GJ_hinge;
WS.EI_tip         = D_25g.EI_tip;
WS.GJ_tip         = D_25g.GJ_tip;
WS.GJ_over_EI     = D_25g.GJ_root / D_25g.EI_root;

% Optimisation / FEDR — fidelity ladder values
WS.m_ClassI_II_Al = E_Al.m_total;
WS.m_ClassI_II_CF = E_CF.m_total;
WS.m_raymer       = E_Al.m_raymer;
WS.m_torenbeek    = E_Al.m_torenbeek;
WS.m_usaf         = E_Al.m_usaf;

% Geometry summary
WS.Span           = G.Span;
WS.s              = G.s;
WS.c_root         = G.c_root;
WS.c_tip          = G.c_tip;
WS.y_hinge        = G.y_hinge;
WS.y_engine       = G.y_engine;

fprintf('\n--- Wing Structures Handoff ---\n');
fprintf('  [Mass & Stability]\n');
fprintf('  Wing mass (Al)    %7.0f kg  (%.2f%% MTOM)\n', WS.m_wing_total, WS.m_frac_MTOM*100);
fprintf('  Wing mass (CF)    %7.0f kg\n',                 WS.m_wing_CF);
fprintf('  Primary/Sec/Hinge %6.0f / %6.0f / %6.0f kg\n', WS.m_wing_primary, WS.m_secondary, WS.m_hinge);
fprintf('  Wing CG           %7.2f m from CL\n',          WS.y_CG_wing);
fprintf('  I_roll            %.4e kg·m²\n',                WS.I_roll_wing);
fprintf('  [Aerodynamics]\n');
fprintf('  Oswald e          %7.4f\n',                     WS.e_oswald);
fprintf('  AR / MAC          %7.2f  /  %.2f m\n',          WS.AR, WS.MAC);
fprintf('  GJ/EI (root)      %7.4f\n',                     WS.GJ_over_EI);
fprintf('  [Fuselage Structures — 2.5g root]\n');
fprintf('  Q / M / T   %.3f MN  /  %.3f MNm  /  %.3f MNm\n', ...
    abs(WS.Q_root)/1e6, abs(WS.M_root)/1e6, abs(WS.T_root)/1e6);
fprintf('  [Aeroelastics]\n');
fprintf('  EI  root/hinge/tip  %.3e / %.3e / %.3e Nm²\n', WS.EI_root, WS.EI_hinge, WS.EI_tip);
fprintf('  GJ  root/hinge/tip  %.3e / %.3e / %.3e Nm²\n', WS.GJ_root, WS.GJ_hinge, WS.GJ_tip);

end
