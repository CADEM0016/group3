function E = EmpiricalMass(adp, tlar, loc, G, material)

arguments
    adp
    tlar
    loc      struct
    G        struct
    material string = 'Al'
end

% From adp directly
MTOM   = double(adp.MTOM);
b      = double(adp.Span);
S_wing = double(adp.WingArea);
mf     = double(adp.Mf_Fuel);

n_ult = loc.n_limit_pos * loc.SF;
if ismethod(adp, 'AR')
    AR = double(adp.AR());     % fetched from ADP department model
else
    AR = G.AR;                 % fallback from structures geometry output
end
lam   = loc.lambda;
tc    = loc.tc_root;
sw_c4 = deg2rad(loc.sweep_c4_deg);
sw_c2 = deg2rad(loc.sweep_c2_deg);

% From tlar and cast.atmos directly
[~, a_cr] = cast.atmos(tlar.Alt_cruise);
V_cruise  = tlar.M_c * a_cr;

% From SI directly
kg_to_lb  = SI.lb;
m2_to_ft2 = SI.ft^2;

% Raymer 2018 Eq 15.25  (US customary)
W_dg_lb   = MTOM * kg_to_lb;
S_w_ft2   = S_wing * m2_to_ft2;
S_csw_ft2 = 0.15 * S_w_ft2;   % control surface area assumed 15% of wing

m_raymer = 0.0051               ...
    * (W_dg_lb * n_ult)^0.557   ...
    * S_w_ft2^0.649             ...
    * AR^0.5                    ...
    * tc^(-0.4)                 ...
    * (1 + lam)^0.1             ...
    * (1 / cos(sw_c2))          ...
    * S_csw_ft2^0.1             ...
    / kg_to_lb;

% Torenbeek 2013 Eq 8.27  (SI)
m_MZF      = MTOM * (1 - mf);
t_max_root = tc * G.c_root;
inner_tb   = (b * S_wing) / (t_max_root * m_MZF * cos(sw_c2));
bracket_tb = 1 + sqrt(6.3 * cos(sw_c2) / b);

m_torenbeek = 0.00125 * MTOM      ...
    * (b / cos(sw_c2))^0.75       ...
    * bracket_tb                  ...
    * n_ult^0.55                  ...
    * inner_tb^0.30;

% USAF correlation  (V in knots)
V_kts = V_cruise / 0.5144;

m_usaf = 96.948                       ...
    * (n_ult * W_dg_lb / 1e5)^0.65   ...
    * (AR / cos(sw_c4)^2)^0.57       ...
    * (S_w_ft2 / 100)^0.61           ...
    * ((1 + lam) / (2 * tc))^0.36    ...
    * (1 + sqrt(V_kts / 500))^0.993  ...
    / kg_to_lb;

m_avg = (m_raymer + m_torenbeek + m_usaf) / 3;

% CFRP correction — scale by specific-strength ratio
if strcmpi(material, 'CF')
    m_avg    = m_avg * (loc.Al.sig_all / loc.Al.rho) / (loc.CF.sig_all / loc.CF.rho);
    mat_name = 'CFRP';
else
    mat_name = 'Aluminium 7075-T6';
end

b_outer = b/2 - loc.y_hinge;
m_hinge = loc.f_hinge_mech * (m_avg * (b_outer / (b/2)) * 0.40);
m_hinge = max(m_hinge, loc.m_hinge_min);
m_total = m_avg + m_hinge;

E.m_raymer      = m_raymer;
E.m_torenbeek   = m_torenbeek;
E.m_usaf        = m_usaf;
E.m_primary_avg = m_avg;
E.m_hinge       = m_hinge;
E.m_total       = m_total;
E.m_frac_MTOM   = m_total / MTOM;
E.material      = upper(material);
E.mat_name      = mat_name;

fprintf('\n--- Class I/II Wing Mass: %s ---\n', mat_name);
fprintf('  Raymer    2018   %7.0f kg  (%4.1f%% MTOM)\n', m_raymer,    m_raymer/MTOM*100);
fprintf('  Torenbeek 2013   %7.0f kg  (%4.1f%% MTOM)\n', m_torenbeek, m_torenbeek/MTOM*100);
fprintf('  USAF             %7.0f kg  (%4.1f%% MTOM)\n', m_usaf,      m_usaf/MTOM*100);
fprintf('  3-method avg     %7.0f kg\n', m_avg);
fprintf('  Hinge penalty    %7.0f kg\n', m_hinge);
fprintf('  TOTAL            %7.0f kg  (%4.1f%% MTOM)\n', m_total, m_total/MTOM*100);
fprintf('  vs B777F 34,000  %+.1f%%\n', (m_total - 34000)/34000*100);

end
