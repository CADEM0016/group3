function E = EmpiricalMass(p, material)

arguments
    p        struct
    material string = 'Al'
end

MTOM   = double(p.MTOM);
b      = double(p.Span);
S_wing = double(p.WingArea);
AR     = double(p.AR);
lam    = double(p.lambda);
tc     = double(p.tc_root);
mf     = double(p.Mf_fuel);
n_ult  = double(p.n_ult_pos);
sw_c4  = deg2rad(double(p.sweep_c4_deg));
sw_c2  = deg2rad(double(p.sweep_c2_deg));

kg_to_lb  = 2.20462;
m2_to_ft2 = 10.7639;
Pa_to_psf = 0.020885;

% =========================================================================
%  METHOD 1 — RAYMER (transport variant, Eq 15.25, US customary)
%  W_W = 0.0051*(W_dg*Nz)^0.557 * Sw^0.649 * AR^0.5 * (t/c)^-0.4
%        * (1+lambda)^0.1 * (1/cos(sweep_c2)) * Scsw^0.1
% =========================================================================
W_dg_lb   = MTOM   * kg_to_lb;
S_w_ft2   = S_wing * m2_to_ft2;
S_csw_ft2 = 0.15   * S_w_ft2;

m_raymer = 0.0051 ...
    * (W_dg_lb * n_ult)^0.557 ...
    * S_w_ft2^0.649 ...
    * AR^0.5 ...
    * tc^(-0.4) ...
    * (1 + lam)^0.1 ...
    * (1 / cos(sw_c2)) ...
    * S_csw_ft2^0.1;
m_raymer = m_raymer / kg_to_lb;

% =========================================================================
%  METHOD 2 — TORENBEEK (Snorri Ch6, SI)
%  m_W = 0.00125*W0*(b/cos(sweep_c2))^0.75 * [1+sqrt(6.3*cos(sweep)/b)]
%        * Nz^0.55 * [b*S/(t_max*W0*cos(sweep))]^0.30
%  denominator uses m_MZF for physically correct inertia relief
% =========================================================================
m_MZF       = MTOM * (1 - mf);
t_max_root  = tc * p.c_root;
inner_tb    = (b * S_wing) / (t_max_root * m_MZF * cos(sw_c2));
bracket_tb  = 1 + sqrt(6.3 * cos(sw_c2) / b);

m_torenbeek = 0.00125 * MTOM ...
    * (b / cos(sw_c2))^0.75 ...
    * bracket_tb ...
    * n_ult^0.55 ...
    * inner_tb^0.30;

% =========================================================================
%  METHOD 3 — USAF (Snorri Ch6, US customary)
%  W_W = 96.948 * (Nz*W0/1e5)^0.65 * (AR/cos^2(sweep_c4))^0.57
%        * (S/100)^0.61 * ((1+lambda)/(2*t/c))^0.36
%        * (1 + sqrt(V_H/500))^0.993
%  V_H in knots (max level flight speed)
% =========================================================================
V_cruise_kts = p.V_cruise / 0.5144;       % m/s to knots
S_w_ft2_usaf = S_wing * m2_to_ft2;

m_usaf = 96.948 ...
    * (n_ult * W_dg_lb / 1e5)^0.65 ...
    * (AR / cos(sw_c4)^2)^0.57 ...
    * (S_w_ft2_usaf / 100)^0.61 ...
    * ((1 + lam) / (2 * tc))^0.36 ...
    * (1 + sqrt(V_cruise_kts / 500))^0.993;
m_usaf = m_usaf / kg_to_lb;

% ---- Average all three methods
m_avg = (m_raymer + m_torenbeek + m_usaf) / 3.0;

% =========================================================================
%  CFRP MATERIAL CORRECTION (specific strength ratio)
%  m_CF = m_Al * (sigma_all_Al / rho_Al) / (sigma_all_CF / rho_CF)
% =========================================================================
if strcmpi(material, 'CF')
    ssp_Al   = p.Al.sig_all / p.Al.rho;
    ssp_CF   = p.CF.sig_all / p.CF.rho;
    m_avg    = m_avg * (ssp_Al / ssp_CF);
    mat_name = 'CFRP';
else
    mat_name = 'Aluminium 7075-T6';
end

% =========================================================================
%  FOLDING HINGE PENALTY
%  outer panel span = semi-span - y_hinge = 36 - 32.5 = 3.5 m
%  penalty = f_hinge * (f_outer * m_avg * 0.40)
%  floor at p.m_hinge_min (1500 kg) from 777X programme data
% =========================================================================
b_semi  = p.Span / 2;
b_outer = b_semi - p.y_hinge;
f_outer = b_outer / b_semi;
m_outer = m_avg * f_outer * 0.40;
m_hinge = p.f_hinge_mech * m_outer;
m_hinge = max(m_hinge, p.m_hinge_min);

m_total = m_avg + m_hinge;

% ---- Output
E.m_raymer      = m_raymer;
E.m_torenbeek   = m_torenbeek;
E.m_usaf        = m_usaf;
E.m_primary_avg = m_avg;
E.m_hinge       = m_hinge;
E.m_total       = m_total;
E.m_frac_MTOM   = m_total / MTOM;
E.material      = upper(material);
E.mat_name      = mat_name;

fprintf('\n--- Class I/II Empirical Wing Mass: %s ---\n', mat_name);
fprintf('  Raymer   (2018):   %7.0f kg  (%4.1f%% MTOM)\n', m_raymer,    m_raymer/MTOM*100);
fprintf('  Torenbeek(2013):   %7.0f kg  (%4.1f%% MTOM)\n', m_torenbeek, m_torenbeek/MTOM*100);
fprintf('  USAF:              %7.0f kg  (%4.1f%% MTOM)\n', m_usaf,      m_usaf/MTOM*100);
fprintf('  Three-method avg:  %7.0f kg\n', m_avg);
fprintf('  Hinge penalty:     %7.0f kg\n', m_hinge);
fprintf('  TOTAL:             %7.0f kg  (%4.1f%% MTOM)\n', m_total, m_total/MTOM*100);
fprintf('  [B777F reference:  ~34,000 kg  (~9.8%% MTOM)]\n');
fprintf('  Validation vs B777F: %+.1f%%\n', (m_total-34000)/34000*100);

end
