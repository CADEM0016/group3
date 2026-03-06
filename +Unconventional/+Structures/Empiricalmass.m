function E = EmpiricalMass(p, material)
% =========================================================================
% +Structures/EmpiricalMass.m
% Class I/II empirical wing mass: Raymer (2018) + Torenbeek (2013).
% Call as:  E = Unconventional.Structures.EmpiricalMass(p)
%           E = Unconventional.Structures.EmpiricalMass(p, 'CF')
%
% FIX: Torenbeek was returning 0 due to operand order in inner_term.
%      inner_term = b*S / (m_MZF * tc) — all scalar doubles, now explicit.
% =========================================================================

arguments
    p        struct
    material string = 'Al'
end

MTOM  = double(p.MTOM);
b     = double(p.Span);
S_wing = double(p.WingArea);
AR    = double(p.AR);
lam   = double(p.lambda);
tc    = double(p.tc_root);
mf    = double(p.Mf_fuel);
n_ult = double(p.n_ult_pos);
sw    = deg2rad(double(p.sweep_c2_deg));

% ---- Raymer (2018) Eq 15.25 SI-adapted
S_csw    = 0.15 * S_wing;
m_raymer = 0.0051 ...
         * (MTOM * n_ult)^0.557 ...
         * S_wing^0.649 ...
         * AR^0.5 ...
         * tc^(-0.4) ...
         * (1 + lam)^0.1 ...
         / cos(sw) ...
         * S_csw^0.1;

% ---- Torenbeek (2013) Eq 8.27
% m_w = A_w * b^0.75 * [1 + sqrt(6.3*cos(sw)/b)] * n_ult^0.55
%       * [b*S/(m_MZF*tc)]^0.30
% A_w = 4.58e-3 (Torenbeek Table 8-4, metal structure)
A_w   = 4.58e-3;
m_MZF = MTOM * (1 - mf);           % max zero-fuel mass [kg]

% Explicit scalar computation — avoids any struct field precedence issue

inner_term = (b * S_wing) / (m_MZF * tc);

bracket1    = 1 + sqrt(6.3 * cos(sw) / b);

m_torenbeek = A_w ...
            * (b^0.75) ...
            * bracket1 ...
            * (n_ult^0.55) ...
            * (inner_term^0.30);

% ---- Average
m_avg = (m_raymer + m_torenbeek) / 2;

% ---- Material correction (CFRP)
if strcmpi(material, 'CF')
    ssp_Al = p.Al.sig_all / p.Al.rho;
    ssp_CF = p.CF.sig_all / p.CF.rho;
    m_avg  = m_avg * (ssp_Al / ssp_CF);
    mat_name = 'CFRP';
else
    mat_name = 'Aluminium 7075-T6';
end

% ---- Hinge penalty
b_outer = p.Span/2 - p.y_hinge;
m_outer = m_avg * (b_outer / (p.Span/2)) * 0.40;
m_hinge = p.f_hinge_mech * m_outer;
m_total = m_avg + m_hinge;

% ---- Pack
E.m_raymer      = m_raymer;
E.m_torenbeek   = m_torenbeek;
E.m_primary_avg = m_avg;
E.m_hinge       = m_hinge;
E.m_total       = m_total;
E.m_frac_MTOM   = m_total / MTOM;
E.material      = upper(material);
E.mat_name      = mat_name;

fprintf('\n--- Class I/II Empirical Wing Mass: %s ---\n', mat_name);
fprintf('  Raymer (2018):          %7.0f kg  (%4.1f%% MTOM)\n', m_raymer,    m_raymer/MTOM*100);
fprintf('  Torenbeek (2013):       %7.0f kg  (%4.1f%% MTOM)\n', m_torenbeek, m_torenbeek/MTOM*100);
fprintf('  Average (primary):      %7.0f kg\n', m_avg);
fprintf('  Hinge penalty:          %7.0f kg\n', m_hinge);
fprintf('  TOTAL:                  %7.0f kg  (%4.1f%% MTOM)\n', m_total, m_total/MTOM*100);
fprintf('  [B777F reference: ~34,000 kg (~9.8%% MTOM)]\n');
fprintf('  Validation error: %+.1f%%\n', (m_total - 34000)/34000*100);

end