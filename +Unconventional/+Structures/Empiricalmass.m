function E = EmpiricalMass(p, material)
% =========================================================================
% EmpiricalMass.m  —  +Structures package
% Class I / Class II empirical wing mass estimation.
%
% Two independent methods — averaged for the MDO loop estimate:
%   1. Raymer (2018) Eq 15.25  — transport aircraft correlation
%   2. Torenbeek (2013) Eq 8.27 — high-speed transport correlation
%
% Also computes material correction factor for CFRP.
% Also computes hinge mechanism mass penalty for folding wingtip.
%
% Use this function:
%   (a) As the initial/fast estimate inside the MDO convergence loop
%   (b) As the Class I/II reference for the fidelity comparison table
%   (c) To validate your II.5 model against B777F reference
%
% INPUT:
%   p        — AircraftParams struct (from AircraftParams.m)
%   material — string: 'Al' (default) | 'CF'
%
% OUTPUT:  E — struct
%   E.m_raymer      [kg]  Raymer estimate (primary, no secondary/hinge)
%   E.m_torenbeek   [kg]  Torenbeek estimate
%   E.m_primary_avg [kg]  Average of both
%   E.m_hinge       [kg]  Folding hinge mechanism penalty
%   E.m_total       [kg]  Total = primary_avg + hinge
%   E.m_frac_MTOM   [-]   Total / MTOM
%   E.material      string
%
% NO external dependencies.
% =========================================================================

arguments
    p        struct
    material string = 'Al'
end

MTOM     = p.MTOM;
b        = p.Span;
S        = p.WingArea;
AR       = p.AR;
lambda   = p.lambda;
tc       = p.tc_root;
mf_fuel  = p.Mf_fuel;
M_c      = p.M_cruise;
y_hinge  = p.y_hinge;
g        = p.g;

sweep_c2 = deg2rad(p.sweep_c2_deg);
n_ult    = p.n_ult_pos;

% -------------------------------------------------------------------------
%  1.  RAYMER (2018) Eq 15.25  — transport aircraft wing mass
%
%  Original (SI-adapted):
%    W_w = 0.0051 * (W_dg * N_z)^0.557 * S_w^0.649 * A^0.5
%          * (t/c)^-0.4 * (1+λ)^0.1 * cos(Λ_c/2)^-1 * S_csw^0.1
%
%  Where:
%    W_dg  = design gross weight [kg]  (= MTOM)
%    N_z   = ultimate load factor      (= n_ult_pos = 3.75)
%    S_w   = wing area [m^2]
%    A     = aspect ratio
%    t/c   = root thickness ratio
%    λ     = taper ratio
%    Λ_c/2 = sweep at half-chord [rad]
%    S_csw = control surface area [m^2]  (≈ 15% of S_w)
%
%  Note: Raymer's original equation uses lb and ft.
%        This SI version is adapted via unit analysis — see Raymer §15.3.
% -------------------------------------------------------------------------
S_csw   = 0.15 * S;                    % control surface area [m^2]

m_raymer = 0.0051 ...
         * (MTOM * n_ult)^0.557 ...
         * S^0.649 ...
         * AR^0.5 ...
         * tc^(-0.4) ...
         * (1 + lambda)^0.1 ...
         / cos(sweep_c2) ...
         * S_csw^0.1;

% -------------------------------------------------------------------------
%  2.  TORENBEEK (2013) Eq 8.27  — high-speed transport wing mass
%
%    m_w = A_w * b^0.75 * [1 + sqrt(6.3*cos(Λ)/b)]
%          * n_ult^0.55 * [b*S / (m_MZF * t/c)]^0.30
%
%  Where:
%    A_w   = 4.58e-3 (Torenbeek Table 8-4, metal structure)
%    m_MZF = max zero-fuel mass = MTOM * (1 - mf_fuel)
%    b     = wingspan [m]
% -------------------------------------------------------------------------
A_w   = 4.58e-3;
m_MZF = MTOM * (1 - mf_fuel);

m_torenbeek = A_w * b^0.75 ...
            * (1 + sqrt(6.3 * cos(sweep_c2) / b)) ...
            * n_ult^0.55 ...
            * (b * S / (m_MZF * tc))^0.30;

% -------------------------------------------------------------------------
%  AVERAGE  (reduces individual method scatter)
% -------------------------------------------------------------------------
m_primary_avg = (m_raymer + m_torenbeek) / 2;

% -------------------------------------------------------------------------
%  MATERIAL CORRECTION
%  CFRP saves mass proportional to the ratio of mass-specific strengths:
%    f_mat = (σ_all/ρ)_Al / (σ_all/ρ)_CF
%  Applies to primary structural mass only.
% -------------------------------------------------------------------------
if upper(material) == "CF"
    ssp_Al = p.Al.sig_all / p.Al.rho;
    ssp_CF = p.CF.sig_all / p.CF.rho;
    f_mat  = ssp_Al / ssp_CF;
    m_primary_avg = m_primary_avg * f_mat;
    mat_name = 'CFRP';
else
    f_mat    = 1.0;
    mat_name = 'Aluminium 7075-T6';
end

% -------------------------------------------------------------------------
%  FOLDING WINGTIP HINGE PENALTY
%  Outer panel span = Span/2 - y_hinge
%  Outer panel carries ~40% of total wing primary mass
%  (structural loads decrease rapidly outboard — lighter outer box)
%  Hinge penalty = f_hinge_mech × outer panel mass
% -------------------------------------------------------------------------
s         = b / 2;
b_outer   = s - y_hinge;                        % outer panel semi-span [m]
frac_out  = b_outer / s;                        % fraction of semi-span
m_outer   = m_primary_avg * frac_out * 0.40;    % outer panel primary mass
m_hinge   = p.f_hinge_mech * m_outer;

% -------------------------------------------------------------------------
%  TOTAL
% -------------------------------------------------------------------------
m_total = m_primary_avg + m_hinge;

% -------------------------------------------------------------------------
%  PACK OUTPUT
% -------------------------------------------------------------------------
E.m_raymer      = m_raymer;
E.m_torenbeek   = m_torenbeek;
E.m_primary_avg = m_primary_avg;
E.m_hinge       = m_hinge;
E.m_total       = m_total;
E.m_frac_MTOM   = m_total / MTOM;
E.material      = material;
E.mat_name      = mat_name;
E.f_mat         = f_mat;
E.AR            = AR;
E.lambda        = lambda;
E.sweep_c2_deg  = p.sweep_c2_deg;
E.tc_root       = tc;

% -------------------------------------------------------------------------
%  PRINT SUMMARY
% -------------------------------------------------------------------------
fprintf('\n--- Class I/II Empirical Wing Mass: %s ---\n', mat_name);
fprintf('  Raymer (2018):          %7.0f kg  (%4.1f%% MTOM)\n', ...
        m_raymer, m_raymer/MTOM*100);
fprintf('  Torenbeek (2013):       %7.0f kg  (%4.1f%% MTOM)\n', ...
        m_torenbeek, m_torenbeek/MTOM*100);
fprintf('  Average (primary):      %7.0f kg\n', m_primary_avg);
fprintf('  Hinge penalty:          %7.0f kg  (outer panel × %.0f%%)\n', ...
        m_hinge, p.f_hinge_mech*100);
fprintf('  TOTAL:                  %7.0f kg  (%4.1f%% MTOM)\n', ...
        m_total, m_total/MTOM*100);
fprintf('  [B777F reference: ~34,000 kg  (~9.8%% MTOM)]\n');
fprintf('  Validation error: %+.1f%%\n', (m_total - 34000)/34000*100);

end