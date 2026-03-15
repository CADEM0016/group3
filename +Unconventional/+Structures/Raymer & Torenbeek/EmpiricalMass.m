function E = EmpiricalMass(p, material)
% =========================================================================
% Class I/II empirical wing mass estimation.
%
% Implements two independent methods and averages the result:
%   (1) Raymer (2018) Eq 15.25  — US customary, converted to SI here
%   (2) Torenbeek (2013) Eq 8.27 — native SI

arguments
    p        struct
    material string = 'Al'
end

% -------------------------------------------------------------------------
%  EXTRACT ALL INPUTS AS EXPLICIT SCALAR DOUBLES
% -------------------------------------------------------------------------
MTOM   = double(p.MTOM);           % [kg]  max take-off mass
b      = double(p.Span);           % [m]   full wingspan
S_wing = double(p.WingArea);       % [m^2] reference wing area
AR     = double(p.AR);             % [-]   aspect ratio
lam    = double(p.lambda);         % [-]   taper ratio c_tip/c_root
tc     = double(p.tc_root);        % [-]   root t/c ratio
mf     = double(p.Mf_fuel);        % [-]   fuel mass fraction of MTOM
n_ult  = double(p.n_ult_pos);      % [-]   ultimate load factor = 3.75
sw_deg = double(p.sweep_c2_deg);   % [deg] half-chord sweep
sw     = deg2rad(sw_deg);          % [rad]

% =========================================================================
%  METHOD 1 — RAYMER (2018) EQUATION 15.25
%
%  Where S_csw = control surface area (assumed 15% of S_w here)
% =========================================================================

% Unit conversion factors
kg_to_lb  = 2.20462;      % 1 kg = 2.20462 lb
m2_to_ft2 = 10.7639;      % 1 m^2 = 10.7639 ft^2

% Convert inputs to US customary
W_dg_lb   = MTOM   * kg_to_lb;    % design gross weight [lb]
S_w_ft2   = S_wing * m2_to_ft2;   % wing area [ft^2]
S_csw_ft2 = 0.15   * S_w_ft2;     % control surface area [ft^2]

% Apply Raymer Eq 15.25 in US customary
W_wing_lb = 0.0051 ...
          * (W_dg_lb * n_ult)^0.557 ...
          * S_w_ft2^0.649 ...
          * AR^0.5 ...
          * tc^(-0.4) ...
          * (1.0 + lam)^0.1 ...
          * (1.0 / cos(sw)) ...
          * S_csw_ft2^0.1;

% Convert result back to kg
m_raymer = W_wing_lb / kg_to_lb;

% =========================================================================
%  METHOD 2 — TORENBEEK (2013) EQUATION 8.27
%
%  Native SI throughout. No unit conversion needed.
%
%  m_w [kg] = A_w
%             × b^0.75
%             × [1 + sqrt(6.3 × cos(Lambda) / b)]
%             × N_z^0.55
%             × [b × S / (m_MZF × t/c)]^0.30
%
%  Where:
%    A_w   = 4.58e-3  (Torenbeek Table 8-4, metal transport wing)
%    m_MZF = max zero-fuel mass = MTOM × (1 - Mf_fuel)
%
%  Physical meaning of inner term [b×S/(m_MZF×t/c)]:
%    Numerator  b×S   [m^3]  — structural volume proxy (long span, large area)
%    Denominator m_MZF×t/c [kg] — inertia relief × section efficiency
%    Higher value = structurally more demanding = heavier wing
% =========================================================================

A_w   = 4.58e-3;
m_MZF = MTOM * (1.0 - mf);        % max zero-fuel mass [kg]

% Compute inner term explicitly — all scalar doubles, no ambiguity
num_tb    = b * S_wing;            % [m^3]
den_tb    = m_MZF * tc;            % [kg]  (tc is dimensionless)
inner_tb  = num_tb / den_tb;       % [m^3/kg] — units absorbed by A_w

bracket_tb = 1.0 + sqrt(6.3 * cos(sw) / b);

m_torenbeek = A_w ...
            * (b^0.75) ...
            * bracket_tb ...
            * (n_ult^0.55) ...
            * (inner_tb^0.30);

% =========================================================================
%  AVERAGE BOTH METHODS
% =========================================================================
m_avg = (m_raymer + m_torenbeek) / 2.0;

% =========================================================================
%  MATERIAL CORRECTION FOR CFRP
% =========================================================================
if strcmpi(material, 'CF')
    ssp_Al   = p.Al.sig_all / p.Al.rho;   % specific strength Al [Pa·m^3/kg]
    ssp_CF   = p.CF.sig_all / p.CF.rho;   % specific strength CF
    m_avg    = m_avg * (ssp_Al / ssp_CF);
    mat_name = 'CFRP';
else
    mat_name = 'Aluminium 7075-T6';
end

% =========================================================================
%  FOLDING HINGE MECHANISM PENALTY
% =========================================================================
b_semi  = p.Span / 2.0;                      % semi-span [m]
b_outer = b_semi - p.y_hinge;                % outer panel span [m] = 3.5m
f_outer = b_outer / b_semi;                  % fraction of semi-span
m_outer = m_avg * f_outer * 0.40;            % outer panel primary mass estimate
m_hinge = p.f_hinge_mech * m_outer;          % 10% penalty

% Apply minimum floor if defined in AircraftParams
if isfield(p, 'm_hinge_min')
    m_hinge = max(m_hinge, p.m_hinge_min);
end

m_total = m_avg + m_hinge;

% =========================================================================
%  PACK OUTPUT STRUCT
% =========================================================================
E.m_raymer      = m_raymer;
E.m_torenbeek   = m_torenbeek;
E.m_primary_avg = m_avg;
E.m_hinge       = m_hinge;
E.m_total       = m_total;
E.m_frac_MTOM   = m_total / MTOM;
E.material      = upper(material);
E.mat_name      = mat_name;

% =========================================================================
%  PRINT SUMMARY
% =========================================================================
fprintf('\n--- Class I/II Empirical Wing Mass: %s ---\n', mat_name);
fprintf('  Raymer (2018):          %7.0f kg  (%4.1f%% MTOM)\n', ...
        m_raymer,    m_raymer/MTOM*100);
fprintf('  Torenbeek (2013):       %7.0f kg  (%4.1f%% MTOM)\n', ...
        m_torenbeek, m_torenbeek/MTOM*100);
fprintf('  Average (primary):      %7.0f kg\n', m_avg);
fprintf('  Hinge penalty:          %7.0f kg\n', m_hinge);
fprintf('  TOTAL:                  %7.0f kg  (%4.1f%% MTOM)\n', ...
        m_total, m_total/MTOM*100);
fprintf('  [B777F reference: ~34,000 kg (~9.8%% MTOM)]\n');
fprintf('  Validation error vs B777F: %+.1f%%\n', ...
        (m_total - 34000)/34000*100);

end