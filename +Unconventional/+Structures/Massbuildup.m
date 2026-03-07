function MB = MassBuildup(p, G, W)
% =========================================================================
% MassBuildup.m  —  +Structures package
% Estimate total wing structural mass from sized cross-sections.
%
% Method:
%   1. Integrate primary wingbox material volume along span
%   2. Multiply by material density → primary structural mass
%   3. Add secondary structure allowance (ribs, LE, TE, ctrl surfaces)
%   4. Add folding wingtip hinge mechanism penalty
%
% Structure of wingbox at each station:
%   - Top skin    : t_skin × w_wb  [area per unit span]
%   - Bottom skin : t_skin × w_wb
%   - Two spar webs: each  t_web × h_wb
%   - 4 spar caps : each A_cap  (front top/bot, rear top/bot)
%   Both wings (×2 for full aircraft)
%
% INPUT:
%   p  — AircraftParams struct
%   G  — WingGeometry struct
%   W  — WingboxSizing struct
%
% OUTPUT:  MB — struct
%   MB.m_skin       [kg]  skin mass (both wings)
%   MB.m_caps       [kg]  spar cap mass
%   MB.m_webs       [kg]  spar web mass
%   MB.m_primary    [kg]  total primary wingbox mass
%   MB.m_secondary  [kg]  secondary structure
%   MB.m_hinge      [kg]  folding hinge mechanism penalty
%   MB.m_total      [kg]  TOTAL wing mass
%   MB.m_frac_MTOM  [-]   wing mass as fraction of MTOM
%   MB.m_skin_dist  [kg/m] skin mass per unit span (one semi-wing)
%   MB.m_total_dist [kg/m] total primary mass per unit span
%
% NO external dependencies.
% =========================================================================

N   = G.N;
dy  = G.dy;
rho = W.rho_mat;

% Pre-allocate per-station mass arrays (one semi-wing)
m_skin_dist = zeros(1, N);
m_cap_dist  = zeros(1, N);
m_web_dist  = zeros(1, N);

for i = 1:N
    h  = G.h_wb(i);
    w  = G.w_wb(i);

    t_s = W.t_skin(i);
    A_c = W.A_cap(i);
    t_w = W.t_web(i);

    % Volume per unit span [m^2] × dy [m] → volume [m^3] × rho → mass [kg]
    % Top + bottom skin panels
    V_skin_per_m = 2 * t_s * w;           % [m^2/m]
    % 4 spar caps (front spar: top+bot, rear spar: top+bot)
    V_caps_per_m = 4 * A_c;               % [m^2/m]
    % 2 spar webs (front and rear)
    V_web_per_m  = 2 * t_w * h;           % [m^2/m]

    m_skin_dist(i) = rho * V_skin_per_m * dy;
    m_cap_dist(i)  = rho * V_caps_per_m * dy;
    m_web_dist(i)  = rho * V_web_per_m  * dy;
end

% Sum over semi-span → multiply by 2 for full aircraft
m_skin_semi  = sum(m_skin_dist);
m_caps_semi  = sum(m_cap_dist);
m_webs_semi  = sum(m_web_dist);

m_skin  = 2 * m_skin_semi;
m_caps  = 2 * m_caps_semi;
m_webs  = 2 * m_webs_semi;

m_primary = m_skin + m_caps + m_webs;

% -------------------------------------------------------------------------
%  SECONDARY STRUCTURE
%  35% allowance on primary wingbox mass.
%  Covers: ribs, leading edge, trailing edge structure, control surfaces,
%          access panels, sealant, fasteners, surface protection.
% -------------------------------------------------------------------------
m_secondary = p.f_secondary * m_primary;

% -------------------------------------------------------------------------
%  FOLDING WINGTIP HINGE MECHANISM PENALTY
%  The hinge joint introduces:
%   - Structural doublers around the hinge cut-out
%   - Actuator and lock mechanism
%   - Fairing and seal
%  Estimated as f_hinge × outer panel primary mass.
%
%  Outer panel: from hinge (G.i_hinge) to tip (station 1)
%  Note: G.y is tip→root, so outer panel = stations 1 : i_hinge
% -------------------------------------------------------------------------
ih = G.i_hinge;
% Outer panel semi-span primary mass
m_outer_skin_semi = sum(m_skin_dist(1:ih));
m_outer_caps_semi = sum(m_cap_dist(1:ih));
m_outer_webs_semi = sum(m_web_dist(1:ih));
m_outer_primary   = 2 * (m_outer_skin_semi + m_outer_caps_semi + m_outer_webs_semi);

m_hinge = p.f_hinge_mech * m_outer_primary;

m_hinge = max(m_hinge, p.m_hinge_min);

% -------------------------------------------------------------------------
%  TOTAL
% -------------------------------------------------------------------------
m_total = m_primary + m_secondary + m_hinge;

% -------------------------------------------------------------------------
%  PACK OUTPUT
% -------------------------------------------------------------------------
MB.m_skin         = m_skin;
MB.m_caps         = m_caps;
MB.m_webs         = m_webs;
MB.m_primary      = m_primary;
MB.m_secondary    = m_secondary;
MB.m_hinge        = m_hinge;
MB.m_total        = m_total;
MB.m_frac_MTOM    = m_total / p.MTOM;
MB.m_outer_primary= m_outer_primary;
MB.m_skin_dist    = m_skin_dist;                         % one semi-wing [kg/station]
MB.m_total_dist   = m_skin_dist + m_cap_dist + m_web_dist; % one semi-wing

% -------------------------------------------------------------------------
%  PRINT SUMMARY
% -------------------------------------------------------------------------
fprintf('\n--- Mass Buildup: %s (%s) ---\n', W.material, W.mat_name);
fprintf('  PRIMARY WINGBOX\n');
fprintf('    Skin (top+bot):    %7.0f kg  (%4.1f%%)\n', m_skin, m_skin/m_primary*100);
fprintf('    Spar caps (×4):    %7.0f kg  (%4.1f%%)\n', m_caps, m_caps/m_primary*100);
fprintf('    Spar webs (×2):    %7.0f kg  (%4.1f%%)\n', m_webs, m_webs/m_primary*100);
fprintf('    Primary total:     %7.0f kg\n', m_primary);
fprintf('  SECONDARY STRUCTURE\n');
fprintf('    Secondary (%.0f%% of primary): %7.0f kg\n', p.f_secondary*100, m_secondary);
fprintf('  FOLDING HINGE MECHANISM\n');
fprintf('    Outer panel primary:   %7.0f kg\n', m_outer_primary);
fprintf('    Hinge penalty (%.0f%%): %7.0f kg\n', p.f_hinge_mech*100, m_hinge);
fprintf('  -----------------------------------------------\n');
fprintf('  TOTAL WING MASS:         %7.0f kg\n', m_total);
fprintf('  Wing mass fraction MTOM: %6.2f%%\n',  MB.m_frac_MTOM*100);
fprintf('  [B777F reference:        ~34000 kg]\n');

end