function MB = MassBuildup(p, G, W)

N   = G.N;
dy  = G.dy;
rho = W.rho_mat;

m_skin_dist = zeros(1, N);
m_cap_dist  = zeros(1, N);
m_web_dist  = zeros(1, N);

for i = 1:N

    h   = G.h_wb(i);
    w   = G.w_wb(i);
    t_s = W.t_skin(i);
    A_c = W.A_cap(i);
    t_w = W.t_web(i);

    % =================================================================
    %  DIRECT STRUCTURAL MASS  (Snorri Ch6: W = rho * V)
    %  Volume per unit span for each component:
    %    Skin (top + bottom):  2 * t_s * w        [m^2/m]
    %    Spar caps (x4):       4 * A_c             [m^2/m]
    %    Spar webs (x2):       2 * t_w * h         [m^2/m]
    %  Mass at station = rho * V_per_m * dy
    % =================================================================
    m_skin_dist(i) = rho * (2 * t_s * w)  * dy;
    m_cap_dist(i)  = rho * (4 * A_c)      * dy;
    m_web_dist(i)  = rho * (2 * t_w * h)  * dy;

end

m_skin  = 2 * sum(m_skin_dist);
m_caps  = 2 * sum(m_cap_dist);
m_webs  = 2 * sum(m_web_dist);

m_primary   = m_skin + m_caps + m_webs;
m_secondary = p.f_secondary * m_primary;

% =========================================================================
%  FOLDING HINGE PENALTY
%  Outer panel: stations 1 (tip) to i_hinge in tip->root order
% =========================================================================
ih = G.i_hinge;
m_outer_primary = 2 * (sum(m_skin_dist(1:ih)) + ...
                       sum(m_cap_dist(1:ih))  + ...
                       sum(m_web_dist(1:ih)));

m_hinge = p.f_hinge_mech * m_outer_primary;
m_hinge = max(m_hinge, p.m_hinge_min);

m_total = m_primary + m_secondary + m_hinge;

% =========================================================================
%  SPANWISE CENTRE OF GRAVITY OF WING STRUCTURE  (Snorri Ch6)
%  x_CG = sum(m_i * y_i) / sum(m_i)
%  Uses one semi-wing primary mass distribution for the spanwise CG
% =========================================================================
m_total_dist  = m_skin_dist + m_cap_dist + m_web_dist;   % one semi-wing
y_CG_wing     = dot(m_total_dist, G.y) / max(sum(m_total_dist), 1e-12);

% =========================================================================
%  MASS MOMENT OF INERTIA  (Snorri Ch6: I = sum(m * r^2))
%  Roll inertia contribution from one semi-wing primary structure
%  r = spanwise distance from centreline = G.y
% =========================================================================
I_roll_semi   = dot(m_total_dist, G.y.^2);
I_roll_wing   = 2 * I_roll_semi;                         % both wings

% ---- Pack output
MB.m_skin          = m_skin;
MB.m_caps          = m_caps;
MB.m_webs          = m_webs;
MB.m_primary       = m_primary;
MB.m_secondary     = m_secondary;
MB.m_hinge         = m_hinge;
MB.m_total         = m_total;
MB.m_frac_MTOM     = m_total / p.MTOM;
MB.m_outer_primary = m_outer_primary;
MB.m_skin_dist     = m_skin_dist;
MB.m_total_dist    = m_total_dist;
MB.y_CG_wing       = y_CG_wing;
MB.I_roll_wing     = I_roll_wing;

fprintf('\n--- Mass Buildup: %s (%s) ---\n', W.material, W.mat_name);
fprintf('  PRIMARY WINGBOX\n');
fprintf('    Skin (top+bot):   %7.0f kg  (%4.1f%%)\n', m_skin, m_skin/m_primary*100);
fprintf('    Spar caps (x4):   %7.0f kg  (%4.1f%%)\n', m_caps, m_caps/m_primary*100);
fprintf('    Spar webs (x2):   %7.0f kg  (%4.1f%%)\n', m_webs, m_webs/m_primary*100);
fprintf('    Primary total:    %7.0f kg\n', m_primary);
fprintf('  SECONDARY (%.0f%%): %7.0f kg\n', p.f_secondary*100, m_secondary);
fprintf('  HINGE PENALTY:      %7.0f kg\n', m_hinge);
fprintf('  ----------------------------------------\n');
fprintf('  TOTAL WING MASS:    %7.0f kg  (%.2f%% MTOM)\n', m_total, MB.m_frac_MTOM*100);
fprintf('  Wing CG spanwise:   %7.2f m  from centreline\n', y_CG_wing);
fprintf('  Wing roll inertia:  %.4e kg*m^2\n', I_roll_wing);
fprintf('  [B777F reference:   ~34,000 kg]\n');

end
