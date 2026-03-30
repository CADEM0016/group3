function MB = MassBuildup(adp, loc, G, W)

N    = G.N;
dy   = G.dy;
rho  = W.rho_mat;
ih   = G.i_hinge;
MTOM = double(adp.MTOM);   % from adp directly

m_skin_dist = zeros(1, N);
m_cap_dist  = zeros(1, N);
m_web_dist  = zeros(1, N);

for i = 1:N
    h   = G.h_wb(i);
    w   = G.w_wb(i);
    t_s = W.t_skin(i);
    A_c = W.A_cap(i);
    t_w = W.t_web(i);

    m_skin_dist(i) = rho * 2*t_s*w * dy;   % top + bottom skins
    m_cap_dist(i)  = rho * 4*A_c   * dy;   % four spar caps
    m_web_dist(i)  = rho * 2*t_w*h * dy;   % front + rear webs
end

m_skin = 2 * sum(m_skin_dist);
m_caps = 2 * sum(m_cap_dist);
m_webs = 2 * sum(m_web_dist);

m_primary   = m_skin + m_caps + m_webs;
m_secondary = loc.f_secondary * m_primary;

m_outer = 2 * (sum(m_skin_dist(1:ih)) + sum(m_cap_dist(1:ih)) + sum(m_web_dist(1:ih)));
m_hinge = max(loc.f_hinge_mech * m_outer, loc.m_hinge_min);

m_total = m_primary + m_secondary + m_hinge;

m_dist      = m_skin_dist + m_cap_dist + m_web_dist;
y_CG_wing   = dot(m_dist, G.y) / sum(m_dist);
I_roll_wing = 2 * dot(m_dist, G.y.^2);

MB.m_skin          = m_skin;
MB.m_caps          = m_caps;
MB.m_webs          = m_webs;
MB.m_primary       = m_primary;
MB.m_secondary     = m_secondary;
MB.m_hinge         = m_hinge;
MB.m_total         = m_total;
MB.m_frac_MTOM     = m_total / MTOM;
MB.m_outer_primary = m_outer;
MB.m_skin_dist     = m_skin_dist;
MB.m_total_dist    = m_dist;
MB.y_CG_wing       = y_CG_wing;
MB.I_roll_wing     = I_roll_wing;

fprintf('\n--- Mass Buildup: %s (%s) ---\n', W.material, W.mat_name);
fprintf('  Skin        %6.0f kg  (%4.1f%%)\n', m_skin, m_skin/m_primary*100);
fprintf('  Caps        %6.0f kg  (%4.1f%%)\n', m_caps, m_caps/m_primary*100);
fprintf('  Webs        %6.0f kg  (%4.1f%%)\n', m_webs, m_webs/m_primary*100);
fprintf('  Primary     %6.0f kg\n', m_primary);
fprintf('  Secondary   %6.0f kg  (%.0f%% of primary)\n', m_secondary, loc.f_secondary*100);
fprintf('  Hinge       %6.0f kg\n', m_hinge);
fprintf('  TOTAL       %6.0f kg  (%.2f%% MTOM)\n', m_total, MB.m_frac_MTOM*100);
fprintf('  B777F ref   ~34,000 kg\n');
fprintf('  Wing CG     %.2f m from CL  |  I_roll = %.4e kg·m²\n', y_CG_wing, I_roll_wing);

end
