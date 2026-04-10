function L = LoadDistribution(adp, tlar, loc, G, loadcase)

arguments
    adp
    tlar
    loc      struct
    G        struct
    loadcase string = '2.5g'
end

% From global files directly
g      = SI.g;
MTOM   = double(adp.MTOM);
Span   = double(adp.Span);
Mf     = double(adp.Mf_Fuel);
Mf_res = double(adp.Mf_res);

switch loadcase
    case '2.5g'
        n_lim = loc.n_limit_pos;
        n_ult = loc.n_limit_pos * loc.SF;
        sign_lift = +1;  sign_relief = -1;
    case '1g'
        n_lim = 1.0;
        n_ult = 1.0 * loc.SF;
        sign_lift = +1;  sign_relief = -1;
    case 'neg1g'
        n_lim = abs(loc.n_limit_neg);
        n_ult = abs(loc.n_limit_neg) * loc.SF;
        sign_lift = -1;  sign_relief = +1;
    otherwise
        error('LoadDistribution: use 2.5g | 1g | neg1g');
end

L_total = n_lim * MTOM * g;
L_semi  = L_total / 2;

% Elliptical lift distribution normalised to L_semi
eta_aero   = 1 - G.eta;
lift_shape = sqrt(max(1 - eta_aero.^2, 0));
lift_dist  = lift_shape * (L_semi / trapz(G.y(end:-1:1), lift_shape(end:-1:1)));

M_root_snorri = L_total * Span / 8;   % Snorri Lb/8 sanity check

% Wing self-weight relief proportional to chord
m_wing_semi = 0.09 * MTOM / 2;
chord_norm  = G.chord / trapz(G.y(end:-1:1), G.chord(end:-1:1));
w_wing_load = n_ult * m_wing_semi .* chord_norm * g;

% Fuel inertia relief proportional to wingbox volume; reserve excluded
M_fuel      = Mf * MTOM;
m_fuel_semi = 0.97 * (1 - Mf_res) * M_fuel / 2;
vol_norm    = G.A_enc / trapz(G.y(end:-1:1), G.A_enc(end:-1:1));
w_fuel_load = n_ult * m_fuel_semi .* vol_norm * g;

net_dist = sign_lift * lift_dist + sign_relief * (w_wing_load + w_fuel_load);
% Engine point loads - fetch from propulsion output at point-of-use.
if isprop(adp,'Engine') && ~isempty(adp.Engine) && isprop(adp.Engine,'Mass') && ~isempty(adp.Engine.Mass)
    m_engine = double(adp.Engine.Mass);
else
    m_engine = G.m_engine;  % fallback from geometry handoff
end
P_engine1 = n_ult * m_engine * g;   % inner engine point load
P_engine2 = n_ult * m_engine * g;   % outer engine point load
L_tip    = (2 * L_total / Span) * ((Span - loc.Span_taxi) / 2);

L.lift_dist     = lift_dist;
L.w_wing        = w_wing_load;
L.w_fuel        = w_fuel_load;
L.net_dist      = net_dist;
L.P_engine      = P_engine1;    
L.P_engine1     = P_engine1;   % N  inner engine point load
L.P_engine2     = P_engine2;   % N  outer engine point load
L.n_limit       = n_lim;
L.n_ult         = n_ult;
L.sign_lift     = sign_lift;
L.sign_relief   = sign_relief;
L.L_semi        = L_semi;
L.L_total       = L_total;
L.L_root        = lift_dist(G.N);
L.L_tip         = L_tip;
L.M_root_snorri = M_root_snorri;
L.m_wing_semi   = m_wing_semi;
L.m_fuel_semi   = m_fuel_semi;
L.loadcase      = loadcase;

total_relief = w_wing_load + w_fuel_load;
relief_pct   = trapz(G.y(end:-1:1), total_relief(end:-1:1)) ...
             / trapz(G.y(end:-1:1), lift_dist(end:-1:1)) * 100;

fprintf('\n--- Load Distribution: %s ---\n', loadcase);
fprintf('  n_limit / n_ult       %.1f / %.2f\n',  n_lim, n_ult);
fprintf('  Total lift            %.3f MN\n',       L_total/1e6);
fprintf('  Semi-wing lift        %.3f MN\n',       L_semi/1e6);
fprintf('  Root BM (Snorri Lb/8) %.3f MNm\n',     M_root_snorri/1e6);
fprintf('  Wing relief           %.3f MN\n', trapz(G.y(end:-1:1), w_wing_load(end:-1:1))/1e6);
fprintf('  Fuel  relief          %.3f MN\n', trapz(G.y(end:-1:1), w_fuel_load(end:-1:1))/1e6);
fprintf('  Engine loads (inner/outer)  %.3f / %.3f MN  (per station)\n', P_engine1/1e6, P_engine2/1e6);
fprintf('  Tip panel lift        %.3f kN\n',       L_tip/1e3);
fprintf('  Relief / lift         %.1f%%\n',         relief_pct);

end