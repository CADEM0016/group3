function L = LoadDistribution(p, G, loadcase)

arguments
    p         struct
    G         struct
    loadcase  string = '2.5g'
end

N  = G.N;
dy = G.dy;
g  = p.g;

% =========================================================================
%  LOAD CASE SELECTION  (CS-25.337)
% =========================================================================
switch loadcase
    case '2.5g'
        n_lim       = p.n_limit_pos;
        n_ult       = p.n_ult_pos;
        sign_lift   = +1;
        sign_relief = -1;
    case '1g'
        n_lim       = 1.0;
        n_ult       = 1.0 * p.SF;
        sign_lift   = +1;
        sign_relief = -1;
    case 'neg1g'
        n_lim       = abs(p.n_limit_neg);
        n_ult       = abs(p.n_ult_neg);
        sign_lift   = -1;
        sign_relief = +1;
    otherwise
        error('LoadDistribution: unknown case "%s". Use: 2.5g | 1g | neg1g', loadcase);
end

% =========================================================================
%  TOTAL LIFT  (Snorri Ch6: L = W0 * nz, semi-wing = half of that)
% =========================================================================
L_total = n_lim * p.MTOM * g;             % total aircraft lift [N]
L_semi  = L_total / 2;                    % one semi-wing lift [N]

% =========================================================================
%  ELLIPTICAL LIFT DISTRIBUTION  (Snorri Ch9 / Prandtl optimal)
%  l(y) proportional to sqrt(1 - (2y/b)^2)
%  Normalised so that integral from 0 to s = L_semi
% =========================================================================
eta_aero   = 1 - G.eta;                   % 0=root, 1=tip (aero convention)
lift_shape = sqrt(max(1 - eta_aero.^2, 0));

norm_factor    = trapz(G.y(end:-1:1), lift_shape(end:-1:1));
lift_dist      = lift_shape .* (L_semi / norm_factor);   % [N/m], tip->root

% =========================================================================
%  SNORRI SIMPLIFIED VERIFICATION  (Ch6, uniform distribution check)
%  M_root_simple = L*b/8  (useful as a sanity bound)
% =========================================================================
M_root_snorri = L_total * p.Span / 8;

% =========================================================================
%  WING SELF-WEIGHT INERTIA RELIEF
%  Distributed proportional to local chord (heavier inboard)
%  m_wing_semi ~ 9% of MTOM per semi-wing (Class I estimate)
% =========================================================================
m_wing_semi  = 0.09 * p.MTOM / 2;

chord_integral = trapz(G.y(end:-1:1), G.chord(end:-1:1));
chord_norm     = G.chord / max(chord_integral, 1e-10);
w_wing_load    = n_ult * m_wing_semi .* chord_norm * g;   % [N/m], ult

% =========================================================================
%  FUEL INERTIA RELIEF
%  97% of fuel in wing tanks; distributed proportional to wingbox volume
%  Fuel mass: Snorri Ch6 — Wf = Mf * W0
%  Reserve fuel is NOT counted as available relief (worst case)
% =========================================================================
m_fuel_usable = (1 - p.Mf_res) * p.M_fuel;   % exclude reserve
m_fuel_semi   = 0.97 * m_fuel_usable / 2;

vol_integral  = trapz(G.y(end:-1:1), G.A_enc(end:-1:1));
vol_norm      = G.A_enc / max(vol_integral, 1e-10);
w_fuel_load   = n_ult * m_fuel_semi .* vol_norm * g;      % [N/m], ult

% =========================================================================
%  NET DISTRIBUTED LOAD
%  2.5g / 1g: net = +lift - wing_relief - fuel_relief
%  neg1g:     net = -lift + wing_relief + fuel_relief
% =========================================================================
net_dist = sign_lift * lift_dist + sign_relief * (w_wing_load + w_fuel_load);

% =========================================================================
%  ENGINE POINT LOAD  (magnitude only; SMT applies sign)
%  P_engine = nz * m_engine * g   [N]
% =========================================================================
P_engine = n_ult * p.m_engine_each * g;

% =========================================================================
%  HINGE TIP LIFT  (Snorri folding tip: L_tip = (2L/b)*delta_b)
%  delta_b = span - taxi_span = 72 - 65 = 7 m  (both wingtips combined)
%  Stored for hinge moment reference
% =========================================================================
delta_b  = p.Span - p.Span_taxi;
L_tip    = (2 * L_total / p.Span) * (delta_b / 2);   % per tip panel [N]

% ---- Pack output
L.lift_dist      = lift_dist;
L.w_wing         = w_wing_load;
L.w_fuel         = w_fuel_load;
L.net_dist       = net_dist;
L.P_engine       = P_engine;
L.n_limit        = n_lim;
L.n_ult          = n_ult;
L.sign_lift      = sign_lift;
L.sign_relief    = sign_relief;
L.L_semi         = L_semi;
L.L_total        = L_total;
L.L_root         = lift_dist(G.N);
L.L_tip          = L_tip;
L.M_root_snorri  = M_root_snorri;
L.m_wing_semi    = m_wing_semi;
L.m_fuel_semi    = m_fuel_semi;
L.loadcase       = loadcase;

total_relief = w_wing_load + w_fuel_load;
relief_total = trapz(G.y(end:-1:1), total_relief(end:-1:1));
relief_pct   = relief_total / trapz(G.y(end:-1:1), lift_dist(end:-1:1)) * 100;

fprintf('\n--- Load Distribution: %s ---\n', loadcase);
fprintf('  n_limit = %.1f  |  n_ult = %.2f\n', n_lim, n_ult);
fprintf('  Total aircraft lift:      %8.3f MN\n', L_total/1e6);
fprintf('  Semi-wing lift:           %8.3f MN\n', L_semi/1e6);
fprintf('  Snorri M_root check:      %8.3f MNm  (Lb/8)\n', M_root_snorri/1e6);
fprintf('  Wing inertia relief:      %8.3f MN\n', ...
    trapz(G.y(end:-1:1), w_wing_load(end:-1:1))/1e6);
fprintf('  Fuel  inertia relief:     %8.3f MN\n', ...
    trapz(G.y(end:-1:1), w_fuel_load(end:-1:1))/1e6);
fprintf('  Engine point load:        %8.3f MN\n', P_engine/1e6);
fprintf('  Tip panel lift (Snorri):  %8.3f kN\n', L_tip/1e3);
fprintf('  Total relief / lift:      %8.1f%%\n', relief_pct);

end
