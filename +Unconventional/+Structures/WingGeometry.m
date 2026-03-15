function G = WingGeometry(p, N_override)

arguments
    p          struct
    N_override double = 0
end

N  = p.N_stations;
if N_override > 0;  N = N_override;  end

s  = p.Span / 2;
dy = s / (N - 1);

% Spanwise stations: tip (y=s) to root (y=0)
y   = linspace(s, 0, N);
eta = 1 - y/s;              % 0 at tip, 1 at root

% ---- Chord distribution  Snorri Ch9: c(y) = cr*(1 - (2y/b)*(1-lambda))
chord = p.c_root * (1 - (2.*y/p.Span).*(1 - p.lambda));

% ---- Thickness-to-chord ratio (linear spanwise variation)
tc = p.tc_tip + (p.tc_root - p.tc_tip) .* eta;

% ---- Wingbox cross-section
h_wb  = tc    .* chord;
w_wb  = p.wb_frac .* chord;
A_enc = h_wb  .* w_wb;

% ---- Oswald efficiency  Snorri Ch9: e = 1.78*(1 - 0.045*AR^0.68) - 0.64
e_oswald = 1.78 * (1 - 0.045 * p.AR^0.68) - 0.64;

% ---- Sweep geometry
sweep_LE = deg2rad(p.sweep_LE_deg);
sweep_c2 = deg2rad(p.sweep_c2_deg);

x_LE    = (s - y) .* tan(sweep_LE);
x_ac    = x_LE + 0.25 .* chord;

% Flexural axis: 25% chord at tip to 50% chord at root
fa_frac = 0.25 + 0.25 .* eta;
x_fa    = x_LE + fa_frac .* chord;

e_ac_fa = x_ac - x_fa;

% ---- Nearest station indices
[~, i_hinge] = min(abs(y - p.y_hinge));
[~, i_engine]= min(abs(y - p.y_engine));

% ---- Pack output
G.y          = y;
G.eta        = eta;
G.chord      = chord;
G.tc         = tc;
G.h_wb       = h_wb;
G.w_wb       = w_wb;
G.A_enc      = A_enc;
G.x_LE       = x_LE;
G.x_ac       = x_ac;
G.x_fa       = x_fa;
G.e_ac_fa    = e_ac_fa;
G.e_oswald   = e_oswald;
G.s          = s;
G.N          = N;
G.dy         = dy;
G.i_hinge    = i_hinge;
G.i_engine   = i_engine;
G.c_root     = p.c_root;
G.c_tip      = p.c_tip;
G.MAC        = p.MAC;
G.AR         = p.AR;
G.lambda     = p.lambda;
G.sweep_LE_deg = p.sweep_LE_deg;
G.sweep_c2_deg = p.sweep_c2_deg;

fprintf('\n--- Wing Geometry ---\n');
fprintf('  Span (flight):    %.2f m\n',   p.Span);
fprintf('  Semi-span:        %.2f m\n',   s);
fprintf('  Root chord:       %.2f m\n',   p.c_root);
fprintf('  Tip  chord:       %.2f m\n',   p.c_tip);
fprintf('  MAC:              %.2f m\n',   p.MAC);
fprintf('  AR:               %.2f\n',     p.AR);
fprintf('  Taper ratio:      %.2f\n',     p.lambda);
fprintf('  LE sweep:         %.1f deg\n', p.sweep_LE_deg);
fprintf('  Half-chord sweep: %.1f deg\n', p.sweep_c2_deg);
fprintf('  t/c root:         %.3f\n',     p.tc_root);
fprintf('  t/c tip:          %.3f\n',     p.tc_tip);
fprintf('  Oswald e:         %.4f\n',     e_oswald);
fprintf('  Wingbox:          %.0f%% to %.0f%% chord\n', p.fs_fwd*100, p.fs_aft*100);
fprintf('  Fold hinge:       y = %.1f m  (station %d)\n', p.y_hinge, i_hinge);
fprintf('  Engine:           y = %.1f m  (station %d)\n', p.y_engine, i_engine);
fprintf('  Stations:         %d  (dy = %.3f m)\n', N, dy);

end
