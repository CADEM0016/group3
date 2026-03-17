function G = WingGeometry(adp, tlar, loc, N_override)

arguments
    adp        
    tlar       
    loc        struct
    N_override double = 0
end

N  = loc.N_stations;
if N_override > 0,  N = N_override;  end

% Global values fetched directly
Span    = adp.Span;
WingArea = adp.WingArea;
lambda  = loc.lambda;

s  = Span / 2;
dy = s / (N - 1);

% Stations run tip→root so free-end BCs (Q=M=T=0) sit at station 1
y = linspace(s, 0, N);
eta = 1 - y/s;              % 0 at tip, 1 at root

AR     = Span^2 / WingArea;
c_root = 2 * WingArea / (Span * (1 + lambda));
c_tip  = lambda * c_root;
MAC    = (2/3) * c_root * (1 + lambda + lambda^2) / (1 + lambda);

chord = c_root * (1 - (2*y/Span) .* (1 - lambda));
tc    = loc.tc_tip + (loc.tc_root - loc.tc_tip) .* eta;

h_wb  = tc             .* chord;   % wingbox height
w_wb  = (loc.fs_aft - loc.fs_fwd) .* chord;   % wingbox width
A_enc = h_wb           .* w_wb;

e_oswald = 1.78 * (1 - 0.045 * AR^0.68) - 0.64;

x_LE    = (s - y) .* tan(deg2rad(loc.sweep_LE_deg));
x_ac    = x_LE + 0.25 .* chord;
fa_frac = 0.25 + 0.25 .* eta;     % flexural axis: 25% at tip → 50% at root
x_fa    = x_LE + fa_frac .* chord;
e_ac_fa = x_ac - x_fa;            % AC-to-FA offset (torque arm)

y_engine = 0.35 * s;

[~, i_hinge]  = min(abs(y - loc.y_hinge));
[~, i_engine] = min(abs(y - y_engine));

G.y            = y;
G.eta          = eta;
G.chord        = chord;
G.tc           = tc;
G.h_wb         = h_wb;
G.w_wb         = w_wb;
G.A_enc        = A_enc;
G.x_LE         = x_LE;
G.x_ac         = x_ac;
G.x_fa         = x_fa;
G.e_ac_fa      = e_ac_fa;
G.e_oswald     = e_oswald;
G.s            = s;
G.N            = N;
G.dy           = dy;
G.AR           = AR;
G.c_root       = c_root;
G.c_tip        = c_tip;
G.MAC          = MAC;
G.lambda       = lambda;
G.wb_frac      = loc.fs_aft - loc.fs_fwd;
G.y_hinge      = loc.y_hinge;
G.y_engine     = y_engine;
G.i_hinge      = i_hinge;
G.i_engine     = i_engine;
G.sweep_LE_deg = loc.sweep_LE_deg;
G.sweep_c2_deg = loc.sweep_c2_deg;

fprintf('\n--- Wing Geometry ---\n');
fprintf('  Semi-span          %.2f m\n',         s);
fprintf('  Root / tip chord   %.2f / %.2f m\n',  c_root, c_tip);
fprintf('  MAC                %.2f m\n',          MAC);
fprintf('  AR / taper         %.2f / %.2f\n',     AR, lambda);
fprintf('  LE / c/2 sweep     %.1f / %.1f deg\n', loc.sweep_LE_deg, loc.sweep_c2_deg);
fprintf('  t/c root / tip     %.3f / %.3f\n',     loc.tc_root, loc.tc_tip);
fprintf('  Oswald e           %.4f\n',             e_oswald);
fprintf('  Wingbox            %.0f–%.0f%% chord\n', loc.fs_fwd*100, loc.fs_aft*100);
fprintf('  Fold hinge         y = %.1f m  (stn %d)\n', loc.y_hinge, i_hinge);
fprintf('  Engine             y = %.1f m  (stn %d)\n', y_engine,    i_engine);
fprintf('  Stations           %d  (dy = %.3f m)\n',    N, dy);

end
