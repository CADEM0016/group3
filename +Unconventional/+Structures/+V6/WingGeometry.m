function G = WingGeometry(adp, tlar, loc, N_override)

arguments
    adp
    tlar
    loc        struct
    N_override double = 0
end

N = loc.N_stations;
if N_override > 0,  N = N_override;  end

Span     = double(adp.Span);
WingArea = double(adp.WingArea);

s  = Span / 2;
dy = s / (N - 1);

% Stations run tip→root so free-end BCs (Q=M=T=0) sit at station 1
y   = linspace(s, 0, N);
eta = 1 - y/s;   % 0 at tip, 1 at root

AR     = Span^2 / WingArea;
c_root = 2 * WingArea / (Span * (1 + loc.lambda));
c_tip  = loc.lambda * c_root;
MAC    = (2/3) * c_root * (1 + loc.lambda + loc.lambda^2) / (1 + loc.lambda);

chord = c_root * (1 - (2*y/Span) .* (1 - loc.lambda));
tc    = loc.tc_tip + (loc.tc_root - loc.tc_tip) .* eta;

h_wb  = tc .* chord;
w_wb  = (loc.fs_aft - loc.fs_fwd) .* chord;
A_enc = h_wb .* w_wb;

e_oswald = 1.78 * (1 - 0.045 * AR^0.68) - 0.64;

x_LE    = (s - y) .* tan(deg2rad(loc.sweep_LE_deg));
x_ac    = x_LE + 0.25 .* chord;
fa_frac = 0.25 + 0.25 .* eta;   % flexural axis 25% (tip) → 50% (root)
x_fa    = x_LE + fa_frac .* chord;
e_ac_fa = x_ac - x_fa;          % AC–FA offset drives torsion

% Engine configuration - 4 engines, 2 per semi-wing
if isprop(adp,'Engine') && ~isempty(adp.Engine)
    m_eng = adp.Engine.Mass;        % kg  per engine (rubberised, from PPC)
else
    m_eng = loc.m_engine_each;      % kg  fallback from AircraftParams
end

N_en  = loc.N_engines;              % 4 total
% Spanwise positions of the two engine stations per semi-wing
y_eng1 = loc.y_eng1_frac * s;      % m  inner engine (existing)
y_eng2 = loc.y_eng2_frac * s;      % m  outer engine (new)

[~, i_hinge]   = min(abs(y - loc.y_hinge));
[~, i_engine1] = min(abs(y - y_eng1));
[~, i_engine2] = min(abs(y - y_eng2));

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
G.Span         = Span;
G.c_root       = c_root;
G.c_tip        = c_tip;
G.MAC          = MAC;
G.lambda       = loc.lambda;
G.wb_frac      = loc.fs_aft - loc.fs_fwd;
G.y_engine     = y_eng1;       % m  inner engine (backward-compatible alias)
G.y_engine1    = y_eng1;       % m  inner engine station
G.y_engine2    = y_eng2;       % m  outer engine station
G.N_engines    = N_en;         % 4 total
G.m_engine     = m_eng;        % kg per engine (from propulsion)
G.y_hinge      = loc.y_hinge;
G.i_hinge      = i_hinge;
G.i_engine     = i_engine1;    % backward-compatible alias
G.i_engine1    = i_engine1;
G.i_engine2    = i_engine2;
G.sweep_LE_deg = loc.sweep_LE_deg;
G.sweep_c2_deg = loc.sweep_c2_deg;

fprintf('\n--- Wing Geometry ---\n');
fprintf('  Semi-span          %.2f m\n',         s);
fprintf('  Root / tip chord   %.2f / %.2f m\n',  c_root, c_tip);
fprintf('  MAC                %.2f m\n',          MAC);
fprintf('  AR / taper         %.2f / %.2f\n',     AR, loc.lambda);
fprintf('  LE / c/2 sweep     %.1f / %.1f deg\n', loc.sweep_LE_deg, loc.sweep_c2_deg);
fprintf('  t/c root / tip     %.3f / %.3f\n',     loc.tc_root, loc.tc_tip);
fprintf('  Oswald e           %.4f\n',             e_oswald);
fprintf('  Wingbox            %.0f-%.0f%% chord\n', loc.fs_fwd*100, loc.fs_aft*100);
fprintf('  Fold hinge         y = %.1f m  (stn %d)\n', loc.y_hinge, i_hinge);
fprintf('  Engine config      %d total  (%d per semi-wing)\n', N_en, N_en/2);
fprintf('  Inner engine       y = %.1f m  (stn %d)   [%.0f%% semi-span]\n', y_eng1, i_engine1, loc.y_eng1_frac*100);
fprintf('  Outer engine       y = %.1f m  (stn %d)   [%.0f%% semi-span]\n', y_eng2, i_engine2, loc.y_eng2_frac*100);
if isprop(adp,'Engine') && ~isempty(adp.Engine)
    eng_src = 'adp.Engine (propulsion)';
else
    eng_src = 'AircraftParams fallback';
end
fprintf('  Mass per engine    %.0f kg  (source: %s)\n', m_eng, eng_src);
fprintf('  Stations           %d  (dy = %.3f m)\n', N, dy);

end