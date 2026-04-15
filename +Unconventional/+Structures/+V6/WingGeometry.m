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

if ismethod(adp, 'AR')
    AR = double(adp.AR());          % fetched from ADP department model
else
    AR = Span^2 / WingArea;         % fallback
end
% Kink station (from centreline/root along semi-span), clamped to [0, s].
if isprop(adp, 'KinkPos') && ~isempty(adp.KinkPos)
    y_kink = min(max(double(adp.KinkPos), 0), s);
else
    y_kink = 0;
end

% Piecewise planform:
% - inboard (root->kink): constant chord
% - outboard (kink->tip): linear taper to c_tip = lambda*c_root
% c_root is solved from semi-wing area so the total WingArea is preserved.
S_semi = WingArea / 2;
denom = y_kink + 0.5 * (1 + loc.lambda) * (s - y_kink);
if denom <= 0
    error('WingGeometry:InvalidKink', 'Invalid kink geometry denominator.');
end
c_root = S_semi / denom;
c_tip  = loc.lambda * c_root;
if isprop(adp, 'c_ac') && ~isempty(adp.c_ac)
    MAC = double(adp.c_ac);         % fetched from geometry department output
else
    MAC = (2/3) * c_root * (1 + loc.lambda + loc.lambda^2) / (1 + loc.lambda);
end

chord = zeros(size(y));
inboard = (y <= y_kink);
if s > y_kink
    eta_out = (y(~inboard) - y_kink) / (s - y_kink);  % 0 at kink, 1 at tip
else
    eta_out = zeros(size(y(~inboard)));
end
chord(inboard)  = c_root;
chord(~inboard) = c_root - (c_root - c_tip) .* eta_out;
tc    = loc.tc_tip + (loc.tc_root - loc.tc_tip) .* eta;

h_wb  = tc .* chord;
w_wb  = (loc.fs_aft - loc.fs_fwd) .* chord;
A_enc = h_wb .* w_wb;

if isprop(adp, 'e') && ~isempty(adp.e)
    e_oswald = double(adp.e);       % fetched from aero/global ADP output
else
    e_oswald = 1.78 * (1 - 0.045 * AR^0.68) - 0.64;
end

% LE representation with a geometric kink:
% no inboard sweep up to y_kink, then swept outboard panel.
x_LE    = zeros(size(y));
x_LE(~inboard) = (y(~inboard) - y_kink) .* tan(deg2rad(loc.sweep_LE_deg));
x_ac    = x_LE + 0.25 .* chord;
fa_frac = 0.25 + 0.25 .* eta;   % flexural axis 25% (tip) → 50% (root)
x_fa    = x_LE + fa_frac .* chord;
e_ac_fa = x_ac - x_fa;          % AC–FA offset drives torsion

% Engine configuration from external department outputs at point-of-use.
if isprop(adp,'Engine') && ~isempty(adp.Engine)
    m_eng = double(adp.Engine.Mass);        % kg per engine (rubberised, from PPC)
else
    m_eng = 6850;                   % kg fallback if propulsion is unavailable
end

try
    C = Unconventional.geom.constants();
    N_en = 2 * double(C.N_en);      % total engines from pair count
catch
    N_en = 4;                        % fallback
end
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
G.y_kink       = y_kink;
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

% Geometric wingbox/tank volume from enclosed area integration.
% Integrate root->tip ordering for positive volume.
A_enc_rt = A_enc(end:-1:1);
y_rt     = y(end:-1:1);
V_wb_semi = trapz(y_rt, A_enc_rt);
G.V_wb_semi  = V_wb_semi;
G.V_wb_total = 2 * V_wb_semi;

if isprop(adp, 'Mf_Fuel') && ~isempty(adp.Mf_Fuel)
    Mf = double(adp.Mf_Fuel);
else
    Mf = 0;
end
if isprop(adp, 'Mf_res') && ~isempty(adp.Mf_res)
    Mf_res = double(adp.Mf_res);
else
    Mf_res = 0;
end
M_fuel_total = Mf * double(adp.MTOM);
M_fuel_usable_total = max((1 - Mf_res) * M_fuel_total, 0);
rho_fuel = 800;  % kg/m^3 representative Jet-A density for volume estimate
V_fuel_req_total = M_fuel_usable_total / rho_fuel;
G.V_fuel_req_total = V_fuel_req_total;
if G.V_wb_total > 0
    G.fuel_fill_frac = V_fuel_req_total / G.V_wb_total;
else
    G.fuel_fill_frac = NaN;
end

fprintf('\n--- Wing Geometry ---\n');
fprintf('  Semi-span          %.2f m\n',         s);
fprintf('  Root / tip chord   %.2f / %.2f m\n',  c_root, c_tip);
fprintf('  MAC                %.2f m\n',          MAC);
fprintf('  AR / taper         %.2f / %.2f\n',     AR, loc.lambda);
fprintf('  LE / c/2 sweep     %.1f / %.1f deg\n', loc.sweep_LE_deg, loc.sweep_c2_deg);
fprintf('  Kink station       y = %.2f m  (%.1f%% semi-span)\n', y_kink, 100*y_kink/s);
fprintf('  t/c root / tip     %.3f / %.3f\n',     loc.tc_root, loc.tc_tip);
fprintf('  Oswald e           %.4f\n',             e_oswald);
fprintf('  Wingbox            %.0f-%.0f%% chord\n', loc.fs_fwd*100, loc.fs_aft*100);
fprintf('  Wingbox volume     %.1f m^3 (semi)  %.1f m^3 (total)\n', G.V_wb_semi, G.V_wb_total);
fprintf('  Fuel volume req.   %.1f m^3 total  (fill ratio %.2f)\n', G.V_fuel_req_total, G.fuel_fill_frac);
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