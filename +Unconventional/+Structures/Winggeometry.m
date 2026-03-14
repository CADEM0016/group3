function G = WingGeometry(p, N_override)
% =========================================================================
% Compute wing geometry at every spanwise station.
% INPUT:
%   p          — parameter struct from AircraftParams()
%   N_override — (optional) override number of stations
%
% OUTPUT:  G — struct with fields at N spanwise stations (tip → root):
%   G.y        [m]    spanwise position  (y(1)=tip, y(N)=root)
%   G.eta      [-]    normalised station  0=tip, 1=root
%   G.chord    [m]    local chord length
%   G.tc       [-]    local thickness-to-chord ratio
%   G.h_wb     [m]    wingbox height  = tc * chord
%   G.w_wb     [m]    wingbox width   = wb_frac * chord
%   G.A_enc    [m^2]  wingbox enclosed area  = h_wb * w_wb
%   G.x_LE     [m]    leading-edge x-offset from root LE (due to sweep)
%   G.x_ac     [m]    aerodynamic centre x-position from root LE
%   G.x_fa     [m]    flexural axis x-position from root LE
%   G.e_ac_fa  [m]    offset: x_ac - x_fa  (positive = AC aft of FA)
%   G.s        [m]    semi-span
%   G.N        [-]    number of stations
%   G.dy       [m]    station spacing
%   G.i_hinge  [-]    station index nearest to fold hinge
%   G.i_engine [-]    station index nearest to engine
% =========================================================================

arguments
    p          struct
    N_override double = 0
end

N  = p.N_stations;
if N_override > 0; N = N_override; end

s  = p.Span / 2;                    % semi-span [m]
dy = s / (N - 1);                   % station spacing [m]

% Spanwise stations: tip (y=s) → root (y=0)
y   = linspace(s, 0, N);
eta = 1 - y/s;                      % normalised: 0 at tip, 1 at root

% -------------------------------------------------------------------------
%  CHORD  —  linear taper
% -------------------------------------------------------------------------
chord = p.c_tip + (p.c_root - p.c_tip) .* eta;

% -------------------------------------------------------------------------
%  THICKNESS-TO-CHORD RATIO  —  linear spanwise variation
% -------------------------------------------------------------------------
tc = p.tc_tip + (p.tc_root - p.tc_tip) .* eta;

% -------------------------------------------------------------------------
%  WINGBOX CROSS-SECTION DIMENSIONS
% -------------------------------------------------------------------------
h_wb  = tc    .* chord;                      % box height  [m]
w_wb  = p.wb_frac .* chord;                  % box width   [m]
A_enc = h_wb  .* w_wb;                       % enclosed area [m^2]

% -------------------------------------------------------------------------
%  SWEEP OFFSETS
% -------------------------------------------------------------------------
sweep_LE = deg2rad(p.sweep_LE_deg);
sweep_c2 = deg2rad(p.sweep_c2_deg);
sweep_c4 = deg2rad(p.sweep_c4_deg);

% x-coordinate of local leading edge relative to root LE
x_LE = (s - y) .* tan(sweep_LE);

% Aerodynamic centre at quarter chord (subsonic thin aerofoil theory)
x_ac = x_LE + 0.25 .* chord;

% Flexural axis (elastic axis) position
fa_frac = 0.25 + 0.25 .* eta;          % fraction of local chord
x_fa    = x_LE + fa_frac .* chord;

% Offset between AC and FA (drives torque calculation)
e_ac_fa = x_ac - x_fa;                 % positive when AC is AFT of FA

% -------------------------------------------------------------------------
%  NEAREST STATION INDICES
% -------------------------------------------------------------------------
[~, i_hinge] = min(abs(y - p.y_hinge));
[~, i_engine]= min(abs(y - p.y_engine));

% -------------------------------------------------------------------------
%  PACK OUTPUT
% -------------------------------------------------------------------------
G.y        = y;
G.eta      = eta;
G.chord    = chord;
G.tc       = tc;
G.h_wb     = h_wb;
G.w_wb     = w_wb;
G.A_enc    = A_enc;
G.x_LE     = x_LE;
G.x_ac     = x_ac;
G.x_fa     = x_fa;
G.e_ac_fa  = e_ac_fa;
G.s        = s;
G.N        = N;
G.dy       = dy;
G.i_hinge  = i_hinge;
G.i_engine = i_engine;
G.c_root   = p.c_root;
G.c_tip    = p.c_tip;
G.AR       = p.AR;
G.sweep_LE_deg = p.sweep_LE_deg;
G.sweep_c2_deg = p.sweep_c2_deg;

% -------------------------------------------------------------------------
%  PRINT SUMMARY
% -------------------------------------------------------------------------
fprintf('\n--- Wing Geometry ---\n');
fprintf('  Semi-span:        %.2f m\n',   s);
fprintf('  Root chord:       %.2f m\n',   p.c_root);
fprintf('  Tip  chord:       %.2f m\n',   p.c_tip);
fprintf('  Aspect ratio:     %.2f\n',     p.AR);
fprintf('  Taper ratio:      %.2f\n',     p.lambda);
fprintf('  LE sweep:         %.1f deg\n', p.sweep_LE_deg);
fprintf('  c/2 sweep:        %.1f deg\n', p.sweep_c2_deg);
fprintf('  t/c root:         %.3f\n',     p.tc_root);
fprintf('  t/c tip:          %.3f\n',     p.tc_tip);
fprintf('  Wingbox frac:     %.2f (%.0f%%-%.0f%% chord)\n', ...
        p.wb_frac, p.fs_fwd*100, p.fs_aft*100);
fprintf('  Fold hinge @ y =  %.1f m  (station %d)\n', p.y_hinge, i_hinge);
fprintf('  Engine     @ y =  %.1f m  (station %d)\n', p.y_engine, i_engine);
fprintf('  Stations:         %d  (dy = %.3f m)\n', N, dy);

end