%% Aerodynamics_Interface — Wing Structures outputs for Role 4

% ── Oswald efficiency for drag polar ─────────────────────────────────
e_oswald   = WS.e_oswald;         % —   use in  CDi = CL²/(pi·AR·e)
AR         = WS.AR;               % —   aspect ratio

% ── Spanwise aerodynamic centres ──────────────────────────────────────
y          = WS.y;                % m   spanwise stations (tip→root)
x_ac       = WS.x_ac;            % m   AC x-position at each station
x_fa       = WS.x_fa;            % m   flexural axis x-position
e_ac_fa    = WS.e_ac_fa;         % m   AC–FA torque arm (drives aeroelastic wash-out)

% ── Chord distribution ────────────────────────────────────────────────
chord      = WS.chord;            % m   c(y) for load distribution refinement
MAC        = WS.MAC;              % m   mean aerodynamic chord
c_root     = WS.c_root;          % m
c_tip      = WS.c_tip;           % m

% ── Baseline spanwise loading (elliptic, 2.5g) ───────────────────────
lift_dist_norm = WS.lift_dist_25g;   % normalised BM shape from 2.5g case

% ── Stiffness for aeroelastic coupling ────────────────────────────────
EI         = WS.EI;               % Nm²  bending stiffness full span
GJ         = WS.GJ;               % Nm²  torsional stiffness full span
GJ_over_EI = WS.GJ_over_EI;      % —    torsional/bending stiffness ratio

fprintf('\n--- Wing Structures → Aerodynamics handoff ---\n');
fprintf('  Oswald e               %8.4f\n', e_oswald);
fprintf('  Aspect ratio           %8.2f\n', AR);
fprintf('  MAC                    %8.2f m\n', MAC);
fprintf('  GJ/EI at root          %8.4f\n', GJ_over_EI);
fprintf('  Note: GJ/EI < 0.2 → aileron reversal risk; review with Aero\n');
fprintf('  Spanwise stations      %d  (y array in WS.y)\n', length(y));
fprintf('  AC positions available in WS.x_ac (%d values)\n', length(x_ac));
