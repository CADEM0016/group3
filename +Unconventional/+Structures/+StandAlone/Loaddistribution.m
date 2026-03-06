function L = LoadDistribution(p, G, loadcase)
% =========================================================================
% LoadDistribution.m  —  +Structures package
% Compute spanwise distributed loads for a given load case.
%
% Covers three CS-25 load cases:
%   '2.5g'   — positive limit manoeuvre  (CS-25.337a, n=+2.5)
%   '1g'     — level flight              (n=+1.0)
%   'neg1g'  — negative limit manoeuvre  (CS-25.337b, n=-1.0)
%
% Physics computed:
%   1. Aerodynamic lift distribution (modified elliptical)
%   2. Wing structural self-weight relief
%   3. Fuel inertia relief  (huge effect — ~145 t at 2.5g MTOW)
%   4. Engine point load
%   5. Net distributed load for SMT integration
%
% INPUT:
%   p         — AircraftParams struct
%   G         — WingGeometry struct
%   loadcase  — string: '2.5g' | '1g' | 'neg1g'
%
% OUTPUT:  L — struct
%   L.lift_dist    [N/m]  aerodynamic lift intensity (upward +ve)
%   L.w_wing       [N/m]  wing self-weight (downward, i.e. relief)
%   L.w_fuel       [N/m]  fuel weight (downward, i.e. relief)
%   L.net_dist     [N/m]  net load = lift - inertia (what drives SMT)
%   L.P_engine     [N]    engine point load (downward for +ve manoeuvre)
%   L.n_limit      [-]    limit load factor used
%   L.n_ult        [-]    ultimate load factor used (n_limit × 1.5)
%   L.L_root       [N/m]  lift intensity at root
%   L.loadcase     string
%
% NO external dependencies.
% =========================================================================

arguments
    p         struct
    G         struct
    loadcase  string = '2.5g'
end

N  = G.N;
dy = G.dy;
g  = p.g;

% -------------------------------------------------------------------------
%  1.  LOAD CASE SELECTION
% -------------------------------------------------------------------------
switch loadcase
    case '2.5g'
        n_lim = p.n_limit_pos;         % +2.5
        n_ult = p.n_ult_pos;           % +3.75
        sign_lift   = +1;              % lift upward
        sign_relief = -1;              % inertia downward (relieves root BM)
    case '1g'
        n_lim =  1.0;
        n_ult =  1.0 * p.SF;           % 1.5
        sign_lift   = +1;
        sign_relief = -1;
    case 'neg1g'
        n_lim = abs(p.n_limit_neg);    % 1.0
        n_ult = abs(p.n_ult_neg);      % 1.5
        sign_lift   = -1;              % lift acts DOWNWARD
        sign_relief = +1;              % inertia now OPPOSES downward load
    otherwise
        error('Structures:LoadDistribution:badCase', ...
              'Unknown load case "%s". Use: 2.5g | 1g | neg1g', loadcase);
end

% -------------------------------------------------------------------------
%  2.  AERODYNAMIC LIFT DISTRIBUTION
%  Modified elliptical distribution (suitable for swept tapered wing).
%  Cooper slide 18: use elliptical as baseline for initial load estimate.
%
%  l(eta) = l_0 * sqrt(1 - eta^2)   where eta = y/s, 0=root, 1=tip
%
%  BUT our G.eta = 0 at TIP, 1 at root — need to remap:
%  Let eta_aero = 1 - G.eta  (0 at root, 1 at tip, standard convention)
%
%  Normalised so: integral_0^s l(y) dy = L_semi
%                 L_semi = n_lim * MTOM * g / 2  (one semi-wing)
% -------------------------------------------------------------------------
L_semi     = n_lim * p.MTOM * g / 2;          % total semi-wing lift [N]

eta_aero   = 1 - G.eta;                        % 0=root, 1=tip
lift_shape = sqrt(max(1 - eta_aero.^2, 0));    % elliptical, max at root

% Numerical normalisation (trapezoidal)
integral_shape = trapz(G.y(end:-1:1), lift_shape(end:-1:1));  % root→tip
if integral_shape < 1e-10
    error('Structures:LoadDistribution:zeroLift', ...
          'Lift shape integral is zero — check G.y and G.eta.');
end
lift_dist = lift_shape .* (L_semi / integral_shape);   % [N/m], tip→root

% -------------------------------------------------------------------------
%  3.  WING SELF-WEIGHT INERTIA RELIEF
%  Distribute wing mass proportional to local chord (heavier inboard).
%  Class I/II estimate used here; replaced by II.5 value in refinement.
%
%  Rough wing mass for relief calculation:
%    m_wing_est ≈ 0.09 * MTOM  (typical 9% for transport a/c)
% -------------------------------------------------------------------------
m_wing_semi  = 0.09 * p.MTOM / 2;             % per semi-wing [kg]

chord_sum    = trapz(G.y(end:-1:1), G.chord(end:-1:1));
if chord_sum < 1e-10; chord_sum = 1; end
chord_norm   = G.chord / chord_sum;           % normalised chord shape
w_wing       = m_wing_semi .* chord_norm;     % [kg/m] structural mass/span
w_wing_load  = n_ult * w_wing * g;            % [N/m] inertia load (ult)

% -------------------------------------------------------------------------
%  4.  FUEL INERTIA RELIEF
%  ~97% of fuel in wing tanks (B777-class integral tank).
%  Distribute proportional to wingbox cross-section area (= fuel volume).
%
%  This is the DOMINANT relief load at 2.5g MTOW:
%    Fuel = 0.19 * 348700 = 66,253 kg → 66 t × 2.5g = 1.6 MN per semi-wing
% -------------------------------------------------------------------------
m_fuel_semi  = 0.97 * p.M_fuel / 2;           % per semi-wing [kg]

vol_shape    = G.A_enc;                        % ∝ local wingbox volume
vol_integral = trapz(G.y(end:-1:1), vol_shape(end:-1:1));
if vol_integral < 1e-10; vol_integral = 1; end
vol_norm     = vol_shape / vol_integral;
w_fuel       = m_fuel_semi .* vol_norm;        % [kg/m]
w_fuel_load  = n_ult * w_fuel * g;             % [N/m] inertia load (ult)

% -------------------------------------------------------------------------
%  5.  NET DISTRIBUTED LOAD
%  Sign convention (Cooper slide 17):
%    Upward force on wing = positive
%    2.5g: lift up (+), weight/fuel down (−)  → net = lift − relief
%    neg1g: lift down (−), weight/fuel up (+) → net = −lift + relief
% -------------------------------------------------------------------------
net_dist = sign_lift * lift_dist + sign_relief * (w_wing_load + w_fuel_load);

% -------------------------------------------------------------------------
%  6.  ENGINE POINT LOAD
%  One engine per semi-wing at y_engine.
%  Acts DOWNWARD in both positive and negative manoeuvres (mass × n × g).
%  In 2.5g: acts downward → RELIEVES root BM (engine outboard of root)
%  In neg1g: also acts downward → but wing bends down, so engine adds to BM
%  Store as unsigned scalar; SMT module applies correct sign.
% -------------------------------------------------------------------------
P_engine = n_ult * p.m_engine_each * g;        % [N], magnitude

% -------------------------------------------------------------------------
%  PACK OUTPUT
% -------------------------------------------------------------------------
L.lift_dist    = lift_dist;       % [N/m], tip→root array
L.w_wing       = w_wing_load;     % [N/m]
L.w_fuel       = w_fuel_load;     % [N/m]
L.net_dist     = net_dist;        % [N/m]
L.P_engine     = P_engine;        % [N], scalar
L.n_limit      = n_lim;
L.n_ult        = n_ult;
L.sign_lift    = sign_lift;
L.sign_relief  = sign_relief;
L.L_semi       = L_semi;
L.L_root       = lift_dist(G.N);  % lift intensity at root station
L.loadcase     = loadcase;
L.m_wing_semi  = m_wing_semi;
L.m_fuel_semi  = m_fuel_semi;

% -------------------------------------------------------------------------
%  PRINT SUMMARY
% -------------------------------------------------------------------------
fprintf('\n--- Load Distribution: %s ---\n', loadcase);
fprintf('  n_limit = %.1f  |  n_ult = %.2f\n', n_lim, n_ult);
fprintf('  Semi-wing lift:      %8.3f MN\n',   L_semi/1e6);
fprintf('  Wing inertia relief: %8.3f MN\n',   trapz(flip(G.y), flip(w_wing_load))/1e6);
fprintf('  Fuel  inertia relief:%8.3f MN\n',   trapz(flip(G.y), flip(w_fuel_load))/1e6);
fprintf('  Engine point load:   %8.3f MN\n',   P_engine/1e6);
fprintf('  Relief as %% of lift: %6.1f%%\n', ...
        trapz(flip(G.y), flip(w_wing_load+w_fuel_load)) / ...
        trapz(flip(G.y), flip(lift_dist)) * 100);

end