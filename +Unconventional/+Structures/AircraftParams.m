function loc = AircraftParams()
% Structures-specific constants not available in any team global file.
% Global values (MTOM, Span, Mf_fuel, M_c, etc.) are read directly
% from ADP, TLAR, SI, and cast.atmos in each module.

% Wing geometry
loc.Span_taxi    = 65.0;    % m    Code E taxi wingspan limit
loc.y_hinge      = 32.5;    % m    fold hinge position from centreline
loc.lambda       = 0.30;    % —    taper ratio c_tip / c_root
loc.sweep_LE_deg = 34.0;    % deg
loc.sweep_c4_deg = 31.6;    % deg  quarter-chord, used in USAF weight method
loc.sweep_c2_deg = 28.0;    % deg  half-chord, used in Raymer / Torenbeek
loc.tc_root      = 0.150;   % —    thickness-to-chord at root
loc.tc_tip       = 0.110;   % —    thickness-to-chord at tip

% Wingbox
loc.fs_fwd      = 0.15;     % —    front spar as fraction of local chord
loc.fs_aft      = 0.60;     % —    rear spar as fraction of local chord
loc.rib_spacing = 0.60;     % m    governs skin buckling check
loc.k_buckle    = 4.0;      % —    simply-supported panel buckling coefficient

% Engine (thrust and mass not in any global file)
loc.T_static_each = 513e3;  % N    GE90-115B class
loc.m_engine_each = 8762;   % kg

% CS-25 load factors
loc.n_limit_pos = 2.5;      % —    CS-25.337
loc.n_limit_neg = -1.0;     % —    CS-25.337
loc.SF          = 1.5;      % —    CS-25.303  limit → ultimate
loc.U_de_B      = 20.0;     % m/s  CS-25.341 gust at V_B  (Sprint 3)
loc.U_de_C      = 16.0;     % m/s  gust at V_C
loc.U_de_D      =  8.0;     % m/s  gust at V_D

% Aluminium 7075-T6
loc.Al.E         = 71.0e9;  % Pa
loc.Al.G         = 26.9e9;  % Pa
loc.Al.nu        = 0.33;
loc.Al.rho       = 2780;    % kg/m³
loc.Al.sig_ult   = 503e6;   % Pa   ultimate tensile strength
loc.Al.sig_all   = 275e6;   % Pa   allowable (ult / 1.83)
loc.Al.tau_all   = 159e6;   % Pa   allowable shear
loc.Al.t_min     = 0.002;   % m    minimum skin gauge
loc.Al.t_web_min = 0.003;   % m    minimum web gauge
loc.Al.A_cap_min = 1.0e-4;  % m²   minimum spar cap area

% CFRP quasi-isotropic  (E = 3/8 E_uni, Cooper black-metal rule)
loc.CF.E         = (3/8) * 135e9; % Pa
loc.CF.G         = (3/8) *  50e9; % Pa
loc.CF.nu        = 0.33;
loc.CF.rho       = 1550;    % kg/m³
loc.CF.sig_ult   = 600e6;   % Pa
loc.CF.sig_all   = 220e6;   % Pa   post-BVID knockdown, CS-25.571
loc.CF.tau_all   = 110e6;   % Pa
loc.CF.t_min     = 0.002;   % m
loc.CF.t_web_min = 0.003;   % m
loc.CF.A_cap_min = 1.0e-4;  % m²

% Secondary structure and hinge allowances
loc.f_secondary  = 0.35;    % —    secondary mass as fraction of primary wingbox
loc.f_hinge_mech = 0.10;    % —    hinge penalty as fraction of outer panel mass
loc.m_hinge_min  = 1500;    % kg   777X programme floor

% Discretisation
loc.N_stations = 150;

% ISA sea-level constants  (SI only carries g)
loc.R_air      = 287.058;   % J/(kg·K)
loc.gamma_air  = 1.4;
loc.T0         = 288.15;    % K
loc.P0         = 101325;    % Pa
loc.rho0       = 1.225;     % kg/m³
loc.lapse_trop = 0.0065;    % K/m
loc.H_trop     = 11000;     % m

end
