function p = AircraftParams()
% =========================================================================
% AircraftParams.m  —  +Structures package
% Single source of truth for ALL parameters used across every module.
% Edit values HERE only. Every other file reads from this struct.
%
% NO external dependencies — runs completely standalone.
% =========================================================================

% -------------------------------------------------------------------------
%  TOP-LEVEL AIRCRAFT REQUIREMENTS
% -------------------------------------------------------------------------
p.MTOM        = 348700;     % Max take-off mass [kg]  — B777F baseline
p.OEM         = 145000;     % Operating empty mass [kg]
p.Payload     = 103000;     % Design payload [kg]
p.Mf_fuel     = 0.19;       % Fuel mass fraction of MTOM [-]
p.M_fuel      = p.Mf_fuel * p.MTOM;  % Total fuel mass [kg]
p.M_cruise    = 0.84;       % Cruise Mach number [-]
p.Range       = 9200e3;     % Design range [m]  (9200 km)

% -------------------------------------------------------------------------
%  WING GEOMETRY  —  Unconventional wide-body + folding wingtips
% -------------------------------------------------------------------------
p.Span        = 72.0;       % Total FLIGHT wingspan [m]  (Code F ≤ 80 m)
p.Span_taxi   = 65.0;       % Max TAXI wingspan [m]      (Code E ≤ 65 m)
p.y_hinge     = 32.5;       % Fold hinge semi-span position [m from CL]
%   Taxi span check: 2 * y_hinge = 65 m  ✓  Code E compliant

p.WingArea    = 436.8;      % Reference wing area [m^2]  — B777F value
p.AR          = p.Span^2 / p.WingArea;   % Aspect ratio [-]
p.lambda      = 0.30;       % Taper ratio (c_tip / c_root) [-]
p.sweep_LE_deg = 34.0;      % Leading edge sweep [deg]
p.sweep_c4_deg = 31.6;      % Quarter-chord sweep [deg]
p.sweep_c2_deg = 28.0;      % Half-chord sweep [deg]
p.tc_root     = 0.150;      % Root thickness-to-chord ratio [-]
p.tc_tip      = 0.110;      % Tip  thickness-to-chord ratio [-]
p.KinkPos     = 10.0;       % TE kink position [m from CL]  — planform kink
%   NOTE: fold hinge (y_hinge=32.5 m) is OUTBOARD of TE kink (10 m)

% Derived chord lengths
p.c_root = 2 * p.WingArea / (p.Span * (1 + p.lambda));  % root chord [m]
p.c_tip  = p.lambda * p.c_root;                          % tip chord [m]

% -------------------------------------------------------------------------
%  ENGINE  —  GE90-115B class (2 engines, one per wing)
% -------------------------------------------------------------------------
p.N_engines     = 2;
p.T_static_each = 513e3;    % Static thrust per engine [N]  (115,300 lbf)
p.m_engine_each = 8762;     % Mass per engine [kg]  (19,315 lb)
p.y_engine      = 0.35 * p.Span/2;  % Engine spanwise position [m from CL]

% -------------------------------------------------------------------------
%  LOAD CASES  —  CS-25 / GDP Specification
% -------------------------------------------------------------------------
p.n_limit_pos =  2.5;       % Positive limit load factor  (CS-25.337)
p.n_limit_neg = -1.0;       % Negative limit load factor  (CS-25.337)
p.SF          =  1.5;       % Safety factor limit → ultimate  (CS-25.303)
p.n_ult_pos   =  3.75;      % Ultimate positive  = 2.5 × 1.5
p.n_ult_neg   = -1.50;      % Ultimate negative  = 1.0 × 1.5

% Gust velocities (CS-25.341, for Sprint 3 refinement)
p.U_de_B      = 20.0;       % Design gust velocity at V_B [m/s]
p.U_de_C      = 16.0;       % Design gust velocity at V_C [m/s]
p.U_de_D      =  8.0;       % Design gust velocity at V_D [m/s]

% -------------------------------------------------------------------------
%  WINGBOX GEOMETRY (fractions of local chord)
% -------------------------------------------------------------------------
p.fs_fwd      = 0.15;       % Front spar chord position
p.fs_aft      = 0.60;       % Rear spar chord position
p.wb_frac     = p.fs_aft - p.fs_fwd;  % Wingbox chord fraction = 0.45

% -------------------------------------------------------------------------
%  MATERIALS
% -------------------------------------------------------------------------
% Aluminium 7075-T6  (primary structural material)
p.Al.E        = 71.0e9;     % Young's modulus [Pa]
p.Al.G        = 26.9e9;     % Shear modulus [Pa]
p.Al.rho      = 2780;       % Density [kg/m^3]
p.Al.sig_ult  = 503e6;      % Ultimate tensile strength [Pa]
p.Al.sig_all  = 275e6;      % Allowable stress (ult/1.83) [Pa]
p.Al.tau_all  = 159e6;      % Allowable shear stress [Pa]
p.Al.t_min    = 0.002;      % Minimum skin gauge [m]  (2 mm)
p.Al.t_web_min= 0.003;      % Minimum web gauge [m]   (3 mm)
p.Al.A_cap_min= 1.0e-4;     % Minimum spar cap area [m^2]  (100 mm^2)

% CFRP quasi-isotropic laminate  (2040 EIS trade study)
% E = 3/8 × E_unidirectional  (Cooper "black metal" approximation)
p.CF.E        = 3/8 * 135e9;
p.CF.G        = 3/8 *  50e9;
p.CF.rho      = 1550;
p.CF.sig_ult  = 600e6;
p.CF.sig_all  = 220e6;    % Post-knockdown allowable (BVID, CS-25.571)
p.CF.tau_all  = 110e6;    % Shear allowable scaled accordingly
p.CF.t_min    = 0.002;
p.CF.t_web_min= 0.003;
p.CF.A_cap_min= 1.0e-4;

% -------------------------------------------------------------------------
%  MASS FRACTIONS  (secondary structure allowances)
% -------------------------------------------------------------------------
p.f_secondary  = 0.35;      % Secondary structure on primary wingbox
%   Covers: ribs, LE/TE, control surfaces, brackets, sealant
p.f_hinge_mech = 0.10;    % keep fraction
% AND add a minimum floor in EmpiricalMass.m and MassBuildup.m:
p.m_hinge_min  = 1500;    % kg minimum per aircraft (both wings)
                           % based on 777X programme data%   Covers: actuator, lock, structural doublers, fairing

% -------------------------------------------------------------------------
%  DISCRETISATION
% -------------------------------------------------------------------------
p.N_stations  = 150;        % Spanwise integration stations (tip → root)

% -------------------------------------------------------------------------
%  ATMOSPHERE (ISA standard — private copy, no cast.atmos dependency)
% -------------------------------------------------------------------------
p.g           = 9.80665;    % Gravity [m/s^2]
p.R_air       = 287.058;    % Specific gas constant for air [J/kg/K]
p.gamma_air   = 1.4;        % Ratio of specific heats
p.T0          = 288.15;     % ISA sea-level temperature [K]
p.P0          = 101325;     % ISA sea-level pressure [Pa]
p.rho0        = 1.2250;     % ISA sea-level density [kg/m^3]
p.lapse_trop  = 0.0065;     % Troposphere lapse rate [K/m]
p.H_trop      = 11000;      % Tropopause altitude [m]

end