function loc = AircraftParams()

% Wing planform
loc.Span_flight  = 79.75;  % flight span
loc.Span_taxi    = 65.0;   % Code E taxi limit with folding tip
loc.y_hinge      = 32.5;   % fold hinge from centreline
loc.lambda       = 0.25;   % taper ratio
loc.sweep_LE_deg = 33.5;   % deg
loc.sweep_c4_deg = 31.1;   % deg  USAF weight method
loc.sweep_c2_deg = 27.8;   % deg  Raymer / Torenbeek
loc.tc_root      = 0.153;
loc.tc_tip       = 0.108;

% Wingbox
loc.fs_fwd      = 0.15;    % front spar / chord
loc.fs_aft      = 0.60;    % rear spar / chord
loc.rib_spacing = 0.65;    
loc.k_buckle    = 4.0;     % simply-supported panel coefficient

% Folding wingtip structural constraints
loc.b_tip_fold   = 7.375;  % folding outer panel length
loc.n_limit_fold = 1.5;    % fold hinge design limit load factor
loc.hinge_t_min  = 0.010;  % minimum hinge pin web thickness
loc.hinge_sig    = 250e6;  % hinge bearing stress allowable (titanium)
loc.hinge_tau    = 145e6;  % hinge pin shear allowable (titanium)
loc.f_lock_mech  = 0.05;   % lock mechanism mass / outer panel primary
loc.f_actuator   = 0.08;   % fold actuator mass / outer panel primary

% Engine configuration 4 engines total, 2 per semi-wing
loc.T_static_each = 311e3; 
loc.m_engine_each = 6850;  
loc.N_engines     = 4;     % total engines (2 per semi-wing)
% Inner/outer engine spanwise positions as fraction of semi-span.
loc.y_eng1_frac   = 0.35;  % inner engine station / semi-span
loc.y_eng2_frac   = 0.55;  % outer engine station / semi-span

% CS-25 load factors
loc.n_limit_pos = 2.5;     % CS-25.337
loc.n_limit_neg = -1.0;    % CS-25.337
loc.SF          = 1.5;     % CS-25.303  limit → ultimate
loc.U_de_B      = 20.0;    % m/s  CS-25.341
loc.U_de_C      = 16.0;    % m/s
loc.U_de_D      =  8.0;    % m/s

% Aluminium 7010-T7451  (A380 primary wing material)
loc.Al.E         = 70.3e9;  % Pa
loc.Al.G         = 26.5e9;  % Pa
loc.Al.nu        = 0.33;
loc.Al.rho       = 2820;    % kg/m³
loc.Al.sig_ult   = 490e6;   % Pa
loc.Al.sig_all   = 270e6;   % Pa
loc.Al.tau_all   = 156e6;   % Pa
loc.Al.t_min     = 0.002;   % m
loc.Al.t_web_min = 0.003;   % m
loc.Al.A_cap_min = 1.0e-4;  % m²

% CFRP quasi-isotropic  E = 3/8 E_uni
loc.CF.E         = (3/8) * 135e9; % Pa
loc.CF.G         = (3/8) *  50e9; % Pa
loc.CF.nu        = 0.33;
loc.CF.rho       = 1550;    % kg/m³
loc.CF.sig_ult   = 600e6;   % Pa
loc.CF.sig_all   = 220e6;   % Pa  post-BVID knockdown CS-25.571
loc.CF.tau_all   = 110e6;   % Pa
loc.CF.t_min     = 0.002;   % m
loc.CF.t_web_min = 0.003;   % m
loc.CF.A_cap_min = 1.0e-4;  % m²

% Mass allowances
loc.f_secondary  = 0.35;   % secondary / primary wingbox
loc.f_hinge_mech = 0.10;   % hinge mechanism penalty / outer panel primary
loc.m_hinge_min  = 2000;   % kg  minimum fold hinge assembly floor

% Discretisation
loc.N_stations = 150;

% ISA sea-level constants  (SI only carries g)
loc.R_air      = 287.058;  % J/(kg·K)
loc.gamma_air  = 1.4;
loc.T0         = 288.15;   % K
loc.P0         = 101325;   % Pa
loc.rho0       = 1.225;    % kg/m³
loc.lapse_trop = 0.0065;   % K/m
loc.H_trop     = 11000;    % m

end