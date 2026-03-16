function p = AircraftParams()

% ---- Aircraft identity
p.MTOM        = 348700;
p.OEM         = 145000;
p.Payload     = 103000;
p.W_useful    = p.MTOM - p.OEM;            % Snorri Ch6: Wu = W0 - We

% ---- Fuel
p.Mf_fuel     = 0.19;
p.M_fuel      = p.Mf_fuel * p.MTOM;
p.Mf_res      = 0.05;                      % reserve fuel fraction of total fuel
p.M_fuel_res  = p.Mf_res  * p.M_fuel;

% ---- Mission
p.M_cruise    = 0.84;
p.Range       = 9200e3;
p.Alt_cruise  = 10668;                     % 35,000 ft [m]

% ---- Wing planform  (Snorri Ch9)
p.Span        = 72.0;
p.Span_taxi   = 65.0;
p.WingArea    = 436.8;
p.AR          = p.Span^2 / p.WingArea;     % Snorri: AR = b^2 / S
p.lambda      = 0.30;                      % Snorri: lambda = c_tip / c_root
p.c_root      = 2*p.WingArea / (p.Span*(1+p.lambda));
p.c_tip       = p.lambda * p.c_root;
p.MAC         = (2/3)*p.c_root * (1 + p.lambda + p.lambda^2) / (1 + p.lambda);

% ---- Sweep angles
p.sweep_LE_deg = 34.0;
p.sweep_c4_deg = 31.6;
p.sweep_c2_deg = 28.0;

% ---- Aerofoil
p.tc_root     = 0.150;
p.tc_tip      = 0.110;

% ---- Folding wingtip geometry
p.y_hinge     = 32.5;                      % fold hinge semi-span [m]
p.KinkPos     = 10.0;

% ---- Wingbox spar locations (fraction of local chord)
p.fs_fwd      = 0.15;
p.fs_aft      = 0.60;
p.wb_frac     = p.fs_aft - p.fs_fwd;      % = 0.45

% ---- Engine  (GE90-115B class)
p.N_engines     = 2;
p.T_static_each = 513e3;
p.m_engine_each = 8762;
p.y_engine      = 0.35 * p.Span/2;

% ---- CS-25 load factors (Snorri Ch6 / CS-25.337 and .303)
p.n_limit_pos = 2.5;
p.n_limit_neg = -1.0;
p.SF          = 1.5;
p.n_ult_pos   = p.n_limit_pos * p.SF;     % = 3.75
p.n_ult_neg   = p.n_limit_neg * p.SF;     % = -1.50

% ---- Gust velocities CS-25.341 (Sprint 3)
p.U_de_B      = 20.0;
p.U_de_C      = 16.0;
p.U_de_D      =  8.0;

% ---- Cruise dynamic pressure  (Snorri Ch6: q = 0.5*rho*V^2)
p.rho_cruise  = 0.3796;                   % ISA density at 35,000 ft [kg/m^3]
p.a_cruise    = 295.07;                   % speed of sound at 35,000 ft [m/s]
p.V_cruise    = p.M_cruise * p.a_cruise;
p.q_cruise    = 0.5 * p.rho_cruise * p.V_cruise^2;

% ---- Materials: Aluminium 7075-T6
p.Al.E        = 71.0e9;
p.Al.G        = 26.9e9;
p.Al.nu       = 0.33;
p.Al.rho      = 2780;
p.Al.sig_ult  = 503e6;
p.Al.sig_all  = 275e6;
p.Al.tau_all  = 159e6;
p.Al.t_min    = 0.002;
p.Al.t_web_min = 0.003;
p.Al.A_cap_min = 1.0e-4;

% ---- Materials: CFRP quasi-isotropic  (3/8 rule, Cooper)
p.CF.E        = (3/8) * 135e9;
p.CF.G        = (3/8) *  50e9;
p.CF.nu       = 0.33;
p.CF.rho      = 1550;
p.CF.sig_ult  = 600e6;
p.CF.sig_all  = 220e6;
p.CF.tau_all  = 110e6;
p.CF.t_min    = 0.002;
p.CF.t_web_min = 0.003;
p.CF.A_cap_min = 1.0e-4;

% ---- Rib spacing (for skin buckling check — Snorri Ch5)
p.rib_spacing  = 0.60;                    % [m]  nominal rib pitch
p.k_buckle     = 4.0;                     % buckling coefficient, simply supported

% ---- Secondary structure and hinge fractions
p.f_secondary  = 0.35;
p.f_hinge_mech = 0.10;
p.m_hinge_min  = 1500;

% ---- Spanwise discretisation
p.N_stations  = 150;

% ---- ISA sea-level constants
p.g           = 9.80665;
p.R_air       = 287.058;
p.gamma_air   = 1.4;
p.T0          = 288.15;
p.P0          = 101325;
p.rho0        = 1.2250;
p.lapse_trop  = 0.0065;
p.H_trop      = 11000;

end
