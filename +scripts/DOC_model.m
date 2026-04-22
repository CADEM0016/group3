function results = DOC_model(ADP, BlockFuel, MissionTime)
% ==========================================================
% DIRECT OPERATING COST MODEL
% ==========================================================
% Inputs from ExampleUnconventionalEmissionConnected:
%   ADP         - sized aircraft object
%   BlockFuel   - fuel used per mission [kg]
%   MissionTime - mission duration [hr]
%
% Output:
%   results     - structure containing DOC results

%% =============================
% 1. INPUTS FROM AIRCRAFT MODEL
% ==============================

% Aircraft
MTOM = ADP.MTOM;          % kg
V    = ADP.TLAR.M_c;      % Mach
N    = 6;                 % number of aircraft

% Mission / annual utilisation
missions_per_year = 18;
t_mission   = MissionTime;                    % hr
Annual_hours = missions_per_year * t_mission; % hr/year

% Fuel
m_fuel_mission = BlockFuel;   % kg per mission
fuel_price     = 1;           % $/kg

% Financial
r_int  = 0.05;    % interest rate
r_ins  = 0.02;    % insurance rate
life   = 14;      % years

% Residual value
residual_fraction = 0.1;

% Airport
landing_fee = 12446;          % per flight
flights_yr  = missions_per_year;
parking     = 1460000;        % per year

% Labour rates
R_eng  = 100;
R_tool = 80;
R_mfg  = 60;
R_sup  = 50;

production_factor = 1;

%% =============================
% 2. MANUFACTURING COST MODEL
% ==============================

% Labour hours (CERs)
H_eng  = 5.18 * (MTOM^0.777) * (V^0.163) * (N^0.163);
H_tool = 7.22 * (MTOM^0.777) * (V^0.696) * (N^0.263);
H_mfg  = 10.5 * (MTOM^0.82)  * (V^0.484) * (N^0.641);

% Support hours
H_sup = 0.076 * H_mfg;

% Convert to cost
C_eng  = H_eng  * R_eng;
C_tool = H_tool * R_tool;
C_mfg  = H_mfg  * R_mfg;
C_sup  = H_sup  * R_sup;

% Total aircraft production cost
C_aircraft = (C_eng + C_tool + C_mfg + C_sup) * production_factor;

% Fleet cost
C_fleet = C_aircraft * N;

%% =============================
% 3. OPERATING COSTS
% ==============================

% Crew
C_crew = 4 * 150000;

% Fuel
m_dot_fuel    = m_fuel_mission / t_mission;   % kg/hr
m_fuel_annual = m_dot_fuel * Annual_hours;    % kg/year
C_fuel        = m_fuel_annual * fuel_price;   % $/year

% Maintenance
C_maint_airframe = 0.03  * C_aircraft;
C_maint_engine   = 5e-6  * C_aircraft * fuel_price;
C_maint_material = 0.006 * C_aircraft;

C_maint = C_maint_airframe + C_maint_engine + C_maint_material;

% Airport costs
C_landing = landing_fee * flights_yr;
C_parking = parking;

%% =============================
% 4. FINANCIAL COSTS
% ==============================

% Residual value
C_residual = residual_fraction * C_aircraft;

% Insurance & interest
C_ins = r_ins * C_aircraft;
C_int = r_int * C_aircraft;

% Depreciation
C_dep = (C_aircraft - C_residual) / life;

%% =============================
% 5. DIRECT OPERATING COST
% ==============================

DOC = C_crew + C_fuel + C_maint + ...
      C_landing + C_parking + ...
      C_ins + C_int + C_dep;

DOC_fleet = DOC * N;

%% =============================
% 6. OUTPUT RESULTS
% ==============================

results.C_aircraft = C_aircraft;
results.C_fleet    = C_fleet;

results.C_crew     = C_crew;
results.C_fuel     = C_fuel;
results.C_maint    = C_maint;
results.C_landing  = C_landing;
results.C_parking  = C_parking;
results.C_ins      = C_ins;
results.C_int      = C_int;
results.C_dep      = C_dep;

results.DOC        = DOC;
results.DOC_fleet  = DOC_fleet;

results.MTOM           = MTOM;
results.Mach           = V;
results.missions_per_year = missions_per_year;
results.t_mission      = t_mission;
results.Annual_hours   = Annual_hours;
results.m_fuel_mission = m_fuel_mission;
results.m_dot_fuel     = m_dot_fuel;
results.m_fuel_annual  = m_fuel_annual;
results.fuel_price     = fuel_price;
end