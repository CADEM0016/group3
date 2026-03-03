%% ==========================================================
%  FULL DIRECT OPERATING COST (DOC) MODEL - USD VERSION
% ===========================================================

clc; clear; close all;

%% =============================
% 1. CURRENCY SETTINGS
% ==============================

GBP_to_USD = 1.23;   % <-- EDIT exchange rate if needed

%% =============================
% 2. INPUT PARAMETERS
% ==============================

% -------- Fleet --------
N_aircraft = 6;

% -------- Aircraft --------
MTOM      = 497870;      % kg
Velocity  = 0.82;        % Mach
Annual_hours = 4000;     % hr/year

% -------- Fuel --------
fuel_volume_per_hr = 6250; % L/hr
fuel_density       = 0.84; % kg/L
fuel_price_GBP     = 0.80; % £/kg
fuel_price         = fuel_price_GBP * GBP_to_USD; % Convert to USD

% -------- Financial --------
interest_rate      = 0.05;   
insurance_rate     = 0.02;   
depreciation_years = 20;

% -------- Airport --------
landing_fee_GBP       = 12446;
parking_cost_GBP      = 1460000;

landing_fee_per_flight = landing_fee_GBP * GBP_to_USD;
parking_cost_year      = parking_cost_GBP * GBP_to_USD;

flights_per_year = 365;

% -------- Labour Rates --------
labour_rate_eng  = 100 * GBP_to_USD;
labour_rate_tool = 80  * GBP_to_USD;
labour_rate_mfg  = 60  * GBP_to_USD;
labour_rate_sup  = 50  * GBP_to_USD;

production_factor = 1;

%% =============================
% 3. MANUFACTURING COST MODEL
% ==============================

C1 = 44880*(MTOM^0.65);

H_eng  = 5.18*(MTOM^0.777)*(Velocity^0.163)*(N_aircraft^0.163);
H_tool = 7.22*(MTOM^0.777)*(Velocity^0.696)*(N_aircraft^0.263);
H_mfg  = 10.5*(MTOM^0.82)*(Velocity^0.484)*(N_aircraft^0.641);

H_sup  = 0.076 * H_mfg;

C_eng  = H_eng  * labour_rate_eng;
C_tool = H_tool * labour_rate_tool;
C_mfg  = H_mfg  * labour_rate_mfg;
C_sup  = H_sup  * labour_rate_sup;

Aircraft_Production_Cost = ...
    (C_eng + C_tool + C_mfg + C_sup) * production_factor;

Fleet_Acquisition_Cost = Aircraft_Production_Cost * N_aircraft;

%% =============================
% 4. OPERATING COST MODEL
% ==============================

Crew_cost = 25*(MTOM/1000) * GBP_to_USD;

fuel_mass_per_hr = fuel_volume_per_hr / fuel_density;
Fuel_cost = fuel_mass_per_hr * fuel_price * Annual_hours;

Maint_airframe = 0.03 * Aircraft_Production_Cost;
Maint_engine   = 5e-6 * Aircraft_Production_Cost * fuel_price;
Maint_material = 0.006 * Aircraft_Production_Cost;

Total_Maintenance = Maint_airframe + Maint_engine + Maint_material;

Landing_fees = landing_fee_per_flight * flights_per_year;
Parking_cost = parking_cost_year;

Insurance_cost    = insurance_rate * Aircraft_Production_Cost;
Interest_cost     = interest_rate  * Aircraft_Production_Cost;
Depreciation_cost = Aircraft_Production_Cost / depreciation_years;

%% =============================
% 5. DIRECT OPERATING COST
% ==============================

DOC_per_aircraft = ...
      Crew_cost ...
    + Fuel_cost ...
    + Total_Maintenance ...
    + Landing_fees ...
    + Parking_cost ...
    + Insurance_cost ...
    + Interest_cost ...
    + Depreciation_cost;

DOC_fleet = DOC_per_aircraft * N_aircraft;

%% =============================
% 6. OUTPUT RESULTS
% ==============================

fprintf('\n============== RESULTS (USD) ==============\n\n');

fprintf('Aircraft Production Cost: $%.2f Bn\n', ...
        Aircraft_Production_Cost/1e9);

fprintf('Fleet Acquisition Cost: $%.2f Bn\n\n', ...
        Fleet_Acquisition_Cost/1e9);

fprintf('Annual DOC per Aircraft: $%.2f Million\n', ...
        DOC_per_aircraft/1e6);

fprintf('Annual Fleet DOC: $%.2f Million\n\n', ...
        DOC_fleet/1e6);

fprintf('--- Breakdown per Aircraft ---\n');
fprintf('Crew:          $%.2f M\n', Crew_cost/1e6);
fprintf('Fuel:          $%.2f M\n', Fuel_cost/1e6);
fprintf('Maintenance:   $%.2f M\n', Total_Maintenance/1e6);
fprintf('Landing Fees:  $%.2f M\n', Landing_fees/1e6);
fprintf('Parking:       $%.2f M\n', Parking_cost/1e6);
fprintf('Insurance:     $%.2f M\n', Insurance_cost/1e6);
fprintf('Interest:      $%.2f M\n', Interest_cost/1e6);
fprintf('Depreciation:  $%.2f M\n', Depreciation_cost/1e6);

fprintf('\n===========================================\n');