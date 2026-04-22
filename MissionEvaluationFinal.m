set(0,'DefaultFigureVisible','off')


%% MissionEvaluation.m
% Evaluate all operational flights using stored split logic from Fleet.Flights
% and TRUE route distances redefined here.
%
% Assumes these already exist in workspace:
%   ADP
%   Fleet.Flights(i).RouteName
%   Fleet.Flights(i).Type
%   Fleet.Flights(i).Payload_kg
%   Fleet.Flights(i).NumLegs
%
% IMPORTANT:
% - Do NOT clear the workspace here, because ADP and Fleet are needed.
% - This script uses RouteName as the unique identifier and re-maps to the
%   true AirportPairs/dist_km below.

clc

%% =========================
% Sanity checks
% ==========================
if ~exist('ADP','var')
    error('ADP not found in workspace. Run sizing first.');
end

if ~exist('Fleet','var') || ~isfield(Fleet,'Flights')
    error('Fleet.Flights not found in workspace. Run payload-range / flight-logic script first.');
end

if isempty(Fleet.Flights)
    error('Fleet.Flights is empty.');
end

%% =========================
% Choose mission analysis function
% ==========================
% Your current workflow appears to use this one:
missionFcn = @Unconventional.MissionAnalysis_PhysicsFinal;

% If your project/class name is different, change only this line.
% Example alternative:
% missionFcn = @B777.MissionAnalysis;

%% =========================
% TRUE route definitions (authoritative values)
% ==========================
AirportPairs = [
"LHR-MEL"
"MEL-PVG"
"PVG-SUZ"
"SUZ-BAH"
"BAH-JED"
"JED-MIA"
"MIA-YUL"
"YUL-MCM"
"MCM-MAD"
"MAD-GYD"
"GYD-SIN"
"SIN-AUS"
"AUS-MEX"
"MEX-GRU"
"GRU-LAS"
"LAS-LUA"
"LUA-AUH"
"AUH-LHR"
];

dist_km = [
16909.38
8017.62
1456.91
8052.37
1272.47
11621.60
2264.78
6128.98
956.83
4462.04
6941.58
15823.46
1205.03
7433.53
9782.64
13052.79
320.63
5454
];

if numel(unique(AirportPairs)) ~= numel(AirportPairs)
    error('AirportPairs must be unique because RouteName is used as the lookup key.');
end

%% =========================
% Aircraft constants
% ==========================
OEM      = ADP.OEM;       % [kg]
MTOM_max = ADP.MTOM;      % [kg]

% Keep fuel-capacity logic consistent with your payload-range script
rho_fuel = 800;           % [kg/m^3]
V_fuel   = 613.8;         % [m^3]
maxFuel  = rho_fuel * V_fuel * 0.33;   % [kg]

% Safety clamp
maxFuel = min(maxFuel, MTOM_max - OEM);

fprintf('\n===== MISSION EVALUATION SETUP =====\n');
fprintf('OEM:      %.1f t\n', OEM/1e3);
fprintf('MTOM max: %.1f t\n', MTOM_max/1e3);
fprintf('Max fuel: %.1f t\n', maxFuel/1e3);
fprintf('Flights in Fleet: %d\n', numel(Fleet.Flights));

%% =========================
% Expand stored flight logic into actual legs
% using TRUE route distances
% ==========================
ExpandedLegs = struct([]);
k = 1;

for i = 1:numel(Fleet.Flights)

    routeName = string(Fleet.Flights(i).RouteName);
    nLegs     = Fleet.Flights(i).NumLegs;
    payload   = Fleet.Flights(i).Payload_kg;
    flightType = string(Fleet.Flights(i).Type);

    idxRoute = find(AirportPairs == routeName, 1);

    if isempty(idxRoute)
        error('Route "%s" stored in Fleet.Flights not found in AirportPairs.', routeName);
    end

    if nLegs < 1 || abs(nLegs - round(nLegs)) > 0
        error('Invalid NumLegs for route %s.', routeName);
    end

    totalRange_km = dist_km(idxRoute);
    legRange_km   = totalRange_km / nLegs;

    % Consistency checks
    if flightType == "direct" && nLegs ~= 1
        warning('Route %s tagged direct but NumLegs = %d.', routeName, nLegs);
    end
    if flightType == "split" && nLegs ~= 2
        warning('Route %s tagged split but NumLegs = %d.', routeName, nLegs);
    end

    for j = 1:nLegs
        ExpandedLegs(k).RouteName         = routeName;
        ExpandedLegs(k).LegName           = routeName + "_L" + string(j);
        ExpandedLegs(k).ParentFlightIndex = i;
        ExpandedLegs(k).Type              = flightType;
        ExpandedLegs(k).LegIndex          = j;
        ExpandedLegs(k).NumLegs           = nLegs;
        ExpandedLegs(k).TrueRouteRange_km = totalRange_km;
        ExpandedLegs(k).LegRange_km       = legRange_km;
        ExpandedLegs(k).Payload_kg        = payload;
        k = k + 1;
    end
end

nLegsTotal = numel(ExpandedLegs);

fprintf('Expanded mission legs: %d\n', nLegsTotal);

%% =========================
% Mission analysis per leg with fuel-mass convergence
% ==========================
tolFuel_kg = 50;      % convergence tolerance on required start fuel
maxIter    = 30;

LegResults = ExpandedLegs;

TotalTripFuel_kg    = 0;
TotalBlockFuel_kg   = 0;
TotalReserveFuel_kg = 0;
TotalMissionTime_s  = 0;

fprintf('\n===== RUNNING LEG-BY-LEG MISSION ANALYSIS =====\n');

for i = 1:nLegsTotal

    routeName = LegResults(i).RouteName;
    legName   = LegResults(i).LegName;
    R_m       = LegResults(i).LegRange_km * 1000;
    payload   = LegResults(i).Payload_kg;

    if payload < 0
        error('Negative payload on %s.', legName);
    end

    % Initial guess for fuel at start of this leg
    fuelGuess = min(maxFuel, max(2000, 0.15*MTOM_max));

    converged = false;

    for iter = 1:maxIter

        MTOM_guess = OEM + payload + fuelGuess;

        % Enforce MTOM cap
        if MTOM_guess > MTOM_max
            MTOM_guess = MTOM_max;
            fuelGuess = max(MTOM_guess - OEM - payload, 0);
        end

        % Enforce tank cap
        fuelGuess = min(fuelGuess, maxFuel);
        MTOM_guess = OEM + payload + fuelGuess;

        if MTOM_guess > MTOM_max
            error('Cannot satisfy MTOM on %s. Payload %.1f t is too high.', ...
                legName, payload/1e3);
        end

        % Run mission analysis at current takeoff mass
        [BlockFuel, TripFuel, ResFuel, Mf_TOC, MissionTime, cruise_FL] = ...
            missionFcn(ADP, R_m, MTOM_guess);

        % Robust interpretation of required start fuel
        requiredFuel = max([BlockFuel, TripFuel + ResFuel, 0]);

        % Check feasibility
        if requiredFuel > maxFuel + tolFuel_kg
            error('Fuel-capacity violation on %s: requires %.1f t, max available %.1f t.', ...
                legName, requiredFuel/1e3, maxFuel/1e3);
        end

        % Convergence check
        if abs(requiredFuel - fuelGuess) < tolFuel_kg
            converged = true;
            fuelGuess = requiredFuel;
            break
        end

        fuelGuess = min(requiredFuel, maxFuel);
    end

    if ~converged
        warning('Fuel convergence not fully reached on %s after %d iterations.', legName, maxIter);
    end

    % Final pass at converged fuel
    StartFuel_kg = fuelGuess;
    TakeoffMass_kg = OEM + payload + StartFuel_kg;

    [BlockFuel, TripFuel, ResFuel, Mf_TOC, MissionTime, cruise_FL] = ...
        missionFcn(ADP, R_m, TakeoffMass_kg);

    % Store results
    LegResults(i).TakeoffMass_kg = TakeoffMass_kg;
    LegResults(i).StartFuel_kg   = StartFuel_kg;
    LegResults(i).BlockFuel_kg   = BlockFuel;
    LegResults(i).TripFuel_kg    = TripFuel;
    LegResults(i).ReserveFuel_kg = ResFuel;
    LegResults(i).Mf_TOC         = Mf_TOC;
    LegResults(i).MissionTime_s  = MissionTime;
    LegResults(i).Cruise_FL      = cruise_FL;
    LegResults(i).Iterations     = iter;
    LegResults(i).Converged      = converged;
    LegResults(i).MTOMMargin_kg  = MTOM_max - TakeoffMass_kg;
    LegResults(i).FuelMargin_kg  = maxFuel - StartFuel_kg;

    TotalTripFuel_kg    = TotalTripFuel_kg    + TripFuel;
    TotalBlockFuel_kg   = TotalBlockFuel_kg   + BlockFuel;
    TotalReserveFuel_kg = TotalReserveFuel_kg + ResFuel;
    TotalMissionTime_s  = TotalMissionTime_s  + MissionTime;

    fprintf('%-18s | %-12s | %7.0f km | Payload %6.1f t | TO mass %7.1f t | Trip %6.1f t | Res %6.1f t\n', ...
        char(routeName), char(legName), R_m/1000, payload/1e3, ...
        TakeoffMass_kg/1e3, TripFuel/1e3, ResFuel/1e3);
end

%% =========================
% Aggregate back to stored flights
% ==========================
FlightResults = Fleet.Flights;

for i = 1:numel(Fleet.Flights)

    idx = [LegResults.ParentFlightIndex] == i;

    FlightResults(i).TotalTripFuel_kg    = sum([LegResults(idx).TripFuel_kg]);
    FlightResults(i).TotalBlockFuel_kg   = sum([LegResults(idx).BlockFuel_kg]);
    FlightResults(i).TotalReserveFuel_kg = sum([LegResults(idx).ReserveFuel_kg]);
    FlightResults(i).TotalMissionTime_s  = sum([LegResults(idx).MissionTime_s]);

    % Use true route range from lookup
    thisRoute = string(Fleet.Flights(i).RouteName);
    idxRoute = find(AirportPairs == thisRoute, 1);
    FlightResults(i).TrueRouteRange_km = dist_km(idxRoute);

    % Store true per-leg range for reference in this script only
    FlightResults(i).TrueLegRange_km = dist_km(idxRoute) / Fleet.Flights(i).NumLegs;
end

%% =========================
% Tables for easy inspection
% ==========================
LegTable = table( ...
    string({LegResults.RouteName})', ...
    string({LegResults.LegName})', ...
    [LegResults.LegIndex]', ...
    [LegResults.NumLegs]', ...
    [LegResults.LegRange_km]', ...
    [LegResults.Payload_kg]'./1e3, ...
    [LegResults.TakeoffMass_kg]'./1e3, ...
    [LegResults.TripFuel_kg]'./1e3, ...
    [LegResults.ReserveFuel_kg]'./1e3, ...
    [LegResults.BlockFuel_kg]'./1e3, ...
    [LegResults.MissionTime_s]'./3600, ...
    'VariableNames', { ...
    'RouteName','LegName','LegIndex','NumLegs','LegRange_km', ...
    'Payload_t','TakeoffMass_t','TripFuel_t','ReserveFuel_t', ...
    'BlockFuel_t','MissionTime_hr'});

FlightTable = table( ...
    string({FlightResults.RouteName})', ...
    string({FlightResults.Type})', ...
    [FlightResults.NumLegs]', ...
    [FlightResults.TrueRouteRange_km]', ...
    [FlightResults.Payload_kg]'./1e3, ...
    [FlightResults.TotalTripFuel_kg]'./1e3, ...
    [FlightResults.TotalReserveFuel_kg]'./1e3, ...
    [FlightResults.TotalBlockFuel_kg]'./1e3, ...
    [FlightResults.TotalMissionTime_s]'./3600, ...
    'VariableNames', { ...
    'RouteName','Type','NumLegs','TrueRouteRange_km', ...
    'Payload_t','TotalTripFuel_t','TotalReserveFuel_t', ...
    'TotalBlockFuel_t','TotalMissionTime_hr'});

%% =========================
% Summary metrics
% ==========================
nDirect = sum(string({Fleet.Flights.Type}) == "direct");
nSplit  = sum(string({Fleet.Flights.Type}) == "split");

fprintf('\n===== FLEET / MISSION SUMMARY =====\n');
fprintf('Stored flights:          %d\n', numel(Fleet.Flights));
fprintf('Direct flights:          %d\n', nDirect);
fprintf('Split flights:           %d\n', nSplit);
fprintf('Expanded legs:           %d\n', nLegsTotal);
fprintf('Total trip fuel burn:    %.1f t\n', TotalTripFuel_kg/1e3);
fprintf('Total block fuel loaded: %.1f t\n', TotalBlockFuel_kg/1e3);
fprintf('Total reserve fuel sum:  %.1f t\n', TotalReserveFuel_kg/1e3);
fprintf('Total mission time:      %.1f hr\n', TotalMissionTime_s/3600);

%% =========================
% Basic plots
% ==========================

set(0,'DefaultFigureVisible','on')

figure('Color','w');
tiledlayout(2,2);

nexttile
bar(categorical(LegTable.LegName), LegTable.TripFuel_t)
ylabel('Trip Fuel [t]')
title('Trip Fuel per Leg')
xtickangle(45)
grid on

nexttile
bar(categorical(LegTable.LegName), LegTable.TakeoffMass_t)
ylabel('Takeoff Mass [t]')
title('Takeoff Mass per Leg')
xtickangle(45)
grid on

nexttile
bar(categorical(FlightTable.RouteName), FlightTable.TotalTripFuel_t)
ylabel('Total Route Fuel [t]')
title('Total Trip Fuel per Stored Flight')
xtickangle(45)
grid on

nexttile
bar(categorical(FlightTable.RouteName), FlightTable.TotalMissionTime_hr)
ylabel('Mission Time [hr]')
title('Total Mission Time per Stored Flight')
xtickangle(45)
grid on

%% =========================
% Outputs kept in workspace
% ==========================
MissionEval.LegResults   = LegResults;
MissionEval.FlightResults = FlightResults;
MissionEval.LegTable     = LegTable;
MissionEval.FlightTable  = FlightTable;

MissionEval.TotalTripFuel_kg    = TotalTripFuel_kg;
MissionEval.TotalBlockFuel_kg   = TotalBlockFuel_kg;
MissionEval.TotalReserveFuel_kg = TotalReserveFuel_kg;
MissionEval.TotalMissionTime_s  = TotalMissionTime_s;

disp(' ');
disp('MissionEval struct created in workspace.');
disp('LegTable and FlightTable are also available.');


%% =========================
% FLEET-LEVEL ANALYSIS (BOLT-ON)
% ==========================

fprintf('\n===== FLEET-LEVEL ANALYSIS =====\n');

%% --- Assumptions (EDIT THESE) ---
Utilisation_hr_per_day = 18;     % realistic cargo utilisation
SchedulePeriod_days    = 1;      % 1 = daily network repetition
Fuel_CO2_kg_per_kg     = 3.16;   % Jet-A emissions factor

%% --- Core Fleet Sizing ---
TotalMissionTime_hr = TotalMissionTime_s / 3600;

FleetSize = 6;
% FleetSize = ceil(TotalMissionTime_hr / ...
%     (Utilisation_hr_per_day * SchedulePeriod_days));

%% --- Fleet-Level Totals ---
FleetTripFuel_kg    = FleetSize * TotalTripFuel_kg;
FleetBlockFuel_kg   = FleetSize * TotalBlockFuel_kg;
FleetReserveFuel_kg = FleetSize * TotalReserveFuel_kg;
FleetMissionTime_hr = FleetSize * TotalMissionTime_hr;

%% --- Payload consistency check ---
TotalPayload_t = sum([Fleet.Flights.Payload_kg]) / 1e3;

% %% --- Climate Impact ---
% FleetCO2_kg = FleetTripFuel_kg * Fuel_CO2_kg_per_kg;
% FleetCO2_t  = FleetCO2_kg / 1e3;

%% =========================
% PRINT FINAL SUMMARY
% ==========================

fprintf('\n===== FINAL FLEET SUMMARY =====\n');

fprintf('Fleet size required:        %d aircraft\n', FleetSize);

fprintf('\n--- PER AIRCRAFT (1 full schedule) ---\n');
fprintf('Total mission time:         %.1f hr\n', TotalMissionTime_hr);
fprintf('Total trip fuel:            %.1f t\n', TotalTripFuel_kg/1e3);
fprintf('Total block fuel:           %.1f t\n', TotalBlockFuel_kg/1e3);

fprintf('\n--- FLEET TOTALS ---\n');
fprintf('Fleet mission time:         %.1f hr\n', FleetMissionTime_hr);
fprintf('Fleet trip fuel burn:       %.1f t\n', FleetTripFuel_kg/1e3);
fprintf('Fleet block fuel loaded:    %.1f t\n', FleetBlockFuel_kg/1e3);
fprintf('Fleet reserve fuel:         %.1f t\n', FleetReserveFuel_kg/1e3);

fprintf('\n--- PAYLOAD ---\n');
fprintf('Total payload moved:        %.1f t\n', TotalPayload_t);

fprintf('\n--- CLIMATE IMPACT ---\n');
%fprintf('Total CO2 emissions:        %.1f t\n', FleetCO2_t);

fprintf('\n--- UTILISATION ---\n');
fprintf('Utilisation per aircraft:   %.1f hr/day\n', Utilisation_hr_per_day);
fprintf('Schedule period:            %.1f day(s)\n', SchedulePeriod_days);

%% =========================
% STORE IN STRUCT
% ==========================
MissionEval.FleetSize            = FleetSize;
MissionEval.FleetTripFuel_kg     = FleetTripFuel_kg;
MissionEval.FleetBlockFuel_kg    = FleetBlockFuel_kg;
MissionEval.FleetReserveFuel_kg  = FleetReserveFuel_kg;
MissionEval.FleetMissionTime_hr  = FleetMissionTime_hr;
%MissionEval.FleetCO2_t           = FleetCO2_t;

disp(' ');
disp('Fleet-level results added to MissionEval.');

%% ================= FLEET FUEL ACCOUNTING (FIXED) =================

% Use results already computed in this script

TotalFuelBurnt_kg = FleetTripFuel_kg;   % actual fuel burned
ReserveFuel_kg    = FleetReserveFuel_kg;

% Optional: operational buffer (your "+6 flights" idea)
FuelBought_kg = TotalFuelBurnt_kg + 6 * mean([LegResults.ReserveFuel_kg]);

% Convert to tonnes
TotalFuelBurnt_t = TotalFuelBurnt_kg / 1e3;
FuelBought_t     = FuelBought_kg / 1e3;

fprintf('\n--- FLEET FUEL ACCOUNTING ---\n');
fprintf('Total fuel burnt: %.1f t\n', TotalFuelBurnt_t);
fprintf('Fuel bought: %.1f t\n', FuelBought_t);