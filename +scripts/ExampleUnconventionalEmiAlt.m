%% ExampleUnconventionalEmiAlt.m
% Altitude sweep for the Unconventional concept using the full pipeline:
%   Size -> Payload Range -> Mission Evaluation -> Emissions
%
% Assumes these scripts/functions already work individually:
%   - Unconventional.Size
%   - FinalPayloadRange
%   - MissionEvaluationFinal
%   - EmissionsFinal
%
% Also assumes EmissionsFinal stores:
%   ClimateResults.ATR_100yr_K
%   ClimateResults.ATR_100yr_CO2_K
%   ClimateResults.ATR_per_tonne

clear all; clc;
close all force
set(0,'DefaultFigureVisible','off')
drawnow

%% ---------------- PATH / LOOKUP FIX ----------------
thisFile    = mfilename('fullpath');
scriptsDir  = fileparts(thisFile);
projectRoot = fileparts(scriptsDir);   % parent of +scripts

lookupDir   = fullfile(projectRoot, '+Unconventional', '+lookup', '+aerodynamics');
lookupFile  = fullfile(lookupDir, 'cruise_lookup_table.mat');

cd(projectRoot)
addpath(projectRoot)
addpath(scriptsDir)
addpath(lookupDir)
rehash
clear functions

assert(isfolder(lookupDir), 'Lookup directory not found: %s', lookupDir);
assert(isfile(lookupFile),  'Lookup file not found: %s', lookupFile);

fprintf('Project root : %s\n', projectRoot);
fprintf('Lookup file  : %s\n', lookupFile);

%% ---------------- BASELINE AIRCRAFT DEFINITION ----------------
ADP0 = Unconventional.ADP();
ADP0.TLAR = cast.TLAR.Unconventional();
ADP0.TLAR.M_c = 0.84;

% --------------------- Unconventional-specific parameters ---------------------
ADP0.FuselageLength = 65;
ADP0.KinkPos        = 10;
ADP0.CabinRadius    = 6.3;
ADP0.CabinLength    = 50;
ADP0.CockpitLength  = 5;
ADP0.WingPos        = 0.44*ADP0.FuselageLength;
ADP0.V_HT           = 0.97;
ADP0.V_VT           = 0.072;
ADP0.HtpPos         = 0.85*ADP0.FuselageLength;
ADP0.VtpPos         = 0.82*ADP0.FuselageLength;

ADP0.WingArea = 750;

% ------------------------- Hyper-parameters -------------------------
ADP0.Span = 74;

% -------------------------- Class-I estimates --------------------------
ADP0.MTOM    = 490000;
ADP0.Mf_Fuel = 0.32;
ADP0.Mf_res  = 0.03;
ADP0.Mf_Ldg  = 0.62;
ADP0.Mf_TOC  = 0.975;

%% ---------------- SWEEP DEFINITION ----------------
% Keep Mach fixed
Mach_fixed = ADP0.TLAR.M_c;

% IMPORTANT:
% Current TLAR default appears to have:
%   Alt_max = 34.1e3 ./ SI.ft
% so do not exceed ~34100 ft unless you deliberately raise Alt_max.
AltCruise_ft = 22000:2000:34000;
nAlt         = numel(AltCruise_ft);

fprintf('\n===== ALTITUDE SWEEP SETUP =====\n');
fprintf('Fixed Mach: %.2f\n', Mach_fixed);
fprintf('Altitude range: %.0f ft to %.0f ft\n', AltCruise_ft(1), AltCruise_ft(end));
fprintf('Cases: %d\n', nAlt);

%% ---------------- PREALLOCATE RESULTS ----------------
MTOM_t           = nan(nAlt,1);
OEM_t            = nan(nAlt,1);
FleetFuel_t      = nan(nAlt,1);
ScheduleFuel_t   = nan(nAlt,1);
ATR_total_K      = nan(nAlt,1);
ATR_CO2_K        = nan(nAlt,1);
ATR_per_tonne    = nan(nAlt,1);
CruiseFL_avg     = nan(nAlt,1);
MissionTime_hr   = nan(nAlt,1);
SuccessFlag      = false(nAlt,1);

%% ---------------- MAIN SWEEP LOOP ----------------
for iAlt = 1:nAlt

    Alt_now_ft = AltCruise_ft(iAlt);
    Alt_now_m  = Alt_now_ft ./ SI.ft;

    fprintf('\n====================================================\n');
    fprintf('Altitude case %d / %d   |   Commanded Alt = %.0f ft\n', iAlt, nAlt, Alt_now_ft);
    fprintf('====================================================\n');

    % Keep figures under control during sweep
    close all force
    set(0,'DefaultFigureVisible','off')
    drawnow

    % Clear only case-specific outputs
    clear ADP Fleet Flights MissionEval ClimateResults ...
          BlockFuel TripFuel ResFuel Mf_TOC MissionTime cruise_FL ...
          LegResults FlightResults LegTable FlightTable ...
          ExpandedLegs B7Geom B7Mass

    try
        %% 1. Build case-specific aircraft
        ADPi = ADP0;

        % Keep Mach fixed
        ADPi.TLAR.M_c = Mach_fixed;

        % Apply altitude command
        ADPi.TLAR.Alt_cruise = Alt_now_m;

        % Optional guard against exceeding Alt_max
        if isprop(ADPi.TLAR,'Alt_max')
            if ADPi.TLAR.Alt_cruise > ADPi.TLAR.Alt_max
                error('Commanded altitude %.0f ft exceeds TLAR.Alt_max %.0f ft.', ...
                    Alt_now_ft, ADPi.TLAR.Alt_max * SI.ft);
            end
        end

        %% 2. Size aircraft
        ADP = Unconventional.Size(ADPi);

        % Only keep this if some downstream code implicitly expects it
        [B7Geom,B7Mass] = Unconventional.BuildGeometry(ADP); %#ok<NASGU,ASGLU>

        %% 3. Single design mission sanity call
        [BlockFuel,TripFuel,ResFuel,Mf_TOC,MissionTime,cruise_FL] = ...
            Unconventional.MissionAnalysis_PhysicsFinal( ...
            ADP, ADP.TLAR.RangeDes, ADP.MTOM); %#ok<NASGU,ASGLU>

        %% 4. Run full pipeline
        FinalPayloadRange
        MissionEvaluationFinal
        EmissionsFinal

        %% 5. Store results
        MTOM_t(iAlt)         = ADP.MTOM / 1e3;
        OEM_t(iAlt)          = ADP.OEM  / 1e3;

        ScheduleFuel_t(iAlt) = MissionEval.TotalTripFuel_kg / 1e3;
        FleetFuel_t(iAlt)    = MissionEval.FleetTripFuel_kg / 1e3;
        MissionTime_hr(iAlt) = MissionEval.TotalMissionTime_s / 3600;

        if isfield(MissionEval,'LegResults') && ~isempty(MissionEval.LegResults)
            CruiseFL_avg(iAlt) = mean([MissionEval.LegResults.Cruise_FL]);
        end

        ATR_total_K(iAlt)   = ClimateResults.ATR_100yr_K;
        ATR_CO2_K(iAlt)     = ClimateResults.ATR_100yr_CO2_K;
        ATR_per_tonne(iAlt) = ClimateResults.ATR_per_tonne;

        SuccessFlag(iAlt) = true;

        fprintf('Done: Alt = %.0f ft | Fleet fuel = %.1f t | ATR100 = %.3e K\n', ...
            Alt_now_ft, FleetFuel_t(iAlt), ATR_total_K(iAlt));

    catch ME
        warning('Altitude %.0f ft failed: %s', Alt_now_ft, ME.message);
        SuccessFlag(iAlt) = false;
    end
end

%% ---------------- POST-PROCESS ----------------
valid = SuccessFlag;

AltitudeSweepTable = table( ...
    AltCruise_ft(:), ...
    SuccessFlag(:), ...
    MTOM_t(:), ...
    OEM_t(:), ...
    ScheduleFuel_t(:), ...
    FleetFuel_t(:), ...
    MissionTime_hr(:), ...
    CruiseFL_avg(:), ...
    ATR_total_K(:), ...
    ATR_CO2_K(:), ...
    ATR_per_tonne(:), ...
    'VariableNames', { ...
    'AltCruise_ft', 'Success', 'MTOM_t', 'OEM_t', ...
    'ScheduleFuel_t', 'FleetFuel_t', ...
    'MissionTime_hr', 'CruiseFL_avg', ...
    'ATR100_K', 'ATR100_CO2_K', 'ATR_per_tonne'});

disp(' ');
disp('===== ALTITUDE SWEEP RESULTS =====');
disp(AltitudeSweepTable);

%% ---------------- SUMMARY PLOTS ----------------
set(0,'DefaultFigureVisible','on')
close all force
drawnow



set(0,'DefaultAxesFontSize',20)
set(0,'DefaultTextFontSize',20)

if ~any(valid)
    error('No successful altitude sweep cases. Check altitude handling in sizing/mission code.')
end

% 1. ATR total and CO2-only vs commanded altitude
figure('Color','w');
plot(AltCruise_ft(valid), ATR_total_K(valid), 'o-', 'LineWidth', 2, 'MarkerSize', 7); hold on
plot(AltCruise_ft(valid), ATR_CO2_K(valid), 's--', 'LineWidth', 2, 'MarkerSize', 7);
grid on
xlabel('Commanded cruise altitude [ft]')
ylabel('ATR at 100 yr [K]')
title('ATR Sensitivity to Cruise Altitude')
legend('ATR Total','ATR CO_2 Only','Location','best')

% 2. Fleet fuel vs commanded altitude
figure('Color','w');
plot(AltCruise_ft(valid), FleetFuel_t(valid), 'd-', 'LineWidth', 2, 'MarkerSize', 7);
grid on
xlabel('Commanded cruise altitude [ft]')
ylabel('Fleet trip fuel [t]')
title('Fleet Fuel Burn Sensitivity to Cruise Altitude')

% 3. ATR per tonne vs commanded altitude
figure('Color','w');
plot(AltCruise_ft(valid), ATR_per_tonne(valid), 'o-', 'LineWidth', 2, 'MarkerSize', 7);
grid on
xlabel('Commanded cruise altitude [ft]')
ylabel('ATR per tonne payload [K/t]')
title('Climate Efficiency Sensitivity to Cruise Altitude')

% 4. Achieved cruise FL vs commanded altitude
figure('Color','w');
plot(AltCruise_ft(valid), CruiseFL_avg(valid), '^-', 'LineWidth', 2, 'MarkerSize', 7);
grid on
xlabel('Commanded cruise altitude [ft]')
ylabel('Average achieved cruise FL')
title('Achieved Cruise Level vs Commanded Cruise Altitude')

% 5. Normalised comparison
refIdx = find(valid,1,'first');

ATR_total_norm = ATR_total_K / ATR_total_K(refIdx);
ATR_CO2_norm   = ATR_CO2_K   / ATR_CO2_K(refIdx);
Fuel_norm      = FleetFuel_t / FleetFuel_t(refIdx);

figure('Color','w');
plot(AltCruise_ft(valid), ATR_total_norm(valid), 'o-', 'LineWidth', 2); hold on
plot(AltCruise_ft(valid), ATR_CO2_norm(valid), 's--', 'LineWidth', 2);
plot(AltCruise_ft(valid), Fuel_norm(valid), 'd-.', 'LineWidth', 2);
grid on
xlabel('Commanded cruise altitude [ft]')
ylabel('Normalised value [-]')
title('Normalised Sensitivity to Cruise Altitude')
legend('ATR Total','ATR CO_2 Only','Fleet Fuel','Location','best')

%% ---------------- STORE OUTPUT ----------------
AltitudeSweep.ResultsTable   = AltitudeSweepTable;
AltitudeSweep.AltCruise_ft   = AltCruise_ft(:);
AltitudeSweep.SuccessFlag    = SuccessFlag(:);
AltitudeSweep.MTOM_t         = MTOM_t(:);
AltitudeSweep.OEM_t          = OEM_t(:);
AltitudeSweep.ScheduleFuel_t = ScheduleFuel_t(:);
AltitudeSweep.FleetFuel_t    = FleetFuel_t(:);
AltitudeSweep.MissionTime_hr = MissionTime_hr(:);
AltitudeSweep.CruiseFL_avg   = CruiseFL_avg(:);
AltitudeSweep.ATR_total_K    = ATR_total_K(:);
AltitudeSweep.ATR_CO2_K      = ATR_CO2_K(:);
AltitudeSweep.ATR_per_tonne  = ATR_per_tonne(:);

disp(' ');
disp('AltitudeSweep struct created in workspace.');