%% Size an Unconventional at a Mach number of 0.84

clear all; clc;

close all force
set(0,'DefaultFigureVisible','off')
drawnow


%% ---------------- PATH / LOOKUP FIX ----------------
thisFile   = mfilename('fullpath');
scriptsDir = fileparts(thisFile);
projectRoot = fileparts(scriptsDir);   % parent of +scripts

lookupDir  = fullfile(projectRoot, '+Unconventional', '+lookup', '+aerodynamics');
lookupFile = fullfile(lookupDir, 'cruise_lookup_table.mat');

cd(projectRoot)
addpath(projectRoot)
addpath(lookupDir)   % quick workaround for lookup-table access
rehash
clear functions

assert(isfolder(lookupDir), 'Lookup directory not found: %s', lookupDir);
assert(isfile(lookupFile),  'Lookup file not found: %s', lookupFile);

fprintf('Project root : %s\n', projectRoot);
fprintf('Lookup file  : %s\n', lookupFile);

% Instantiate an instance of the Unconventional class add define some initial
% parameters
ADP0 = Unconventional.ADP();
ADP0.TLAR = cast.TLAR.Unconventional(); % sets top level aircraft requirements
ADP0.TLAR.M_c = 0.84;
% Unconventional.aerodynamics.high_lift(ADP)

% --------------------- set Unconventional specific parameters ---------------------
ADP0.FuselageLength = 65; % Total fuselage length (m)
ADP0.KinkPos = 10;       % spanwise position of TE kink in wing planform
ADP0.CabinRadius = 6.3;
ADP0.CabinLength = 50;
ADP0.CockpitLength = 5;
ADP0.WingPos = 0.44*ADP0.FuselageLength; % normalised wing position (% of fuselage length)
ADP0.V_HT = 0.97; % horizontal tail volume coefficent
ADP0.V_VT = 0.072; % vertical tail volume coefficent
ADP0.HtpPos = 0.85*ADP0.FuselageLength; % normalised HTP position (% of fuselage length)
ADP0.VtpPos = 0.82*ADP0.FuselageLength; % normalised VTP position (% of fuselage length)

ADP0.WingArea = 750; % m^2 guess

% ------------------------- set Hyper-parameters -------------------------
ADP0.Span = 74; % Gate code + Folding tips
% ADP0.FleetSize = 6;

% -------------------------- class-I estimates ---------------------------
% initial mission analysis to estimate MTOM

ADP0.MTOM = 490000; % VERY basic guess of MTOM from payload

% initial estimate of fuel mass ( % of MTOM)
ADP0.Mf_Fuel = 0.32; % maximum fuel mass
ADP0.Mf_res  = 0.03; % reserve fuel mass

% initial estimate of mass fractions at important flight phases
ADP0.Mf_Ldg = 0.62;   % maximum landing mass
ADP0.Mf_TOC = 0.975;  % mass at the Top of Climb (TOC)

%% first edits of code for sweep -----------------------









% 
% 
% % -------------------------------- Sizing --------------------------------
% % Note - see the "size" function at the bottom of this script
% ADP = Unconventional.Size(ADP0);
% 
% %% build the "Sized" geometry and plot it
% [B7Geom,B7Mass] = Unconventional.BuildGeometry(ADP);
% 
% f = figure(1);
% clf
% 
% ax = axes(f);
% hold(ax,'on')
% axis(ax,'equal')
% set(ax,'YDir','normal')
% 
% % ------------------- background PNG FIRST -------------------
% %img = imread("C:\Users\OscarAntill\OneDrive - University of Bristol\group3\B777F_planform.png");   % use your new image
% % 
% % % set image extent to match your aircraft drawing coordinates
% % % adjust these numbers if needed to line up perfectly
% % xImg = [0 80];
% % yImg = [-40 40];
% % 
% % hImg = image(ax, ...
% %     'XData', xImg, ...
% %     'YData', yImg, ...
% %     'CData', img);
% % 
% % set(hImg,'AlphaData',0.45)   % transparency
% % uistack(hImg,'bottom')       % keep image behind geometry
% % 
% % % ------------------- draw geometry on top -------------------
% % cast.draw(B7Geom,B7Mass,[0,0])
% % 
% % ax.XAxis.Visible = "on";
% % ax.YAxis.Visible = "on";
% % xlim(ax, xImg)
% % ylim(ax, yImg)
% % 
% % exportgraphics(ax,'B777F_overlay.png','Resolution',300)
% 
% % print some key data points
% %d = B7Mass.GetData;
% % fprintf('MTOM: %0.0f t, Fuel Mass: %0.0f t, Wing Mass %0.0f t\n',ADP.MTOM/1e3,ADP.Mf_Fuel*ADP.MTOM/1e3,double(d(strcmp(d(:,1),"Wing"),2)));
% % fprintf('CD0: %0.3f, CD (CL=0.5): %0.3f \n',ADP.AeroPolar.CD(0),ADP.AeroPolar.CD(0.5));
% 
% %% Example call to mission analysis discipline
% %[BlockFuel,TripFuel,ResFuel,Mf_TOC,MissionTime] = Unconventional.MissionAnalysis_oscar(ADP, ADP.TLAR.RangeDes, ADP.MTOM);
% [BlockFuel,TripFuel,ResFuel,Mf_TOC,MissionTime,cruise_FL] = Unconventional.MissionAnalysis_PhysicsFinal(ADP, ADP.TLAR.RangeDes, ADP.MTOM);
% 
% %% Example Trade study, comparing MTOM and Block Fuel as a function of wing span
% % %predefine spans to test
% % Spans = 50:1:100;
% % 
% % % pre-allocate arrays for results
% % mtoms = zeros(size(Spans));
% % fuels = zeros(size(Spans));
% % 
% % % Use the converged baseline aircraft as the starting point for each case,
% % % then re-size to convergence at each span
% % % for i = 1:length(Spans)
% % %     ADPi = ADP;                 % start from converged baseline aircraft
% % %     ADPi.Span = Spans(i);       % apply new span
% % %     ADPi = Unconventional.Size(ADPi);   % re-converge aircraft at this span
% % % 
% % %     mtoms(i) = ADPi.MTOM;
% % %     fuels(i) = ADPi.Mf_Fuel * ADPi.MTOM;
% % % end
% % 
% % % out = Unconventional.plotCGbubble(ADP0, ...
% % %     FuelFractions = 0:0.1:1, ...
% % %     PalletCounts = 0:45, ...
% % %     CargoStart = 7);
% % 
% % f = figure(2);
% % clf;
% % tt = tiledlayout(2,1);
% % 
% % nexttile(1);
% % plot(Spans, mtoms/1e3, '-s')
% % xlabel('Span [m]')
% % ylabel('MTOM [t]')
% % 
% % nexttile(2);
% % plot(Spans, fuels/1e3, '-o')
% % xlabel('Span [m]')
% % ylabel('Block Fuel [t]')
% % 
% % %% Sizing Function
% 
% 
% 
% %% ================= MISSION PIPELINE =================
% 
% % 1. Payload range + build Fleet
% FinalPayloadRange   % (or whatever your script is called)
% 
% % 2. Mission evaluation (uses ADP + Fleet)
% MissionEvaluationFinal
% 
% 
% set(0,'DefaultFigureVisible','on')
% drawnow
% 
% 
% % 3. Run climate/emissions model
% EmissionsFinal

%% ================= MACH SWEEP BOLT-ON =================

MachVec = 0.60:0.05:0.90;
nMach   = numel(MachVec);

% ----------------- results storage -----------------
MTOM_t            = nan(nMach,1);
OEM_t             = nan(nMach,1);
FleetFuel_t       = nan(nMach,1);
ScheduleFuel_t    = nan(nMach,1);
ATR_total_K       = nan(nMach,1);
ATR_CO2_K         = nan(nMach,1);
ATR_per_tonne     = nan(nMach,1);
CruiseFL_avg      = nan(nMach,1);
MissionTime_hr    = nan(nMach,1);
SuccessFlag       = false(nMach,1);

for iMach = 1:nMach

    Mach_now = MachVec(iMach);

    fprintf('\n====================================================\n');
    fprintf('Mach case %d / %d   |   M = %.2f\n', iMach, nMach, Mach_now);
    fprintf('====================================================\n');

    % ---- stop figure pile-up between cases ----
    close all force
    set(0,'DefaultFigureVisible','off')
    drawnow

    % ---- clear case-specific outputs from previous pass ----
    clear ADP Fleet Flights MissionEval ClimateResults ...
          BlockFuel TripFuel ResFuel Mf_TOC MissionTime cruise_FL ...
          LegResults FlightResults LegTable FlightTable ...
          ExpandedLegs B7Geom B7Mass

    try
        % ====================================================
        % 1. SIZE AIRCRAFT AT THIS MACH
        % ====================================================
        ADPi = ADP0;
        ADPi.TLAR.M_c = Mach_now;
        
        ADP = Unconventional.Size(ADPi);

        % Optional: only keep this if some downstream code needs it
        [B7Geom,B7Mass] = Unconventional.BuildGeometry(ADP);

        % Optional: design mission call for sanity only
        [BlockFuel,TripFuel,ResFuel,Mf_TOC,MissionTime,cruise_FL] = ...
            Unconventional.MissionAnalysis_PhysicsFinal( ...
            ADP, ADP.TLAR.RangeDes, ADP.MTOM); %#ok<NASGU,ASGLU>

        % ====================================================
        % 2. RUN YOUR EXISTING PIPELINE
        % ====================================================
        FinalPayloadRange
        MissionEvaluationFinal
        EmissionsFinal

        % ====================================================
        % 3. STORE RESULTS
        % ====================================================
        MTOM_t(iMach)         = ADP.MTOM / 1e3;
        OEM_t(iMach)          = ADP.OEM  / 1e3;
        
        ScheduleFuel_t(iMach) = MissionEval.TotalTripFuel_kg / 1e3;
        FleetFuel_t(iMach)    = MissionEval.FleetTripFuel_kg / 1e3;
        MissionTime_hr(iMach) = MissionEval.TotalMissionTime_s / 3600;
        
        if isfield(MissionEval,'LegResults') && ~isempty(MissionEval.LegResults)
            CruiseFL_avg(iMach) = mean([MissionEval.LegResults.Cruise_FL]);
        end
        
        ATR_total_K(iMach)   = ClimateResults.ATR_100yr_K;
        ATR_CO2_K(iMach)     = ClimateResults.ATR_100yr_CO2_K;
        ATR_per_tonne(iMach) = ClimateResults.ATR_per_tonne;
        
        SuccessFlag(iMach) = true;
        
        fprintf('Done: M = %.2f | Fleet fuel = %.1f t | ATR100 = %.3e K\n', ...
            Mach_now, FleetFuel_t(iMach), ATR_total_K(iMach));

    catch ME
        warning('Mach %.2f failed: %s', Mach_now, ME.message);
        SuccessFlag(iMach) = false;
    end
end

%% ================= POST-PROCESS SWEEP =================

valid = SuccessFlag;

MachSweepTable = table( ...
    MachVec(:), ...
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
    'Mach', 'Success', 'MTOM_t', 'OEM_t', ...
    'ScheduleFuel_t', 'FleetFuel_t', ...
    'MissionTime_hr', 'CruiseFL_avg', ...
    'ATR100_K', 'ATR100_CO2_K', 'ATR_per_tonne'});

disp(' ');
disp('===== MACH SWEEP RESULTS =====');
disp(MachSweepTable);

%% ================= SUMMARY PLOTS ONLY =================

set(0,'DefaultFigureVisible','on')
close all force
drawnow


set(0,'DefaultAxesFontSize',20)
set(0,'DefaultTextFontSize',20)


% ---- 1. ATR total and CO2-only vs Mach ----
figure('Color','w');
plot(MachVec(valid), ATR_total_K(valid), 'o-', 'LineWidth', 2, 'MarkerSize', 7); hold on
plot(MachVec(valid), ATR_CO2_K(valid), 's--', 'LineWidth', 2, 'MarkerSize', 7);
grid on
xlabel('Cruise Mach')
ylabel('ATR at 100 yr [K]')
title('ATR Sensitivity to Cruise Mach')
legend('ATR Total','ATR CO_2 Only','Location','best')

% ---- 2. Fleet fuel vs Mach ----
figure('Color','w');
plot(MachVec(valid), FleetFuel_t(valid), 'd-', 'LineWidth', 2, 'MarkerSize', 7);
grid on
xlabel('Cruise Mach')
ylabel('Fleet trip fuel [t]')
title('Fleet Fuel Burn Sensitivity to Cruise Mach')

% ---- 3. ATR per tonne vs Mach ----
figure('Color','w');
plot(MachVec(valid), ATR_per_tonne(valid), 'o-', 'LineWidth', 2, 'MarkerSize', 7);
grid on
xlabel('Cruise Mach')
ylabel('ATR per tonne payload [K/t]')
title('Climate Efficiency Sensitivity to Cruise Mach')

% ---- 4. Average cruise FL vs Mach ----
figure('Color','w');
plot(MachVec(valid), CruiseFL_avg(valid), '^-', 'LineWidth', 2, 'MarkerSize', 7);
grid on
xlabel('Cruise Mach')
ylabel('Average cruise FL')
title('Cruise Altitude Sensitivity to Cruise Mach')

%% ================= STORE SWEEP OUTPUT =================

MachSweep.ResultsTable   = MachSweepTable;
MachSweep.MachVec        = MachVec(:);
MachSweep.SuccessFlag    = SuccessFlag(:);
MachSweep.MTOM_t         = MTOM_t(:);
MachSweep.OEM_t          = OEM_t(:);
MachSweep.ScheduleFuel_t = ScheduleFuel_t(:);
MachSweep.FleetFuel_t    = FleetFuel_t(:);
MachSweep.MissionTime_hr = MissionTime_hr(:);
MachSweep.CruiseFL_avg   = CruiseFL_avg(:);
MachSweep.ATR_total_K    = ATR_total_K(:);
MachSweep.ATR_CO2_K      = ATR_CO2_K(:);
MachSweep.ATR_per_tonne  = ATR_per_tonne(:);

disp(' ');
disp('MachSweep struct created in workspace.');

