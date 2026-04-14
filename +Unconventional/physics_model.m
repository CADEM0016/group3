function [BlockFuel, TripFuel, ResFuel, Mf_TOC, ...
          MissionTime, Mission, CriticalTW, CriticalWS] = ...
          MissionAnalysisPhysics(ADP, tripRange, M_TO)
%MISSIONANALYSIS Conduct mission analysis using a refactored mission model
%
% Outputs:
%   BlockFuel   - total fuel required [kg]
%   TripFuel    - main mission fuel [kg]
%   ResFuel     - reserve fuel [kg]
%   Mf_TOC      - mass fraction at top of climb [-]
%   MissionTime - total mission time incl. reserve segments [s]
%   Mission     - struct containing detailed segment data
%   CriticalTW  - struct containing critical T/W case over mission
%   CriticalWS  - struct containing critical W/S case over mission

arguments
    ADP
    tripRange
    M_TO = ADP.MTOM
end

%% ----------------------------- Constants ------------------------------
g = 9.81;
rho0 = 1.225; %#ok<NASGU>

%% ---------------------- Mission assumptions ---------------------------
TaxiTime_out   = 20 * 60;      % s
TaxiTime_in    = 20 * 60;      % s
ApproachTime   = 10 * 60;      % s
LoiterTime     = 30 * 60;      % s
ContTime       = 5 * 60;       % s
AltRange       = 350e3;        % m

% Vertical rates [m/s]
ROC_to      = (20000 - 1500) / (30 * 196.85);
ROC_climb_1 = ROC_to;
ROC_climb_2 = ROC_to;
ROC_climb_3 = ROC_to;

ROD_descent_1 = ROC_climb_1;
ROD_descent_2 = ROC_climb_1;
ROD_descent_3 = ROC_climb_1;

% Speed schedules
V_climb_1   = 250 * 0.514444;  % m/s
V_climb_2   = 300 * 0.514444;  % m/s
V_descent_2 = V_climb_2;
V_descent_3 = V_climb_1;

IdleFrac     = 0.07;
ClimbFrac    = 0.85; %#ok<NASGU>
DescentFrac  = 0.12; %#ok<NASGU>
ApproachFrac = 0.18;

M_cruise = ADP.TLAR.M_c;
M_to     = 0.25;
M_ds     = 0.20;
M_climb  = min(M_cruise, 0.78);
M_desc   = min(M_cruise, 0.70);

Mission = struct();

%% -------------------- Initial mass bookkeeping ------------------------
m_current = M_TO;

%% ============================================================
%% 1) Taxi out
%% ============================================================
T_taxi = IdleFrac * getAvailableThrust(ADP, m_current, 0.0, 0);
FuelTaxiOut = segmentFuelFromThrust(ADP, 0.0, 0, T_taxi, TaxiTime_out);
[TaxiTW, TaxiWS] = thrustToWeightAndWingLoading(T_taxi, m_current, ADP.WingArea, g);

m_current = m_current - FuelTaxiOut;

Mission.TaxiOut.Fuel  = FuelTaxiOut;
Mission.TaxiOut.Time  = TaxiTime_out;
Mission.TaxiOut.Treq  = T_taxi;
Mission.TaxiOut.TW    = TaxiTW;
Mission.TaxiOut.WS    = TaxiWS;
Mission.TaxiOut.Range = 0;

%% ============================================================
%% 2A) Ground acceleration / takeoff sizing from field length
%% ============================================================
[TakeoffTW, T_req_ground, TOGround] = sizeTakeoffFromFieldLength( ...
    ADP, m_current, 2570, ...
    'CLmaxTO', 2.4, ...
    'MuRoll', 0.02, ...
    'Altitude', 0);

FuelTakeoffGround = segmentFuelFromThrust( ...
    ADP, 0.0, 0, T_req_ground, TOGround.EstimatedGroundRollTime);

m_current = m_current - FuelTakeoffGround;

Mission.TakeoffGround.Fuel   = FuelTakeoffGround;
Mission.TakeoffGround.Time   = TOGround.EstimatedGroundRollTime;
Mission.TakeoffGround.Treq   = T_req_ground;
Mission.TakeoffGround.TW     = TakeoffTW;
Mission.TakeoffGround.Vlof   = TOGround.Vlof;
Mission.TakeoffGround.Vstall = TOGround.Vstall_to;
Mission.TakeoffGround.Range  = TOGround.EstimatedRange;
Mission.TakeoffGround.CL     = TOGround.CLground;
Mission.TakeoffGround.CD     = TOGround.CDground;
Mission.TakeoffGround.Drag   = TOGround.D_avg;
Mission.TakeoffGround.WS     = TOGround.WingLoading;

%% ============================================================
%% 2B) Initial climb to 1500 ft
%% ============================================================
h_end = 1500 * 0.3048;   % ft to m
V_climb = 250 * 0.514444;   % 250 kt in m/s
W = m_current * g;

[rho_climb, a_climb, ~, ~] = cast.atmos(h_end/2);

%CL_climb = W / (0.5 * rho_climb * V_climb^2 * ADP.WingArea);
%CD_climb = ADP.AeroPolar.CD(CL_climb);

%% THIS IS CODE IVE ADDED - OSCAR


% Files
baseDir = fileparts(matlab.desktop.editor.getActiveFilename);
cruisePath = fullfile(baseDir, '../+Unconventional/+lookup/+aerodynamics/cruise_lookup_table.mat');
S = load(cruisePath);
cruise_results = S.Results;


% Cruise file
S = load(cruisePath);
cruise_results = S.Results;
span = 73.5;
mach = 0.81;
rootChord = 15.2;
altitude = 9500;

function k = nearestIndex(vec,x)
[~,k] = min(abs(vec-x));
end

span = S.spanVec(nearestIndex(S.spanVec,span));
mach = S.machVec(nearestIndex(S.machVec,mach));
rootChord = S.rootChordVec(nearestIndex(S.rootChordVec,rootChord));
altitude = S.altVec(nearestIndex(S.altVec,altitude));

i = [cruise_results.WingSpan] == span & [cruise_results.Mach] == mach & [cruise_results.RootChord] == rootChord & [cruise_results.Altitude] == altitude;

A = [cruise_results(i).Alpha];
CL = [cruise_results(i).CL];
CD = [cruise_results(i).CD];

[A,ord] = sort(A);
CL = CL(ord);
CD = CD(ord);

alpha = 2;  



% pick optimal altitude for cruise
alts = linspace(8000,11000,12); % OSCAR: Level cruise is in my mind could be anywhere from 8000m to 11000m
[rho,a,T,P] = cast.atmos(alts);
% [rho_s,a_s,~,P_s] = dcrg.aero.atmos(0);
M_cruise = ADP.TLAR.M_c;
CL_c = interp1(A,CL,alpha,'linear','extrap');
CD_c = interp1(A,CD,alpha,'linear','extrap');
LD_c = CL_c./CD_c;
%% THIS IS CODE IVE ADDED - OSCAR


D_climb  = 0.5 * rho_climb * V_climb^2 * ADP.WingArea * CD_climb;

% Use a fixed, reasonable initial climb rate instead of forcing
% the aircraft to reach 1500 ft within a remaining time budget.
ROC_initial = 12.0;   % m/s  (~2360 ft/min), reasonable first-pass value

t_climb = h_end / ROC_initial;

T_req_climb = D_climb + W * ROC_initial / V_climb;
Mach_climb = V_climb / a_climb;

FuelInitialClimb = segmentFuelFromThrust(ADP, Mach_climb, h_end/2, T_req_climb, t_climb);
ROC_climb = ROC_initial;

m_current = m_current - FuelInitialClimb;

Mission.InitialClimb.Fuel     = FuelInitialClimb;
Mission.InitialClimb.Time     = t_climb;
Mission.InitialClimb.CL       = CL_climb;
Mission.InitialClimb.CD       = CD_climb;
Mission.InitialClimb.Drag     = D_climb;
Mission.InitialClimb.Treq     = T_req_climb;
Mission.InitialClimb.V        = V_climb;
Mission.InitialClimb.TW       = T_req_climb / W;
Mission.InitialClimb.WS       = W / ADP.WingArea;
Mission.InitialClimb.Range    = V_climb * t_climb;
Mission.InitialClimb.ROC      = ROC_climb;
Mission.InitialClimb.Mach     = Mach_climb;
Mission.InitialClimb.Altitude = h_end / 2;
Mission.InitialClimb.h_start  = 0;
Mission.InitialClimb.h_end    = h_end;

%% ============================================================
%% 3) Climb 1: 1500 ft to 10000 ft @ 250 kt
%% ============================================================
[m_current, Seg] = flyVerticalSegment(ADP, m_current, ...
    1500 / SI.ft, 10000 / SI.ft, 500 / SI.ft, ...
    'TAS', V_climb_1, ROC_climb_1, 'Climb1');
Mission.Climb1 = Seg;

%% ============================================================
%% 4) Climb 2: 10000 ft to 20000 ft @ 300 kt
%% ============================================================
[m_current, Seg] = flyVerticalSegment(ADP, m_current, ...
    10000 / SI.ft, 20000 / SI.ft, 500 / SI.ft, ...
    'TAS', V_climb_2, ROC_climb_2, 'Climb2');
Mission.Climb2 = Seg;

%% ============================================================
%% 5) Climb 3: 20000 ft to cruise altitude @ Mach
%% ============================================================
[m_current, Seg] = flyVerticalSegment(ADP, m_current, ...
    20000 / SI.ft, ADP.TLAR.Alt_cruise, 500 / SI.ft, ...
    'Mach', min(ADP.TLAR.M_c, 0.80), ROC_climb_3, 'Climb3');
Mission.Climb3 = Seg;

FuelClimb = Mission.InitialClimb.Fuel + Mission.Climb1.Fuel + ...
            Mission.Climb2.Fuel + Mission.Climb3.Fuel;

TimeClimb = Mission.InitialClimb.Time + Mission.Climb1.Time + ...
            Mission.Climb2.Time + Mission.Climb3.Time;

RangeClimbTotal = Mission.InitialClimb.Range + Mission.Climb1.Range + ...
                  Mission.Climb2.Range + Mission.Climb3.Range;

Mf_TOC = m_current / M_TO;

%% ============================================================
%% 5.5) Beginning-of-cruise study + cruise-climb requirement
%% ============================================================
ROC_cruise_req = 300 / 196.85;   % 300 ft/min -> m/s

h_boc = ADP.TLAR.Alt_cruise;
m_boc = m_current;

[rho_boc, a_boc, ~, ~] = cast.atmos(h_boc);

M_boc = M_cruise;
V_boc = M_boc * a_boc;
W_boc = m_boc * g;

%CL_boc = W_boc / (0.5 * rho_boc * V_boc^2 * ADP.WingArea);
%CD_boc = ADP.AeroPolar.CD(CL_boc);

%% THIS IS CODE IVE ADDED - OSCAR


% Files
baseDir = fileparts(matlab.desktop.editor.getActiveFilename);
cruisePath = fullfile(baseDir, '../+Unconventional/+lookup/+aerodynamics/cruise_lookup_table.mat');
S = load(cruisePath);
cruise_results = S.Results;


% Cruise file
S = load(cruisePath);
cruise_results = S.Results;
span = 73.5;
mach = 0.81;
rootChord = 15.2;
altitude = 9500;

function k = nearestIndex(vec,x)
[~,k] = min(abs(vec-x));
end

span = S.spanVec(nearestIndex(S.spanVec,span));
mach = S.machVec(nearestIndex(S.machVec,mach));
rootChord = S.rootChordVec(nearestIndex(S.rootChordVec,rootChord));
altitude = S.altVec(nearestIndex(S.altVec,altitude));

i = [cruise_results.WingSpan] == span & [cruise_results.Mach] == mach & [cruise_results.RootChord] == rootChord & [cruise_results.Altitude] == altitude;

A = [cruise_results(i).Alpha];
CL = [cruise_results(i).CL];
CD = [cruise_results(i).CD];

[A,ord] = sort(A);
CL = CL(ord);
CD = CD(ord);

alpha = 2;  



% pick optimal altitude for cruise
alts = linspace(8000,11000,12); % OSCAR: Level cruise is in my mind could be anywhere from 8000m to 11000m
[rho,a,T,P] = cast.atmos(alts);
% [rho_s,a_s,~,P_s] = dcrg.aero.atmos(0);
M_cruise = ADP.TLAR.M_c;
CL_c = interp1(A,CL,alpha,'linear','extrap');
CD_c = interp1(A,CD,alpha,'linear','extrap');
LD_c = CL_c./CD_c;
%% THIS IS CODE IVE ADDED - OSCAR


D_boc  = 0.5 * rho_boc * V_boc^2 * ADP.WingArea * CD_boc;
LD_boc = CL_boc / CD_boc;

T_req_boc_cruise = D_boc;
[TW_boc_cruise, WS_boc_cruise] = thrustToWeightAndWingLoading( ...
    T_req_boc_cruise, m_boc, ADP.WingArea, g);

Mission.CruiseStudy.BeginCruise.Mass = m_boc;
Mission.CruiseStudy.BeginCruise.Altitude = h_boc;
Mission.CruiseStudy.BeginCruise.Mach = M_boc;
Mission.CruiseStudy.BeginCruise.V = V_boc;
Mission.CruiseStudy.BeginCruise.CL = CL_boc;
Mission.CruiseStudy.BeginCruise.CD = CD_boc;
Mission.CruiseStudy.BeginCruise.LD = LD_boc;
Mission.CruiseStudy.BeginCruise.Drag = D_boc;
Mission.CruiseStudy.BeginCruise.Treq = T_req_boc_cruise;
Mission.CruiseStudy.BeginCruise.TW = TW_boc_cruise;
Mission.CruiseStudy.BeginCruise.WS = WS_boc_cruise;

T_req_boc_cruiseclimb = D_boc + W_boc * ROC_cruise_req / V_boc;
[TW_boc_cruiseclimb, WS_boc_cruiseclimb] = thrustToWeightAndWingLoading( ...
    T_req_boc_cruiseclimb, m_boc, ADP.WingArea, g);

Mission.CruiseStudy.CruiseClimbRequirement.ROC = ROC_cruise_req;
Mission.CruiseStudy.CruiseClimbRequirement.Mass = m_boc;
Mission.CruiseStudy.CruiseClimbRequirement.Altitude = h_boc;
Mission.CruiseStudy.CruiseClimbRequirement.Mach = M_boc;
Mission.CruiseStudy.CruiseClimbRequirement.V = V_boc;
Mission.CruiseStudy.CruiseClimbRequirement.CL = CL_boc;
Mission.CruiseStudy.CruiseClimbRequirement.CD = CD_boc;
Mission.CruiseStudy.CruiseClimbRequirement.LD = LD_boc;
Mission.CruiseStudy.CruiseClimbRequirement.Drag = D_boc;
Mission.CruiseStudy.CruiseClimbRequirement.Treq = T_req_boc_cruiseclimb;
Mission.CruiseStudy.CruiseClimbRequirement.TW = TW_boc_cruiseclimb;
Mission.CruiseStudy.CruiseClimbRequirement.WS = WS_boc_cruiseclimb;

%% ============================================================
%% 6) Main cruise: stepped cruise climb
%% ============================================================
RangeCruiseTarget = max(tripRange - 2 * RangeClimbTotal, 0);

[m_current, Seg] = flyCruiseSegment(ADP, m_current, ...
    RangeCruiseTarget, M_cruise, ADP.TLAR.Alt_cruise, 'MainCruise');
Mission.Cruise = Seg;

FuelCruise = Mission.Cruise.Fuel;
TimeCruise = Mission.Cruise.Time;
RangeCruiseActual = Mission.Cruise.Range; %#ok<NASGU>

%% ============================================================
%% 7) Descent 1: cruise altitude to 20000 ft @ Mach
%% ============================================================
[m_current, Seg] = flyVerticalSegment(ADP, m_current, ...
    ADP.TLAR.Alt_cruise, 20000 / SI.ft, 500 / SI.ft, ...
    'Mach', min(ADP.TLAR.M_c, 0.80), -ROD_descent_1, 'Descent1');
Mission.Descent1 = Seg;

%% ============================================================
%% 8) Descent 2: 20000 ft to 10000 ft @ 300 kt
%% ============================================================
[m_current, Seg] = flyVerticalSegment(ADP, m_current, ...
    20000 / SI.ft, 10000 / SI.ft, 500 / SI.ft, ...
    'TAS', V_descent_2, -ROD_descent_2, 'Descent2');
Mission.Descent2 = Seg;

%% ============================================================
%% 9) Descent 3: 10000 ft to 1500 ft @ 250 kt
%% ============================================================
[m_current, Seg] = flyVerticalSegment(ADP, m_current, ...
    10000 / SI.ft, 1500 / SI.ft, 500 / SI.ft, ...
    'TAS', V_descent_3, -ROD_descent_3, 'Descent3');
Mission.Descent3 = Seg;

FuelDescent = Mission.Descent1.Fuel + Mission.Descent2.Fuel + Mission.Descent3.Fuel;
TimeDescent = Mission.Descent1.Time + Mission.Descent2.Time + Mission.Descent3.Time;

Mission.Descent.Fuel = FuelDescent;
Mission.Descent.Time = TimeDescent;
Mission.Descent.Range = Mission.Descent1.Range + Mission.Descent2.Range + Mission.Descent3.Range;

%% ============================================================
%% 10) Approach / landing
%% ============================================================
alt_app = 0;
[rho_app, a_app, ~, ~] = cast.atmos(alt_app);

V_app = 145 * 0.514444;
Mach_app = V_app / a_app;
W = m_current * g;

%CL_app = W / (0.5 * rho_app * V_app^2 * ADP.WingArea);
%CD_app = ADP.AeroPolar.CD(CL_app);

%% THIS IS CODE IVE ADDED - OSCAR


% Files
baseDir = fileparts(matlab.desktop.editor.getActiveFilename);
cruisePath = fullfile(baseDir, '../+Unconventional/+lookup/+aerodynamics/cruise_lookup_table.mat');
S = load(cruisePath);
cruise_results = S.Results;


% Cruise file
S = load(cruisePath);
cruise_results = S.Results;
span = 73.5;
mach = 0.81;
rootChord = 15.2;
altitude = 9500;

function k = nearestIndex(vec,x)
[~,k] = min(abs(vec-x));
end

span = S.spanVec(nearestIndex(S.spanVec,span));
mach = S.machVec(nearestIndex(S.machVec,mach));
rootChord = S.rootChordVec(nearestIndex(S.rootChordVec,rootChord));
altitude = S.altVec(nearestIndex(S.altVec,altitude));

i = [cruise_results.WingSpan] == span & [cruise_results.Mach] == mach & [cruise_results.RootChord] == rootChord & [cruise_results.Altitude] == altitude;

A = [cruise_results(i).Alpha];
CL = [cruise_results(i).CL];
CD = [cruise_results(i).CD];

[A,ord] = sort(A);
CL = CL(ord);
CD = CD(ord);

alpha = 2;  



% pick optimal altitude for cruise
alts = linspace(8000,11000,12); % OSCAR: Level cruise is in my mind could be anywhere from 8000m to 11000m
[rho,a,T,P] = cast.atmos(alts);
% [rho_s,a_s,~,P_s] = dcrg.aero.atmos(0);
M_cruise = ADP.TLAR.M_c;
CL_c = interp1(A,CL,alpha,'linear','extrap');
CD_c = interp1(A,CD,alpha,'linear','extrap');
LD_c = CL_c./CD_c;
%% THIS IS CODE IVE ADDED - OSCAR


D_app  = 0.5 * rho_app * V_app^2 * ADP.WingArea * CD_app;

T_idle = getIdleThrust(ADP, m_current, Mach_app, alt_app, IdleFrac);
T_req_app = max(T_idle, D_app);

FuelApproach = segmentFuelFromThrust(ADP, Mach_app, alt_app, T_req_app, ApproachTime);
[ApproachTW, ApproachWS] = thrustToWeightAndWingLoading(T_req_app, m_current, ADP.WingArea, g);

m_current = m_current - FuelApproach;

Mission.ApproachLanding.Fuel = FuelApproach;
Mission.ApproachLanding.Time = ApproachTime;
Mission.ApproachLanding.CL = CL_app;
Mission.ApproachLanding.CD = CD_app;
Mission.ApproachLanding.Drag = D_app;
Mission.ApproachLanding.Treq = T_req_app;
Mission.ApproachLanding.V = V_app;
Mission.ApproachLanding.TW = ApproachTW;
Mission.ApproachLanding.WS = ApproachWS;
Mission.ApproachLanding.Range = V_app * ApproachTime;

%% ============================================================
%% 11) Alternate climb
%% ============================================================
[m_current, Seg] = flyVerticalSegment(ADP, m_current, ...
    0, ADP.TLAR.Alt_alternate, 500 / SI.ft, ...
    'Mach', M_climb, ROC_climb_3, 'AltClimb');
Mission.Alternate.Climb = Seg;

%% ============================================================
%% 12) Alternate cruise
%% ============================================================
[m_current, Seg] = flyCruiseSegment(ADP, m_current, ...
    AltRange, M_cruise, ADP.TLAR.Alt_alternate, 'AltCruise');
Mission.Alternate.Cruise = Seg;

%% ============================================================
%% 13) Alternate descent
%% ============================================================
[m_current, Seg] = flyVerticalSegment(ADP, m_current, ...
    ADP.TLAR.Alt_alternate, 0, 500 / SI.ft, ...
    'Mach', M_desc, -ROD_descent_1, 'AltDescent');
Mission.Alternate.Descent = Seg;

%% ============================================================
%% 14) Alternate approach
%% ============================================================
T_req_altapp = ApproachFrac * getAvailableThrust(ADP, m_current, M_ds, 0);
FuelAltApproach = segmentFuelFromThrust(ADP, M_ds, 0, T_req_altapp, ApproachTime);
[AltAppTW, AltAppWS] = thrustToWeightAndWingLoading(T_req_altapp, m_current, ADP.WingArea, g);
m_current = m_current - FuelAltApproach;

Mission.Alternate.Approach.Fuel = FuelAltApproach;
Mission.Alternate.Approach.Time = ApproachTime;
Mission.Alternate.Approach.Treq = T_req_altapp;
Mission.Alternate.Approach.TW = AltAppTW;
Mission.Alternate.Approach.WS = AltAppWS;
Mission.Alternate.Approach.Range = 0;

%% ============================================================
%% 15) Loiter at 1500 ft for 30 min at min drag velocity
%% ============================================================
alt_loiter = 1500 / SI.ft;
[rho_l, a_l, ~, ~] = cast.atmos(alt_loiter);

%% THIS IS CODE IVE ADDED - OSCAR


% Files
baseDir = fileparts(matlab.desktop.editor.getActiveFilename);
cruisePath = fullfile(baseDir, '../+Unconventional/+lookup/+aerodynamics/cruise_lookup_table.mat');
S = load(cruisePath);
cruise_results = S.Results;


% Cruise file
S = load(cruisePath);
cruise_results = S.Results;
span = 73.5;
mach = 0.81;
rootChord = 15.2;
altitude = 9500;

function k = nearestIndex(vec,x)
[~,k] = min(abs(vec-x));
end

span = S.spanVec(nearestIndex(S.spanVec,span));
mach = S.machVec(nearestIndex(S.machVec,mach));
rootChord = S.rootChordVec(nearestIndex(S.rootChordVec,rootChord));
altitude = S.altVec(nearestIndex(S.altVec,altitude));

i = [cruise_results.WingSpan] == span & [cruise_results.Mach] == mach & [cruise_results.RootChord] == rootChord & [cruise_results.Altitude] == altitude;

A = [cruise_results(i).Alpha];
CL = [cruise_results(i).CL];
CD = [cruise_results(i).CD];

[A,ord] = sort(A);
CL = CL(ord);
CD = CD(ord);

alpha = 2;  



% pick optimal altitude for cruise
alts = linspace(8000,11000,12); % OSCAR: Level cruise is in my mind could be anywhere from 8000m to 11000m
[rho,a,T,P] = cast.atmos(alts);
% [rho_s,a_s,~,P_s] = dcrg.aero.atmos(0);
M_cruise = ADP.TLAR.M_c;
CL_c = interp1(A,CL,alpha,'linear','extrap');
CD_c = interp1(A,CD,alpha,'linear','extrap');
LD_c = CL_c./CD_c;
%% THIS IS CODE IVE ADDED - OSCAR

Cls = 0.2:0.01:2.5;
%LDs = Cls ./ ADP.AeroPolar.CD(Cls);
[~, idxL] = max(LDs);

%CL_loiter = Cls(idxL);
%CD_loiter = ADP.AeroPolar.CD(CL_loiter);
LD_loiter = CL_loiter / CD_loiter;

V_loiter = sqrt((2 * m_current * g) / (rho_l * ADP.WingArea * CL_loiter));
M_loiter = V_loiter / a_l;
D_loiter = 0.5 * rho_l * V_loiter^2 * ADP.WingArea * CD_loiter;
[LoiterTW, LoiterWS] = thrustToWeightAndWingLoading(D_loiter, m_current, ADP.WingArea, g);

FuelLoiter = (1 - exp(-LoiterTime * g * ADP.Engine.TSFC(M_loiter, alt_loiter) / LD_loiter)) * m_current;
m_current = m_current - FuelLoiter;

Mission.Loiter.Fuel = FuelLoiter;
Mission.Loiter.Time = LoiterTime;
Mission.Loiter.CL = CL_loiter;
Mission.Loiter.CD = CD_loiter;
Mission.Loiter.LD = LD_loiter;
Mission.Loiter.Treq = D_loiter;
Mission.Loiter.TW = LoiterTW;
Mission.Loiter.WS = LoiterWS;
Mission.Loiter.Range = V_loiter * LoiterTime;

%% ============================================================
%% 16) Contingency
%% ============================================================
TripFuel_preCont = FuelTaxiOut + Mission.TakeoffGround.Fuel + FuelClimb + ...
                   FuelCruise + FuelDescent + FuelApproach;

Fuel5min = (1 - exp(-ContTime * g * ADP.Engine.TSFC(M_loiter, alt_loiter) / LD_loiter)) * m_current;
FuelCont = max(Fuel5min, 0.03 * TripFuel_preCont);

m_current = m_current - FuelCont;

Mission.Contingency.Fuel = FuelCont;
Mission.Contingency.Time = ContTime;
Mission.Contingency.Treq = D_loiter;
Mission.Contingency.TW = LoiterTW;
Mission.Contingency.WS = LoiterWS;
Mission.Contingency.Range = V_loiter * ContTime;

%% ============================================================
%% 17) Totals
%% ============================================================
TripFuel = FuelTaxiOut + Mission.TakeoffGround.Fuel + FuelClimb + ...
           FuelCruise + FuelDescent + FuelApproach;

ResFuel = Mission.Alternate.Climb.Fuel + Mission.Alternate.Cruise.Fuel + ...
          Mission.Alternate.Descent.Fuel + FuelAltApproach + ...
          FuelLoiter + FuelCont;

BlockFuel = TripFuel + ResFuel;

AirborneDesignMissionTime = Mission.TakeoffGround.Time + ...
                            TimeClimb + TimeCruise + TimeDescent + ...
                            ApproachTime;

DesignMissionTime = TaxiTime_out + AirborneDesignMissionTime + TaxiTime_in;

MissionTime = DesignMissionTime + ...
              Mission.Alternate.Climb.Time + Mission.Alternate.Cruise.Time + ...
              Mission.Alternate.Descent.Time + ApproachTime + ...
              LoiterTime + ContTime;

Mission.Total.TripFuel = TripFuel;
Mission.Total.ResFuel = ResFuel;
Mission.Total.BlockFuel = BlockFuel;
Mission.Total.AirborneDesignMissionTime = AirborneDesignMissionTime;
Mission.Total.DesignMissionTime = DesignMissionTime;
Mission.Total.TotalMissionTime = MissionTime;

%% ============================================================
%% 18) Mission sanity-check summary
%% ============================================================
Mission.Summary.TakeoffGround.TW = Mission.TakeoffGround.TW;
Mission.Summary.TakeoffGround.WS = Mission.TakeoffGround.WS;
Mission.Summary.TakeoffGround.Vlof = Mission.TakeoffGround.Vlof;

Mission.Summary.InitialClimb.TW = Mission.InitialClimb.TW;
Mission.Summary.InitialClimb.WS = Mission.InitialClimb.WS;
Mission.Summary.InitialClimb.ROC = Mission.InitialClimb.ROC;

Mission.Summary.CruiseBegin.TW = Mission.CruiseStudy.BeginCruise.TW;
Mission.Summary.CruiseBegin.WS = Mission.CruiseStudy.BeginCruise.WS;
Mission.Summary.CruiseBegin.CL = Mission.CruiseStudy.BeginCruise.CL;
Mission.Summary.CruiseBegin.CD = Mission.CruiseStudy.BeginCruise.CD;
Mission.Summary.CruiseBegin.LD = Mission.CruiseStudy.BeginCruise.LD;

Mission.Summary.CruiseClimbReq.ROC_fpm = 300;
Mission.Summary.CruiseClimbReq.TW = Mission.CruiseStudy.CruiseClimbRequirement.TW;
Mission.Summary.CruiseClimbReq.WS = Mission.CruiseStudy.CruiseClimbRequirement.WS;
Mission.Summary.CruiseClimbReq.Treq = Mission.CruiseStudy.CruiseClimbRequirement.Treq;
Mission.Summary.CruiseClimbReq.Mach = Mission.CruiseStudy.CruiseClimbRequirement.Mach;
Mission.Summary.CruiseClimbReq.Altitude = Mission.CruiseStudy.CruiseClimbRequirement.Altitude;

%% ============================================================
%% 19) Critical T/W and W/S cases
%% ============================================================
twNames = {};
twVals  = [];
twAlts  = [];
twMachs = [];

wsNames = {};
wsVals  = [];
wsAlts  = [];
wsMachs = [];

% ---------- T/W sizing cases ----------
[twNames, twVals, twAlts, twMachs] = addCase(twNames, twVals, twAlts, twMachs, ...
    'TakeoffGround', Mission.TakeoffGround.TW, 0, 0);

[twNames, twVals, twAlts, twMachs] = addCase(twNames, twVals, twAlts, twMachs, ...
    'InitialClimb', Mission.InitialClimb.TW, ...
    Mission.InitialClimb.Altitude, Mission.InitialClimb.Mach);

[twNames, twVals, twAlts, twMachs] = addMaxStepCase(twNames, twVals, twAlts, twMachs, ...
    'Climb1', Mission.Climb1.StepTW, Mission.Climb1.StepAlt, Mission.Climb1.StepMach);

[twNames, twVals, twAlts, twMachs] = addMaxStepCase(twNames, twVals, twAlts, twMachs, ...
    'Climb2', Mission.Climb2.StepTW, Mission.Climb2.StepAlt, Mission.Climb2.StepMach);

[twNames, twVals, twAlts, twMachs] = addMaxStepCase(twNames, twVals, twAlts, twMachs, ...
    'Climb3', Mission.Climb3.StepTW, Mission.Climb3.StepAlt, Mission.Climb3.StepMach);

[twNames, twVals, twAlts, twMachs] = addCase(twNames, twVals, twAlts, twMachs, ...
    'CruiseClimbRequirement', ...
    Mission.CruiseStudy.CruiseClimbRequirement.TW, ...
    Mission.CruiseStudy.CruiseClimbRequirement.Altitude, ...
    Mission.CruiseStudy.CruiseClimbRequirement.Mach);

[twNames, twVals, twAlts, twMachs] = addCase(twNames, twVals, twAlts, twMachs, ...
    'Approach', Mission.ApproachLanding.TW, 0, Mach_app);

[twNames, twVals, twAlts, twMachs] = addMaxStepCase(twNames, twVals, twAlts, twMachs, ...
    'AltClimb', Mission.Alternate.Climb.StepTW, Mission.Alternate.Climb.StepAlt, Mission.Alternate.Climb.StepMach);

[twNames, twVals, twAlts, twMachs] = addCase(twNames, twVals, twAlts, twMachs, ...
    'Loiter', Mission.Loiter.TW, alt_loiter, M_loiter);

% ---------- W/S sizing cases ----------
[wsNames, wsVals, wsAlts, wsMachs] = addCase(wsNames, wsVals, wsAlts, wsMachs, ...
    'TakeoffGround', Mission.TakeoffGround.WS, 0, 0);

[wsNames, wsVals, wsAlts, wsMachs] = addCase(wsNames, wsVals, wsAlts, wsMachs, ...
    'InitialClimb', Mission.InitialClimb.WS, ...
    Mission.InitialClimb.Altitude, Mission.InitialClimb.Mach);

[wsNames, wsVals, wsAlts, wsMachs] = addCase(wsNames, wsVals, wsAlts, wsMachs, ...
    'BeginCruise', ...
    Mission.CruiseStudy.BeginCruise.WS, ...
    Mission.CruiseStudy.BeginCruise.Altitude, ...
    Mission.CruiseStudy.BeginCruise.Mach);

[wsNames, wsVals, wsAlts, wsMachs] = addCase(wsNames, wsVals, wsAlts, wsMachs, ...
    'CruiseClimbRequirement', ...
    Mission.CruiseStudy.CruiseClimbRequirement.WS, ...
    Mission.CruiseStudy.CruiseClimbRequirement.Altitude, ...
    Mission.CruiseStudy.CruiseClimbRequirement.Mach);

[wsNames, wsVals, wsAlts, wsMachs] = addCase(wsNames, wsVals, wsAlts, wsMachs, ...
    'Approach', Mission.ApproachLanding.WS, 0, Mach_app);

[twMax, iTW] = max(twVals);
[wsMax, iWS] = max(wsVals);

CriticalTW.Value = twMax;
CriticalTW.Segment = twNames{iTW};
CriticalTW.Altitude_m = twAlts(iTW);
CriticalTW.Mach = twMachs(iTW);
CriticalTW.Description = sprintf('Critical T/W occurs in %s', twNames{iTW});

CriticalWS.Value = wsMax;
CriticalWS.Segment = wsNames{iWS};
CriticalWS.Altitude_m = wsAlts(iWS);
CriticalWS.Mach = wsMachs(iWS);
CriticalWS.Description = sprintf('Critical W/S occurs in %s', wsNames{iWS});

Mission.CriticalTW = CriticalTW;
Mission.CriticalWS = CriticalWS;

%% ============================================================
%% 20) Diagnostic plot
%% ============================================================
figure(11); clf;
tiledlayout(2,1)

nexttile
plot(Mission.Cruise.StepCL, Mission.Cruise.StepLD, 'LineWidth', 1.5)
hold on
plot(Mission.Cruise.CL, Mission.Cruise.LD, 'ro', 'MarkerSize', 8, 'LineWidth', 1.5)
xlabel('Lift Coefficient, C_L')
ylabel('Lift-to-Drag Ratio, L/D')
title('Stepped Cruise-Climb')
grid on

nexttile
plot(Cls, LDs, 'LineWidth', 1.5)
xlabel('Lift Coefficient, C_L')
ylabel('Lift-to-Drag Ratio, L/D')
title('Loiter Efficiency Sweep')
grid on

end

%% ========================== Local helpers ==============================

function [m_out, Seg] = flyVerticalSegment(ADP, m_in, h_start, h_end, dh, ...
    speedMode, speedValue, roc, segName)
% Unified climb/descent solver
% roc > 0 climb, roc < 0 descent

g = 9.81;

Seg = struct();

if h_start == h_end
    Seg = emptyVerticalSeg();
    m_out = m_in;
    return
end

if roc == 0
    error('flyVerticalSegment: roc cannot be zero.')
end

isClimb = h_end > h_start;
if isClimb && roc < 0
    error('flyVerticalSegment: climb requested with negative roc.')
end
if ~isClimb && roc > 0
    error('flyVerticalSegment: descent requested with positive roc.')
end

nSteps = max(0, ceil(abs(h_end - h_start) / dh));

Seg.StepFuel = zeros(nSteps,1);
Seg.StepMass = zeros(nSteps,1);
Seg.StepAlt  = zeros(nSteps,1);
Seg.StepMach = zeros(nSteps,1);
Seg.StepCL   = zeros(nSteps,1);
Seg.StepCD   = zeros(nSteps,1);
Seg.StepDrag = zeros(nSteps,1);
Seg.StepTreq = zeros(nSteps,1);
Seg.StepTime = zeros(nSteps,1);
Seg.StepV    = zeros(nSteps,1);
Seg.StepTW   = zeros(nSteps,1);
Seg.StepWS   = zeros(nSteps,1);
Seg.StepRange = zeros(nSteps,1);

m_current = m_in;
FuelTot = 0;
TimeTot = 0;
RangeTot = 0;

for i = 1:nSteps
    if isClimb
        h_low  = h_start + (i-1) * dh;
        h_high = min(h_low + dh, h_end);
    else
        h_high = h_start - (i-1) * dh;
        h_low  = max(h_high - dh, h_end);
    end

    h_mid = 0.5 * (h_low + h_high);
    dh_step = abs(h_high - h_low);

    [rho, a, ~, ~] = cast.atmos(h_mid);

    [V, Mach] = resolveSpeed(speedMode, speedValue, a);
    W = m_current * g;

    %% THIS IS CODE IVE ADDED - OSCAR


    % Files
    baseDir = fileparts(matlab.desktop.editor.getActiveFilename);
    cruisePath = fullfile(baseDir, '../+Unconventional/+lookup/+aerodynamics/cruise_lookup_table.mat');
    S = load(cruisePath);
    cruise_results = S.Results;
    
    
    % Cruise file
    S = load(cruisePath);
    cruise_results = S.Results;
    span = 73.5;
    mach = 0.81;
    rootChord = 15.2;
    altitude = 9500;
    
    function k = nearestIndex(vec,x)
    [~,k] = min(abs(vec-x));
    end
    
    span = S.spanVec(nearestIndex(S.spanVec,span));
    mach = S.machVec(nearestIndex(S.machVec,mach));
    rootChord = S.rootChordVec(nearestIndex(S.rootChordVec,rootChord));
    altitude = S.altVec(nearestIndex(S.altVec,altitude));
    
    i = [cruise_results.WingSpan] == span & [cruise_results.Mach] == mach & [cruise_results.RootChord] == rootChord & [cruise_results.Altitude] == altitude;
    
    A = [cruise_results(i).Alpha];
    CL = [cruise_results(i).CL];
    CD = [cruise_results(i).CD];
    
    [A,ord] = sort(A);
    CL = CL(ord);
    CD = CD(ord);
    
    alpha = 2;  
    
    
    
    % pick optimal altitude for cruise
    alts = linspace(8000,11000,12); % OSCAR: Level cruise is in my mind could be anywhere from 8000m to 11000m
    [rho,a,T,P] = cast.atmos(alts);
    % [rho_s,a_s,~,P_s] = dcrg.aero.atmos(0);
    M_cruise = ADP.TLAR.M_c;
    CL_c = interp1(A,CL,alpha,'linear','extrap');
    CD_c = interp1(A,CD,alpha,'linear','extrap');
    LD_c = CL_c./CD_c;
    %% THIS IS CODE IVE ADDED - OSCAR

    %CL = W / (0.5 * rho * V^2 * ADP.WingArea);
    %CD = ADP.AeroPolar.CD(CL);
    D  = 0.5 * rho * V^2 * ADP.WingArea * CD;

    T_raw = D + W * roc / V;
    T_idle = getIdleThrust(ADP, m_current, Mach, h_mid, 0.07);
    T_req = max(T_idle, T_raw);

    dt = dh_step / abs(roc);
    dR = V * dt;

    TSFC = ADP.Engine.TSFC(Mach, h_mid);
    FuelStep = TSFC * T_req * dt;

    [TW_step, WS_step] = thrustToWeightAndWingLoading(T_req, m_current, ADP.WingArea, g);

    m_current = m_current - FuelStep;
    FuelTot = FuelTot + FuelStep;
    TimeTot = TimeTot + dt;
    RangeTot = RangeTot + dR;

    Seg.StepFuel(i) = FuelStep;
    Seg.StepMass(i) = m_current;
    Seg.StepAlt(i)  = h_mid;
    Seg.StepMach(i) = Mach;
    Seg.StepCL(i)   = CL;
    Seg.StepCD(i)   = CD;
    Seg.StepDrag(i) = D;
    Seg.StepTreq(i) = T_req;
    Seg.StepTime(i) = dt;
    Seg.StepV(i)    = V;
    Seg.StepTW(i)   = TW_step;
    Seg.StepWS(i)   = WS_step;
    Seg.StepRange(i)= dR;
end

Seg.Fuel = FuelTot;
Seg.Time = TimeTot;
Seg.Range = RangeTot;
Seg.MaxTW = maxOrNaN(Seg.StepTW);
Seg.MaxWS = maxOrNaN(Seg.StepWS);
Seg.Name = segName;

m_out = m_current;
end

function [m_out, Seg] = flyCruiseSegment(ADP, m_in, rangeTarget, Mach, h_start, segName)
% Stepped cruise-climb with constant CL target

g = 9.81;
dR = 50e3;

Seg = struct();

nSteps = max(1, ceil(max(rangeTarget, 1) / dR));

Seg.StepFuel  = zeros(nSteps,1);
Seg.StepMass  = zeros(nSteps,1);
Seg.StepAlt   = zeros(nSteps,1);
Seg.StepMach  = zeros(nSteps,1);
Seg.StepCL    = zeros(nSteps,1);
Seg.StepCD    = zeros(nSteps,1);
Seg.StepLD    = zeros(nSteps,1);
Seg.StepDrag  = zeros(nSteps,1);
Seg.StepTreq  = zeros(nSteps,1);
Seg.StepTime  = zeros(nSteps,1);
Seg.StepRange = zeros(nSteps,1);
Seg.StepV     = zeros(nSteps,1);
Seg.StepTW    = zeros(nSteps,1);
Seg.StepWS    = zeros(nSteps,1);

m_current = m_in;
FuelTot = 0;
TimeTot = 0;
RangeActual = 0;
alt_step = h_start;

[rho0, a0, ~, ~] = cast.atmos(alt_step);
V0 = Mach * a0;
W0 = m_current * g;
CL_target = W0 / (0.5 * rho0 * V0^2 * ADP.WingArea);

for i = 1:nSteps
    dR_step = min(dR, rangeTarget - RangeActual);
    if dR_step <= 0
        break
    end

    [rho, a, ~, ~] = cast.atmos(alt_step);

    V = Mach * a;
    W = m_current * g;

    %% THIS IS CODE IVE ADDED - OSCAR


    % Files
    baseDir = fileparts(matlab.desktop.editor.getActiveFilename);
    cruisePath = fullfile(baseDir, '../+Unconventional/+lookup/+aerodynamics/cruise_lookup_table.mat');
    S = load(cruisePath);
    cruise_results = S.Results;
    
    
    % Cruise file
    S = load(cruisePath);
    cruise_results = S.Results;
    span = 73.5;
    mach = 0.81;
    rootChord = 15.2;
    altitude = 9500;
    
    function k = nearestIndex(vec,x)
    [~,k] = min(abs(vec-x));
    end
    
    span = S.spanVec(nearestIndex(S.spanVec,span));
    mach = S.machVec(nearestIndex(S.machVec,mach));
    rootChord = S.rootChordVec(nearestIndex(S.rootChordVec,rootChord));
    altitude = S.altVec(nearestIndex(S.altVec,altitude));
    
    i = [cruise_results.WingSpan] == span & [cruise_results.Mach] == mach & [cruise_results.RootChord] == rootChord & [cruise_results.Altitude] == altitude;
    
    A = [cruise_results(i).Alpha];
    CL = [cruise_results(i).CL];
    CD = [cruise_results(i).CD];
    
    [A,ord] = sort(A);
    CL = CL(ord);
    CD = CD(ord);
    
    alpha = 2;  
    
    
    
    % pick optimal altitude for cruise
    alts = linspace(8000,11000,12); % OSCAR: Level cruise is in my mind could be anywhere from 8000m to 11000m
    [rho,a,T,P] = cast.atmos(alts);
    % [rho_s,a_s,~,P_s] = dcrg.aero.atmos(0);
    M_cruise = ADP.TLAR.M_c;
    CL_c = interp1(A,CL,alpha,'linear','extrap');
    CD_c = interp1(A,CD,alpha,'linear','extrap');
    LD_c = CL_c./CD_c;
    %% THIS IS CODE IVE ADDED - OSCAR

    %CL = W / (0.5 * rho * V^2 * ADP.WingArea);
    %CD = ADP.AeroPolar.CD(CL);
    D  = 0.5 * rho * V^2 * ADP.WingArea * CD;
    LD = CL / CD;
    T_req = D;

    [TW_step, WS_step] = thrustToWeightAndWingLoading(T_req, m_current, ADP.WingArea, g);

    dt = dR_step / V;
    TSFC = ADP.Engine.TSFC(Mach, alt_step);
    FuelStep = TSFC * T_req * dt;

    m_current = m_current - FuelStep;
    FuelTot = FuelTot + FuelStep;
    TimeTot = TimeTot + dt;
    RangeActual = RangeActual + dR_step;

    Seg.StepFuel(i)  = FuelStep;
    Seg.StepMass(i)  = m_current;
    Seg.StepAlt(i)   = alt_step;
    Seg.StepMach(i)  = Mach;
    Seg.StepCL(i)    = CL;
    Seg.StepCD(i)    = CD;
    Seg.StepLD(i)    = LD;
    Seg.StepDrag(i)  = D;
    Seg.StepTreq(i)  = T_req;
    Seg.StepTime(i)  = dt;
    Seg.StepRange(i) = dR_step;
    Seg.StepV(i)     = V;
    Seg.StepTW(i)    = TW_step;
    Seg.StepWS(i)    = WS_step;

    alt_grid = linspace(h_start, 43e3 / SI.ft, 200);
    [rho_grid, a_grid, ~, ~] = cast.atmos(alt_grid);

    CL_grid = m_current * g ./ ...
        (0.5 .* rho_grid .* (Mach .* a_grid).^2 .* ADP.WingArea);

    [~, idxAlt] = min(abs(CL_grid - CL_target));
    alt_step = alt_grid(idxAlt);
end

Seg.Fuel = FuelTot;
Seg.Time = TimeTot;
Seg.Range = RangeActual;
Seg.Altitude = h_start;
Seg.FL = round(h_start * SI.ft / 100, 0);


%% THIS IS CODE IVE ADDED - OSCAR


% Files
baseDir = fileparts(matlab.desktop.editor.getActiveFilename);
cruisePath = fullfile(baseDir, '../+Unconventional/+lookup/+aerodynamics/cruise_lookup_table.mat');
S = load(cruisePath);
cruise_results = S.Results;


% Cruise file
S = load(cruisePath);
cruise_results = S.Results;
span = 73.5;
mach = 0.81;
rootChord = 15.2;
altitude = 9500;

function k = nearestIndex(vec,x)
[~,k] = min(abs(vec-x));
end

span = S.spanVec(nearestIndex(S.spanVec,span));
mach = S.machVec(nearestIndex(S.machVec,mach));
rootChord = S.rootChordVec(nearestIndex(S.rootChordVec,rootChord));
altitude = S.altVec(nearestIndex(S.altVec,altitude));

i = [cruise_results.WingSpan] == span & [cruise_results.Mach] == mach & [cruise_results.RootChord] == rootChord & [cruise_results.Altitude] == altitude;

A = [cruise_results(i).Alpha];
CL = [cruise_results(i).CL];
CD = [cruise_results(i).CD];

[A,ord] = sort(A);
CL = CL(ord);
CD = CD(ord);

alpha = 2;  



% pick optimal altitude for cruise
alts = linspace(8000,11000,12); % OSCAR: Level cruise is in my mind could be anywhere from 8000m to 11000m
[rho,a,T,P] = cast.atmos(alts);
% [rho_s,a_s,~,P_s] = dcrg.aero.atmos(0);
M_cruise = ADP.TLAR.M_c;
CL_c = interp1(A,CL,alpha,'linear','extrap');
CD_c = interp1(A,CD,alpha,'linear','extrap');
LD_c = CL_c./CD_c;
%% THIS IS CODE IVE ADDED - OSCAR

%Seg.CL = CL_target;
%Seg.CD = ADP.AeroPolar.CD(CL_target);
Seg.LD = CL_target / ADP.AeroPolar.CD(CL_target);
Seg.MaxTW = maxOrNaN(Seg.StepTW);
Seg.MaxWS = maxOrNaN(Seg.StepWS);
Seg.Name = segName;

m_out = m_current;
end

function Seg = emptyVerticalSeg()
Seg.StepFuel = [];
Seg.StepMass = [];
Seg.StepAlt = [];
Seg.StepMach = [];
Seg.StepCL = [];
Seg.StepCD = [];
Seg.StepDrag = [];
Seg.StepTreq = [];
Seg.StepTime = [];
Seg.StepV = [];
Seg.StepTW = [];
Seg.StepWS = [];
Seg.StepRange = [];
Seg.Fuel = 0;
Seg.Time = 0;
Seg.Range = 0;
Seg.MaxTW = NaN;
Seg.MaxWS = NaN;
Seg.Name = '';
end

function [V, Mach] = resolveSpeed(speedMode, speedValue, a)
switch upper(speedMode)
    case 'TAS'
        V = speedValue;
        Mach = V / a;
    case 'MACH'
        Mach = speedValue;
        V = Mach * a;
    otherwise
        error('Unknown speedMode: %s', speedMode)
end
end

function Fuel = segmentFuelFromThrust(ADP, Mach, alt_m, T_req, time_s)
TSFC = ADP.Engine.TSFC(Mach, alt_m);
Fuel = TSFC * T_req * time_s;
end

function T_avail = getAvailableThrust(ADP, mass_kg, Mach, alt_m)
g = 9.81;
if ismethod(ADP.Engine, 'Thrust')
    try
        T_avail = ADP.Engine.Thrust(Mach, alt_m);
    catch
        T_avail = 0.3 * mass_kg * g;
    end
else
    T_avail = 0.3 * mass_kg * g;
end
end

function T_idle = getIdleThrust(ADP, mass_kg, Mach, alt_m, idleFrac)
T_avail = getAvailableThrust(ADP, mass_kg, Mach, alt_m);
T_idle = idleFrac * T_avail;
end

function [TW, WS] = thrustToWeightAndWingLoading(T_req, mass_kg, wingArea, g)
W = mass_kg * g;
TW = T_req / W;
WS = W / wingArea;
end

function val = maxOrNaN(x)
if isempty(x)
    val = NaN;
else
    val = max(x);
end
end

function [names, vals, alts, machs] = addCase(names, vals, alts, machs, name, value, alt, mach)
if ~isnan(value)
    names{end+1} = name; %#ok<AGROW>
    vals(end+1,1) = value; %#ok<AGROW>
    alts(end+1,1) = alt; %#ok<AGROW>
    machs(end+1,1) = mach; %#ok<AGROW>
end
end

function [names, vals, alts, machs] = addMaxStepCase(names, vals, alts, machs, name, values, stepAlt, stepMach)
if isempty(values)
    return
end
[v, idx] = max(values);
if ~isempty(v) && ~isnan(v)
    names{end+1} = name; %#ok<AGROW>
    vals(end+1,1) = v; %#ok<AGROW>
    alts(end+1,1) = stepAlt(idx); %#ok<AGROW>
    machs(end+1,1) = stepMach(idx); %#ok<AGROW>
end
end

function [TW_required, T_required, out] = sizeTakeoffFromFieldLength(ADP, m_current, TakeoffFieldLength, varargin)
% sizeTakeoffFromFieldLength
%
% Preliminary ground-roll takeoff sizing from a required takeoff field length.

p = inputParser;
p.FunctionName = 'sizeTakeoffFromFieldLength';

addParameter(p, 'Altitude',       0.0,  @(x) isnumeric(x) && isscalar(x));
addParameter(p, 'CLmaxTO',        2.4,  @(x) isnumeric(x) && isscalar(x) && x > 0);
addParameter(p, 'CLgroundFactor', 0.7,  @(x) isnumeric(x) && isscalar(x) && x > 0);
addParameter(p, 'MuRoll',         0.02, @(x) isnumeric(x) && isscalar(x) && x >= 0);
addParameter(p, 'VlofFactor',     1.10, @(x) isnumeric(x) && isscalar(x) && x > 1.0);
addParameter(p, 'VavgFactor',     0.70, @(x) isnumeric(x) && isscalar(x) && x > 0 && x <= 1.0);
addParameter(p, 'GammaRunway',    0.0,  @(x) isnumeric(x) && isscalar(x));

parse(p, varargin{:});
opt = p.Results;

g = 9.81;
S = ADP.WingArea;
W = m_current * g;

if S <= 0
    error('sizeTakeoffFromFieldLength:InvalidWingArea', ...
        'ADP.WingArea must be positive.');
end

if TakeoffFieldLength <= 0
    error('sizeTakeoffFromFieldLength:InvalidFieldLength', ...
        'TakeoffFieldLength must be positive.');
end

[rho_to, a_to, ~, ~] = cast.atmos(opt.Altitude);

CLmaxTO = opt.CLmaxTO;
CLg = opt.CLgroundFactor * CLmaxTO;

Vstall_to = sqrt(2 * W / (rho_to * S * CLmaxTO));
Vlof      = opt.VlofFactor * Vstall_to;
Vavg      = opt.VavgFactor * Vlof;

q_avg = 0.5 * rho_to * Vavg^2;

try
    %CDg = ADP.AeroPolar.CD(CLg);

    %% THIS IS CODE IVE ADDED - OSCAR


    % Files
    baseDir = fileparts(matlab.desktop.editor.getActiveFilename);
    cruisePath = fullfile(baseDir, '../+Unconventional/+lookup/+aerodynamics/cruise_lookup_table.mat');
    S = load(cruisePath);
    cruise_results = S.Results;
    
    
    % Cruise file
    S = load(cruisePath);
    cruise_results = S.Results;
    span = 73.5;
    mach = 0.81;
    rootChord = 15.2;
    altitude = 9500;
    
    function k = nearestIndex(vec,x)
    [~,k] = min(abs(vec-x));
    end
    
    span = S.spanVec(nearestIndex(S.spanVec,span));
    mach = S.machVec(nearestIndex(S.machVec,mach));
    rootChord = S.rootChordVec(nearestIndex(S.rootChordVec,rootChord));
    altitude = S.altVec(nearestIndex(S.altVec,altitude));
    
    i = [cruise_results.WingSpan] == span & [cruise_results.Mach] == mach & [cruise_results.RootChord] == rootChord & [cruise_results.Altitude] == altitude;
    
    A = [cruise_results(i).Alpha];
    CL = [cruise_results(i).CL];
    CD = [cruise_results(i).CD];
    
    [A,ord] = sort(A);
    CL = CL(ord);
    CD = CD(ord);
    
    alpha = 2;  
    
    
    
    % pick optimal altitude for cruise
    alts = linspace(8000,11000,12); % OSCAR: Level cruise is in my mind could be anywhere from 8000m to 11000m
    [rho,a,T,P] = cast.atmos(alts);
    % [rho_s,a_s,~,P_s] = dcrg.aero.atmos(0);
    M_cruise = ADP.TLAR.M_c;
    CL_c = interp1(A,CL,alpha,'linear','extrap');
    CD_c = interp1(A,CD,alpha,'linear','extrap');
    LD_c = CL_c./CD_c;
    %% THIS IS CODE IVE ADDED - OSCAR

catch
    error('sizeTakeoffFromFieldLength:AeroPolarCallFailed', ...
        ['Could not evaluate ADP.AeroPolar.CD(CLg). Make sure ', ...
         'ADP.AeroPolar.CD accepts a CL input and returns CD.']);

    %% THIS IS CODE IVE ADDED - OSCAR


    % Files
    baseDir = fileparts(matlab.desktop.editor.getActiveFilename);
    cruisePath = fullfile(baseDir, '../+Unconventional/+lookup/+aerodynamics/cruise_lookup_table.mat');
    S = load(cruisePath);
    cruise_results = S.Results;
    
    
    % Cruise file
    S = load(cruisePath);
    cruise_results = S.Results;
    span = 73.5;
    mach = 0.81;
    rootChord = 15.2;
    altitude = 9500;
    
    function k = nearestIndex(vec,x)
    [~,k] = min(abs(vec-x));
    end
    
    span = S.spanVec(nearestIndex(S.spanVec,span));
    mach = S.machVec(nearestIndex(S.machVec,mach));
    rootChord = S.rootChordVec(nearestIndex(S.rootChordVec,rootChord));
    altitude = S.altVec(nearestIndex(S.altVec,altitude));
    
    i = [cruise_results.WingSpan] == span & [cruise_results.Mach] == mach & [cruise_results.RootChord] == rootChord & [cruise_results.Altitude] == altitude;
    
    A = [cruise_results(i).Alpha];
    CL = [cruise_results(i).CL];
    CD = [cruise_results(i).CD];
    
    [A,ord] = sort(A);
    CL = CL(ord);
    CD = CD(ord);
    
    alpha = 2;  
    
    
    
    % pick optimal altitude for cruise
    alts = linspace(8000,11000,12); % OSCAR: Level cruise is in my mind could be anywhere from 8000m to 11000m
    [rho,a,T,P] = cast.atmos(alts);
    % [rho_s,a_s,~,P_s] = dcrg.aero.atmos(0);
    M_cruise = ADP.TLAR.M_c;
    CL_c = interp1(A,CL,alpha,'linear','extrap');
    CD_c = interp1(A,CD,alpha,'linear','extrap');
    LD_c = CL_c./CD_c;
    %% THIS IS CODE IVE ADDED - OSCAR
end

L_avg = q_avg * S * CLg;
D_avg = q_avg * S * CDg;

L_W = L_avg / W;
D_W = D_avg / W;

a_avg = Vlof^2 / (2 * TakeoffFieldLength);

TW_required = ...
    a_avg / g + ...
    D_W + ...
    opt.MuRoll * (1 - L_W) + ...
    sin(opt.GammaRunway);

T_required = TW_required * W;

out = struct();

out.g                  = g;
out.Altitude           = opt.Altitude;
out.rho_to             = rho_to;
out.a_to               = a_to;

out.Mass               = m_current;
out.Weight             = W;
out.WingArea           = S;
out.WingLoading        = W / S;

out.TakeoffFieldLength = TakeoffFieldLength;

out.CLmaxTO            = CLmaxTO;
out.CLgroundFactor     = opt.CLgroundFactor;
out.CLground           = CLg;
out.CDground           = CDg;

out.Vstall_to          = Vstall_to;
out.Vlof               = Vlof;
out.Vavg               = Vavg;

out.q_avg              = q_avg;
out.L_avg              = L_avg;
out.D_avg              = D_avg;
out.L_W                = L_W;
out.D_W                = D_W;

out.MuRoll             = opt.MuRoll;
out.GammaRunway        = opt.GammaRunway;

out.a_avg              = a_avg;
out.TW_required        = TW_required;
out.T_required         = T_required;

out.EstimatedGroundRollTime = Vlof / a_avg;
out.EstimatedRange          = TakeoffFieldLength;
end