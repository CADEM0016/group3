function [BlockFuel, TripFuel, ResFuel, Mf_TOC, Mf_Land, Mf_AltStart, ...
          MissionTime, Mission, CLmax_land_req, CLmax_to_req] = ...
          MissionAnalysisDynamic(ADP, tripRange, M_TO)
%MISSIONANALYSIS Conduct mission analysis using nominal mission profile

arguments
    ADP
    tripRange
    M_TO = ADP.MTOM
end

%% ----------------------------- Constants ------------------------------
g = 9.81;
rho0 = 1.225;

%% ---------------------- Mission assumptions ---------------------------
TaxiTime_out   = 20 * 60;      % s
TaxiTime_in    = 20 * 60;      % s
TakeoffTime    = 1 * 60;       % s
ClimbTime      = 30 * 60;      % s
ApproachTime   = 10 * 60;      % s
LoiterTime     = 30 * 60;      % s
ContTime       = 5 * 60;       % s
AltRange       = 350e3;        % m

%Altitude Sections

ALT_TO      = 0;        % m (runway)
ALT_CLEAN   = 1500;     % ft
ALT_10K     = 10000;    % ft
ALT_ICA     = 20000;    % ft (initial cruise altitude requirement)

gamma_to    = 10 * pi / 180;                 % rad
ROC_to = 1500 * 0.3048 / 60;   % m/s (0 → 1500 ft in 60 s)
ROC_climb_1 = (ALT_ICA - ALT_CLEAN) * 0.3048 / (25 * 60);
ROC_climb_2 = ROC_climb_1;
ROC_climb_3 = ROC_climb_1; %MOVE THIS DOWN

V_climb_1 = 245 * 0.514444;   % m/s
V_climb_2 = 300 * 0.514444;   % m/s

%cruise and takeoff fractions already know
IdleFrac     = 0.07;
ClimbFrac = 319.60e3 / ADP.Engine.T_Static; %Max continuous/maxthrust
DescentFrac  = 0.12;
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
%% 1) Cruise analysis (choose best cruise altitude)
%% ============================================================
alts = linspace(20e3 ./ SI.ft, 44e3 ./ SI.ft, 49);
[rho_all, a_all, T, P] = cast.atmos(alts);

CL_all = m_current * g ./ (0.5 .* rho_all .* (a_all .* M_cruise).^2 .* ADP.WingArea);
CD_all = ADP.AeroPolar.CD(CL_all);
LD_all = CL_all ./ CD_all;

[~, idx] = max(LD_all);

alt   = alts(idx);
rho_c = rho_all(idx);
a_c   = a_all(idx);
CL_c  = CL_all(idx);
CD_c  = CD_all(idx);
LD_c  = LD_all(idx);

cruise_FL = round(alt .* SI.ft / 1e2, 0); %

%% ------------------- Representative low-speed points -------------------
[rho_to_ref, a_to_ref, ~, ~] = cast.atmos(0);
CL_to_ref = M_TO * g / (0.5 * rho_to_ref * (a_to_ref * M_to)^2 * ADP.WingArea); %#ok<NASGU>
CD_to_ref = ADP.AeroPolar.CD(CL_to_ref); %#ok<NASGU>

CL_ds = M_TO * g / (0.5 * rho_to_ref * (a_to_ref * M_ds)^2 * ADP.WingArea);
CD_ds = ADP.AeroPolar.CD(CL_ds); %#ok<NASGU>

%% ============================================================
%% 2) Taxi out
%% ============================================================
FuelTaxiOut = segmentFuelFromThrustFrac(ADP, m_current, 0.0, 0, IdleFrac, TaxiTime_out);
m_current = m_current - FuelTaxiOut;

Mission.TaxiOut.Fuel = FuelTaxiOut;
Mission.TaxiOut.Time = TaxiTime_out;

%% ============================================================
%% 3) Take-off / initial climb / acceleration
%% ============================================================
[rho_to, a_to, ~, ~] = cast.atmos(0);
V_to = M_to * a_to;
W = m_current * g;

CL_to = W * cos(gamma_to) / (0.5 * rho_to * V_to^2 * ADP.WingArea);
CD_to = ADP.AeroPolar.CD(CL_to);
D_to  = 0.5 * rho_to * V_to^2 * ADP.WingArea * CD_to;

T_req_to = D_to + W * ROC_to / V_to;
FuelTakeoff = segmentFuelFromThrust(ADP, M_to, 0, T_req_to, TakeoffTime);

m_current = m_current - FuelTakeoff;

Mission.Takeoff.Fuel = FuelTakeoff;
Mission.Takeoff.Time = TakeoffTime;
Mission.Takeoff.CL = CL_to;
Mission.Takeoff.CD = CD_to;
Mission.Takeoff.Drag = D_to;
Mission.Takeoff.Treq = T_req_to;
Mission.Takeoff.V = V_to;

%% ============================================================
%% 4) Climb 1: 1500 ft to 10000 ft in 500 ft steps
%% ============================================================
h_start = 1500 / SI.ft;
h_end   = 10000 / SI.ft;
dh      = 500 / SI.ft;

nSteps = ceil((h_end - h_start) / dh);

FuelClimb1 = 0;
TimeClimb1 = 0;

Mission.TakeoffClimb.StepFuel = zeros(nSteps,1);
Mission.TakeoffClimb.StepMass = zeros(nSteps,1);
Mission.TakeoffClimb.StepAlt  = zeros(nSteps,1);
Mission.TakeoffClimb.StepMach = zeros(nSteps,1);
Mission.TakeoffClimb.StepCL   = zeros(nSteps,1);
Mission.TakeoffClimb.StepCD   = zeros(nSteps,1);
Mission.TakeoffClimb.StepDrag = zeros(nSteps,1);
Mission.TakeoffClimb.StepTreq = zeros(nSteps,1);
Mission.TakeoffClimb.StepTime = zeros(nSteps,1);
Mission.TakeoffClimb.StepV    = zeros(nSteps,1);

for i = 1:nSteps
    h_low = h_start + (i-1) * dh;
    h_high = min(h_low + dh, h_end);
    h_mid = 0.5 * (h_low + h_high);

    [rho, a, ~, ~] = cast.atmos(h_mid);

    Mach = V_climb_1 / a;
    W = m_current * g;

    CL_climb_1 = W / (0.5 * rho * V_climb_1^2 * ADP.WingArea);
    CD_climb_1 = ADP.AeroPolar.CD(CL_climb_1);
    D_climb_1  = 0.5 * rho * V_climb_1^2 * ADP.WingArea * CD_climb_1;

    T_req_climb_1 = D_climb_1 + W * ROC_climb_1 / V_climb_1;

    dh_step = h_high - h_low;
    dt = dh_step / ROC_climb_1;

    TSFC = ADP.Engine.TSFC(Mach, h_mid);
    FuelStep = TSFC * T_req_climb_1 * dt;

    m_current = m_current - FuelStep;
    FuelClimb1 = FuelClimb1 + FuelStep;
    TimeClimb1 = TimeClimb1 + dt;

    Mission.TakeoffClimb.StepFuel(i) = FuelStep;
    Mission.TakeoffClimb.StepMass(i) = m_current;
    Mission.TakeoffClimb.StepAlt(i)  = h_mid;
    Mission.TakeoffClimb.StepMach(i) = Mach;
    Mission.TakeoffClimb.StepCL(i)   = CL_climb_1;
    Mission.TakeoffClimb.StepCD(i)   = CD_climb_1;
    Mission.TakeoffClimb.StepDrag(i) = D_climb_1;
    Mission.TakeoffClimb.StepTreq(i) = T_req_climb_1;
    Mission.TakeoffClimb.StepTime(i) = dt;
    Mission.TakeoffClimb.StepV(i)    = V_climb_1;
end

Mission.TakeoffClimb.Fuel = FuelClimb1;
Mission.TakeoffClimb.Time = TimeClimb1;

%% ============================================================
%% 5) Climb 2: 10000 ft to 20000 ft in 500 ft steps
%% ============================================================
h_start = 10000 / SI.ft;
h_end   = 20000 / SI.ft;
dh      = 500 / SI.ft;

nSteps = ceil((h_end - h_start) / dh);

FuelClimb2 = 0;
TimeClimb2 = 0;

Mission.Climb2.StepFuel = zeros(nSteps,1);
Mission.Climb2.StepMass = zeros(nSteps,1);
Mission.Climb2.StepAlt  = zeros(nSteps,1);
Mission.Climb2.StepMach = zeros(nSteps,1);
Mission.Climb2.StepCL   = zeros(nSteps,1);
Mission.Climb2.StepCD   = zeros(nSteps,1);
Mission.Climb2.StepDrag = zeros(nSteps,1);
Mission.Climb2.StepTreq = zeros(nSteps,1);
Mission.Climb2.StepTime = zeros(nSteps,1);
Mission.Climb2.StepV    = zeros(nSteps,1);

for i = 1:nSteps
    h_low = h_start + (i-1) * dh;
    h_high = min(h_low + dh, h_end);
    h_mid = 0.5 * (h_low + h_high);

    [rho, a, ~, ~] = cast.atmos(h_mid);

    Mach = V_climb_2 / a;
    W = m_current * g;

    CL_climb_2 = W / (0.5 * rho * V_climb_2^2 * ADP.WingArea);
    CD_climb_2 = ADP.AeroPolar.CD(CL_climb_2);
    D_climb_2  = 0.5 * rho * V_climb_2^2 * ADP.WingArea * CD_climb_2;

    T_req_climb_2 = D_climb_2 + W * ROC_climb_2 / V_climb_2;

    dh_step = h_high - h_low;
    dt = dh_step / ROC_climb_2;

    TSFC = ADP.Engine.TSFC(Mach, h_mid);
    FuelStep = TSFC * T_req_climb_2 * dt;

    m_current = m_current - FuelStep;
    FuelClimb2 = FuelClimb2 + FuelStep;
    TimeClimb2 = TimeClimb2 + dt;

    Mission.Climb2.StepFuel(i) = FuelStep;
    Mission.Climb2.StepMass(i) = m_current;
    Mission.Climb2.StepAlt(i)  = h_mid;
    Mission.Climb2.StepMach(i) = Mach;
    Mission.Climb2.StepCL(i)   = CL_climb_2;
    Mission.Climb2.StepCD(i)   = CD_climb_2;
    Mission.Climb2.StepDrag(i) = D_climb_2;
    Mission.Climb2.StepTreq(i) = T_req_climb_2;
    Mission.Climb2.StepTime(i) = dt;
    Mission.Climb2.StepV(i)    = V_climb_2;
end

Mission.Climb2.Fuel = FuelClimb2;
Mission.Climb2.Time = TimeClimb2;

%% ============================================================
%% 6) Climb 3: 20000 ft to cruise altitude in 500 ft steps
%% ============================================================
h_start = 20000 / SI.ft;
h_end   = alt;
dh      = 500 / SI.ft;

nSteps = max(0, ceil((h_end - h_start) / dh));

FuelClimb3 = 0;
TimeClimb3 = 0;

Mission.Climb3.StepFuel = zeros(nSteps,1);
Mission.Climb3.StepMass = zeros(nSteps,1);
Mission.Climb3.StepAlt  = zeros(nSteps,1);
Mission.Climb3.StepMach = zeros(nSteps,1);
Mission.Climb3.StepCL   = zeros(nSteps,1);
Mission.Climb3.StepCD   = zeros(nSteps,1);
Mission.Climb3.StepDrag = zeros(nSteps,1);
Mission.Climb3.StepTreq = zeros(nSteps,1);
Mission.Climb3.StepTime = zeros(nSteps,1);
Mission.Climb3.StepV    = zeros(nSteps,1);

for i = 1:nSteps
    h_low = h_start + (i-1) * dh;
    h_high = min(h_low + dh, h_end);
    h_mid = 0.5 * (h_low + h_high);

    [rho, a, ~, ~] = cast.atmos(h_mid);

    Mach = min(ADP.TLAR.M_c, 0.80);
    V_climb_3 = Mach * a;
    W = m_current * g;

    CL_climb_3 = W / (0.5 * rho * V_climb_3^2 * ADP.WingArea);
    CD_climb_3 = ADP.AeroPolar.CD(CL_climb_3);
    D_climb_3  = 0.5 * rho * V_climb_3^2 * ADP.WingArea * CD_climb_3;

    T_req_climb_3 = D_climb_3 + W * ROC_climb_3 / V_climb_3;

    dh_step = h_high - h_low;
    dt = dh_step / ROC_climb_3;

    TSFC = ADP.Engine.TSFC(Mach, h_mid);
    FuelStep = TSFC * T_req_climb_3 * dt;

    m_current = m_current - FuelStep;
    FuelClimb3 = FuelClimb3 + FuelStep;
    TimeClimb3 = TimeClimb3 + dt;

    Mission.Climb3.StepFuel(i) = FuelStep;
    Mission.Climb3.StepMass(i) = m_current;
    Mission.Climb3.StepAlt(i)  = h_mid;
    Mission.Climb3.StepMach(i) = Mach;
    Mission.Climb3.StepCL(i)   = CL_climb_3;
    Mission.Climb3.StepCD(i)   = CD_climb_3;
    Mission.Climb3.StepDrag(i) = D_climb_3;
    Mission.Climb3.StepTreq(i) = T_req_climb_3;
    Mission.Climb3.StepTime(i) = dt;
    Mission.Climb3.StepV(i)    = V_climb_3;
end

Mission.Climb3.Fuel = FuelClimb3;
Mission.Climb3.Time = TimeClimb3;

FuelClimb = FuelClimb1 + FuelClimb2 + FuelClimb3;
TimeClimb = TimeClimb1 + TimeClimb2 + TimeClimb3;

Mf_TOC = m_current / M_TO;

%% ============================================================
%% 7) Cruise: stepped cruise-climb with constant CL target
%% ============================================================
RangeClimb1 = sum(Mission.TakeoffClimb.StepTime .* Mission.TakeoffClimb.StepV);
RangeClimb2 = sum(Mission.Climb2.StepTime .* Mission.Climb2.StepV);
RangeClimb3 = sum(Mission.Climb3.StepTime .* Mission.Climb3.StepV);

RangeClimbTotal = RangeClimb1 + RangeClimb2 + RangeClimb3;

RangeCruise = tripRange - 2 * RangeClimbTotal;
RangeCruise = max(RangeCruise, 0);

dR = 50e3;
nSteps = max(1, ceil(RangeCruise / dR));

FuelCruise = 0;
TimeCruise = 0;
RangeCruiseActual = 0;

alt_cruise_step = alt;

[rho_c0, a_c0, ~, ~] = cast.atmos(alt_cruise_step);
V_c0 = M_cruise * a_c0;
W_c0 = m_current * g;

CL_target = W_c0 / (0.5 * rho_c0 * V_c0^2 * ADP.WingArea);

Mission.Cruise.StepFuel  = zeros(nSteps,1);
Mission.Cruise.StepMass  = zeros(nSteps,1);
Mission.Cruise.StepAlt   = zeros(nSteps,1);
Mission.Cruise.StepMach  = zeros(nSteps,1);
Mission.Cruise.StepCL    = zeros(nSteps,1);
Mission.Cruise.StepCD    = zeros(nSteps,1);
Mission.Cruise.StepLD    = zeros(nSteps,1);
Mission.Cruise.StepDrag  = zeros(nSteps,1);
Mission.Cruise.StepTreq  = zeros(nSteps,1);
Mission.Cruise.StepTime  = zeros(nSteps,1);
Mission.Cruise.StepRange = zeros(nSteps,1);
Mission.Cruise.StepV     = zeros(nSteps,1);

for i = 1:nSteps
    dR_step = min(dR, RangeCruise - RangeCruiseActual);
    if dR_step <= 0
        break
    end

    [rho, a, ~, ~] = cast.atmos(alt_cruise_step);

    V = M_cruise * a;
    W = m_current * g;

    CL = W / (0.5 * rho * V^2 * ADP.WingArea);
    CD = ADP.AeroPolar.CD(CL);
    D  = 0.5 * rho * V^2 * ADP.WingArea * CD;
    LD = CL / CD;

    T_req = D;
    dt = dR_step / V;

    TSFC = ADP.Engine.TSFC(M_cruise, alt_cruise_step);
    FuelStep = TSFC * T_req * dt;

    m_current = m_current - FuelStep;
    FuelCruise = FuelCruise + FuelStep;
    TimeCruise = TimeCruise + dt;
    RangeCruiseActual = RangeCruiseActual + dR_step;

    Mission.Cruise.StepFuel(i)  = FuelStep;
    Mission.Cruise.StepMass(i)  = m_current;
    Mission.Cruise.StepAlt(i)   = alt_cruise_step;
    Mission.Cruise.StepMach(i)  = M_cruise;
    Mission.Cruise.StepCL(i)    = CL;
    Mission.Cruise.StepCD(i)    = CD;
    Mission.Cruise.StepLD(i)    = LD;
    Mission.Cruise.StepDrag(i)  = D;
    Mission.Cruise.StepTreq(i)  = T_req;
    Mission.Cruise.StepTime(i)  = dt;
    Mission.Cruise.StepRange(i) = dR_step;
    Mission.Cruise.StepV(i)     = V;

    alt_grid = linspace(alt, 43e3 / SI.ft, 200);
    [rho_grid, a_grid, ~, ~] = cast.atmos(alt_grid);

    CL_grid = m_current * g ./ ...
        (0.5 .* rho_grid .* (M_cruise .* a_grid).^2 .* ADP.WingArea);

    [~, idxAlt] = min(abs(CL_grid - CL_target));
    alt_cruise_step = alt_grid(idxAlt);
end

Mission.Cruise.Fuel = FuelCruise;
Mission.Cruise.Time = TimeCruise;
Mission.Cruise.Range = RangeCruiseActual;
Mission.Cruise.Altitude = alt;
Mission.Cruise.FL = round(alt * SI.ft / 100, 0);
Mission.Cruise.CL = CL_target;
Mission.Cruise.CD = ADP.AeroPolar.CD(CL_target);
Mission.Cruise.LD = CL_target / ADP.AeroPolar.CD(CL_target);

%% ============================================================
%% 8) Descent
%% ============================================================
TimeDescent = max(15 * 60, alt / 10);
FuelDescent = segmentFuelFromThrustFrac(ADP, m_current, M_desc, alt/2, DescentFrac, TimeDescent);
m_current = m_current - FuelDescent;

Mission.Descent.Fuel = FuelDescent;
Mission.Descent.Time = TimeDescent;

%% ============================================================
%% 9) Approach / landing
%% ============================================================
FuelApproach = segmentFuelFromThrustFrac(ADP, m_current, M_ds, 0, ApproachFrac, ApproachTime);
m_current = m_current - FuelApproach;

Mission.ApproachLanding.Fuel = FuelApproach;
Mission.ApproachLanding.Time = ApproachTime;

Mf_Land = m_current / M_TO;
Mf_AltStart = Mf_Land;

%% ============================================================
%% 10) Taxi in
%% ============================================================
FuelTaxiIn = segmentFuelFromThrustFrac(ADP, m_current, 0.0, 0, IdleFrac, TaxiTime_in);
m_current = m_current - FuelTaxiIn;

Mission.TaxiIn.Fuel = FuelTaxiIn;
Mission.TaxiIn.Time = TaxiTime_in;

%% ============================================================
%% 11) Alternate climb
%% ============================================================
FuelAltClimb = segmentFuelFromThrustFrac(ADP, m_current, M_climb, ADP.TLAR.Alt_alternate/2, ClimbFrac, ClimbTime);
m_current = m_current - FuelAltClimb;

Mission.Alternate.Climb.Fuel = FuelAltClimb;
Mission.Alternate.Climb.Time = ClimbTime;

%% ============================================================
%% 12) Alternate cruise
%% ============================================================
[rho_alt, a_alt, ~, ~] = cast.atmos(ADP.TLAR.Alt_alternate);

CL_alt = m_current * g / (0.5 * rho_alt * (a_alt * M_cruise)^2 * ADP.WingArea);
CD_alt = ADP.AeroPolar.CD(CL_alt);
LD_alt = CL_alt / CD_alt;

fs_alt = exp(-AltRange * g * ADP.Engine.TSFC(M_cruise, ADP.TLAR.Alt_alternate) / ...
            (M_cruise * a_alt * LD_alt));

FuelAltCruise = (1 - fs_alt) * m_current;
TimeAltCruise = AltRange / (M_cruise * a_alt);
m_current = m_current * fs_alt;

Mission.Alternate.Cruise.Fuel = FuelAltCruise;
Mission.Alternate.Cruise.Time = TimeAltCruise;

%% ============================================================
%% 13) Alternate descent
%% ============================================================
TimeAltDescent = max(10 * 60, ADP.TLAR.Alt_alternate / 10);
FuelAltDescent = segmentFuelFromThrustFrac(ADP, m_current, M_desc, ADP.TLAR.Alt_alternate/2, DescentFrac, TimeAltDescent);
m_current = m_current - FuelAltDescent;

Mission.Alternate.Descent.Fuel = FuelAltDescent;
Mission.Alternate.Descent.Time = TimeAltDescent;

%% ============================================================
%% 14) Alternate approach
%% ============================================================
FuelAltApproach = segmentFuelFromThrustFrac(ADP, m_current, M_ds, 0, ApproachFrac, ApproachTime);
m_current = m_current - FuelAltApproach;

Mission.Alternate.Approach.Fuel = FuelAltApproach;
Mission.Alternate.Approach.Time = ApproachTime;

%% ============================================================
%% 15) Loiter at 1500 ft for 30 min at minimum drag velocity
%% ============================================================
alt_loiter = 1500 / SI.ft;
[rho_l, a_l, ~, ~] = cast.atmos(alt_loiter);

Cls = 0.2:0.01:2.5;
LDs = Cls ./ ADP.AeroPolar.CD(Cls);
[~, idxL] = max(LDs);

CL_loiter = Cls(idxL);
CD_loiter = ADP.AeroPolar.CD(CL_loiter);
LD_loiter = CL_loiter / CD_loiter;

V_loiter = sqrt((2 * m_current * g) / (rho_l * ADP.WingArea * CL_loiter));
M_loiter = V_loiter / a_l;

FuelLoiter = (1 - exp(-LoiterTime * g * ADP.Engine.TSFC(M_loiter, alt_loiter) / LD_loiter)) * m_current;
m_current = m_current - FuelLoiter;

Mission.Loiter.Fuel = FuelLoiter;
Mission.Loiter.Time = LoiterTime;
Mission.Loiter.CL = CL_loiter;
Mission.Loiter.CD = CD_loiter;
Mission.Loiter.LD = LD_loiter;

%% ============================================================
%% 16) Contingency
%% ============================================================
TripFuel_preCont = FuelTaxiOut + FuelTakeoff + FuelClimb + FuelCruise + ...
                   FuelDescent + FuelApproach + FuelTaxiIn;

Fuel5min = (1 - exp(-ContTime * g * ADP.Engine.TSFC(M_loiter, alt_loiter) / LD_loiter)) * m_current;
FuelCont = max(Fuel5min, 0.03 * TripFuel_preCont);

m_current = m_current - FuelCont;

Mission.Contingency.Fuel = FuelCont;
Mission.Contingency.Time = ContTime;

%% ============================================================
%% 17) Totals
%% ============================================================
TripFuel = FuelTaxiOut + FuelTakeoff + FuelClimb + FuelCruise + ...
           FuelDescent + FuelApproach + FuelTaxiIn;

ResFuel = FuelAltClimb + FuelAltCruise + FuelAltDescent + ...
          FuelAltApproach + FuelLoiter + FuelCont;

BlockFuel = TripFuel + ResFuel;

MissionTime = TaxiTime_out + TakeoffTime + TimeClimb + TimeCruise + ...
              TimeDescent + ApproachTime + TaxiTime_in + ...
              ClimbTime + TimeAltCruise + TimeAltDescent + ...
              ApproachTime + LoiterTime + ContTime;

Mission.Total.TripFuel = TripFuel;
Mission.Total.ResFuel = ResFuel;
Mission.Total.BlockFuel = BlockFuel;
Mission.Total.Time = MissionTime;

%% ============================================================
%% 18) Required CLmax estimates from mission weights
%% ============================================================
Vapp_max = 145 * 0.514444;
Vs_land = Vapp_max / 1.3;

W_land = Mf_Land * M_TO * g;
CLmax_land_req = W_land / (0.5 * rho0 * Vs_land^2 * ADP.WingArea);

Vlof = 1.2 * Vs_land;
Vs_to = Vlof / 1.2;
W_to = M_TO * g;

CLmax_to_req = W_to / (0.5 * rho0 * Vs_to^2 * ADP.WingArea);

%% ============================================================
%% 19) Diagnostic plot
%% ============================================================
figure(11); clf;
tiledlayout(2,1)

nexttile
plot(CL_all, LD_all, 'LineWidth', 1.5)
hold on
plot(CL_c, LD_c, 'ro', 'MarkerSize', 8, 'LineWidth', 1.5)
xlabel('Lift Coefficient, C_L')
ylabel('Lift-to-Drag Ratio, L/D')
title('Cruise Efficiency Sweep')
grid on

nexttile
plot(Cls, LDs, 'LineWidth', 1.5)
xlabel('Lift Coefficient, C_L')
ylabel('Lift-to-Drag Ratio, L/D')
title('Loiter Efficiency Sweep')
grid on

end

function Fuel = segmentFuelFromThrust(ADP, Mach, alt_m, T_req, time_s)
TSFC = ADP.Engine.TSFC(Mach, alt_m);
Fuel = TSFC * T_req * time_s;
end

function Fuel = segmentFuelFromThrustFrac(ADP, mass_kg, Mach, alt_m, thrustFrac, time_s)
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

TSFC = ADP.Engine.TSFC(Mach, alt_m);
Fuel = TSFC * thrustFrac * T_avail * time_s;
end