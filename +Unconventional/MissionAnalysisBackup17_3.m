function [BlockFuel,TripFuel,ResFuel,Mf_TOC,MissionTime,cruise_FL] = MissionAnalysis(ADP,tripRange,M_TO)
%MISSIONANALYSIS conduct mission analysis to estimate fuel burn
arguments
    ADP % geometry object
    tripRange % mission range in m
    M_TO = ADP.MTOM; % take off mass
end



EWF = 1;   % empty weight fraction
fs = double.empty;
ts = double.empty;

%% --- Trip: taxi + takeoff (empirical) ---

f_taxi  = 0.970;

fs(1) = f_taxi;
EWF = EWF * fs(1);

%% climb

f_climb = 0.985;


fs(2) = f_climb;
EWF = EWF * fs(2);

%% cruise analysis (assume constant C_L)

% pick optimal altitude for cruise
alts = linspace(15e3./SI.ft,44e3./SI.ft,61);
[rho,a,T,P] = cast.atmos(alts);
% [rho_s,a_s,~,P_s] = dcrg.aero.atmos(0);
M_cruise = ADP.TLAR.M_c;
CL_c = EWF*M_TO*9.81./(1/2.*rho.*(a.*M_cruise).^2.*ADP.WingArea); % cruise C_L
CD_c = ADP.AeroPolar.CD(CL_c);
LD_c = CL_c./CD_c;
[~,idx] = max(LD_c);

alt = alts(idx);
CL_c = CL_c(idx);
CD_c = CD_c(idx);
LD_c = CL_c/CD_c;
[~,a,~,~] = cast.atmos(alt);
cruise_FL = round(alt.*SI.ft/1e2,0);
disp(cruise_FL)

Cls = 0.4:0.01:0.8;
LDs = Cls*0;
for i = 1:length(Cls)
   LDs(i) =  Cls(i)/ADP.AeroPolar.CD(Cls(i));
end
f = figure(11);clf;plot(Cls,LDs)

% account for fact I don't model climb with an "effective" trip range
tripRange = tripRange * 1;
fs(3) = exp(-tripRange*9.81*ADP.Engine.TSFC(M_cruise,alt)/(M_cruise*a*LD_c)); % Rearranged Brequet
ts(3) = tripRange/(M_cruise*a); % time taken
EWF = EWF*fs(3);

%% --- Trip: descent ---

f_desc = 0.995;

fs(4) = f_desc;
EWF = EWF * fs(4);

%% approach

f_app  = 0.995;

fs(5) = f_app;
EWF = EWF * fs(5);


%% Contingency
df = (1-EWF)*0.03;
fs(6) = 1-df/EWF;
ts(6) = 5*60; % 5 minutes...
EWF = EWF*fs(6);


%% Reserve climb
fs(7) = 0.985;
EWF = EWF * fs(7);

%% reserve alternate mission analysis
[rho,a,~,P] = cast.atmos(ADP.TLAR.Alt_alternate);
% [rho_s,a_s,~,P_s] = dcrg.aero.atmos(0);
M_cruise = ADP.TLAR.M_c;

CL_c = EWF*M_TO*9.81/(1/2*rho*(a*M_cruise)^2*ADP.WingArea); % cruise C_L
CD_c = ADP.AeroPolar.CD(CL_c);
LD_c = CL_c/CD_c;

% account for fact I don't model climb with an "effective" trip range
altRange = ADP.TLAR.Range_alternate * 1;

fs(8) = exp(-altRange*9.81*ADP.Engine.TSFC(M_cruise,ADP.TLAR.Alt_alternate)/(M_cruise*a*LD_c)); % Rearranged Brequet
ts(8) = altRange/(M_cruise*a); % time taken
EWF = EWF*fs(8);


%% Reserve descent
fs(9) = 0.995;
EWF = EWF * fs(9);

%% loiter
[rho,a,~,P] = cast.atmos(0);
Mach = 150/a;
CL = EWF*M_TO*9.81/(1/2*rho*(a*Mach)^2*ADP.WingArea);
CD = ADP.AeroPolar.CD(CL);
LD = CL/CD;

fs(10) = exp(-ADP.TLAR.Loiter*9.81*ADP.Engine.TSFC(Mach,0)/LD); % Snorri
ts(10) = ADP.TLAR.Loiter; % time taken
EWF = EWF*fs(10);

%% Reserve approach
fs(11) = 0.995;
EWF = EWF * fs(11);

%% Taxi to gate
fs(12) = 0.997;
EWF = EWF * fs(12);

%% update model

BlockFuel = (1 - EWF) * M_TO;

% Trip fuel = taxi → climb → cruise → descent → approach → taxi to gate
TripFuel = (1 - prod(fs([1:5 12]))) * M_TO;

% Reserve fuel = everything else
ResFuel = BlockFuel - TripFuel;

Mf_TOC = prod(fs(1:2));

MissionTime = ts(3) + ts(6) + ts(8) + ts(10);
end