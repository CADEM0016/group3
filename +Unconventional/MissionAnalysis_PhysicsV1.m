function [BlockFuel,TripFuel,ResFuel,Mf_TOC,MissionTime,cruise_FL] = MissionAnalysis_PhysicsV1(ADP,tripRange,M_TO)
%MISSIONANALYSIS conduct mission analysis to estimate fuel burn
arguments
    ADP % geometry object
    tripRange % mission range in m
    M_TO = ADP.MTOM; % take off mass
 
end

M_c = ADP.TLAR.M_c;
Alt_max    = ADP.TLAR.Alt_max;
Alt_cruise = ADP.TLAR.Alt_cruise;
V_climb = ADP.TLAR.V_climb;
GroundRun = ADP.TLAR.GroundRun;
M_climb = ADP.TLAR.M_climb;


%% THIS IS CODE IVE ADDED - OSCAR


EWF = 1;   % empty weight fraction
fs = double.empty;
ts = double.empty;

%% Physics setup

IdleFrac     = 0.07; % fraction of thrust at idle
ClimbFrac = 0.85;

N_eng = 4;

g = 9.81;
S = ADP.WingArea;

%% --- Trip: taxi + takeoff (empirical) ---
%OLD
%f_taxi  = 0.970;
%fs(1) = f_taxi;

%NEW

% --- Taxi --- 
T2W = ADP.ThrustToWeightRatio;
[rho, a, ~, ~] = cast.atmos(0);


TaxiTime = 20*60;
T_idle = IdleFrac * T2W * M_TO * g; % fallback thrust
FuelTaxi = ADP.Engine.TSFC(0,0) * T_idle * TaxiTime;

%fs(1) = 1 - FuelTaxi / M_TO;
%EWF = EWF * fs(1);

k = length(fs) + 1;

fs(k) = 1 - FuelTaxi / M_TO;
ts(k) = TaxiTime;   % if time exists

EWF = EWF * fs(k);

% --- Ground roll ---

% --- Assumptions / inputs ---
CLmax_TO = 2.4;        % takeoff max lift coefficient ------ OSCAR
CD_TO    = 0.08;       % takeoff drag coefficient (placeholder for now) ------ OSCAR
mu       = 0.04;       % ground friction coefficient from lecture
s_G      = GroundRun; % -------- KAMRAN !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!2500, !!!!!!!!!!

% --- Atmosphere at runway ---
[rho, a, ~, ~] = cast.atmos(0); %#ok<ASGLU>

m_gr = EWF * M_TO;     % mass AFTER taxi
W_gr = m_gr * g;       % weight at start of ground roll

% --- Wing loading ---
W2S = W_gr / S;

% --- Required thrust-to-weight from lecture equation ---
TW_req = ...
    (1.21 / (g * rho * CLmax_TO * s_G)) * W2S + ...
    0.5 * (CD_TO / CLmax_TO) + ...
    0.5 * mu;

% --- Required thrust ---
T_req = TW_req * W_gr;

% --- Estimate liftoff speed for time/fuel ---
V_stall = sqrt(2 * W_gr / (rho * S * CLmax_TO));
V_lof   = 1.1 * V_stall; %--------- double check LIAMMMMM

% --- Estimate average acceleration from runway length ---
a_avg = V_lof^2 / (2 * s_G);

% --- Time estimate ---
t_ground = V_lof / a_avg;

% --- Fuel burn ---
TSFC = ADP.Engine.TSFC(0,0);   % low-speed approximation
FuelTO = TSFC * T_req * t_ground;

% % --- Mass fraction update ---
% fs(2) = 1 - FuelTO / m_gr;
% EWF = EWF * fs(2);


k = length(fs) + 1;

fs(k) = 1 - FuelTO / m_gr;
ts(k) = t_ground;

EWF = EWF * fs(k);

%% climb

% f_climb = 0.985;
% 
% 
% fs(3) = f_climb;
% EWF = EWF * fs(3);


% % --- Climb (single segment) --- NEW
% 
% % --- Setup ---
% m_climb = EWF * M_TO;     % mass after takeoff
% W_climb = m_climb * g;
% 
% h_climb    = Alt_cruise; % TLAR 34k for now
% V_climb_real = V_climb*0.9;            % climb speed [m/s] (~450 knots-ish)
% ROC     = 10;             % rate of climb [m/s] (~2000 ft/min)
% 
% % --- Atmosphere at mid-climb ---
% h_mid = h_climb / 2;
% [rho, a, ~, ~] = cast.atmos(h_mid);
% 
% Mach_climb = V_climb_real / a;
% 
% % --- Aerodynamics (PLACEHOLDERS for now) ---
% CL_climb = 0.5;     % typical climb CL
% CD_climb = 0.05;    % typical transport climb CD (~L/D ≈ 10)
% D_climb  = 0.5 * rho * V_climb_real^2 * S * CD_climb;
% 
% % --- Required thrust ---
% T_req_climb = D_climb + W_climb * (ROC / V_climb_real);
% 
% % --- Time ---
% t_climb = h_climb / ROC;
% 
% % --- Fuel burn ---
% TSFC = ADP.Engine.TSFC(Mach_climb, h_mid);
% FuelClimb = TSFC * T_req_climb * t_climb;
% 
% % --- Mass fraction update ---
% fs(3) = 1 - FuelClimb / m_climb;
% EWF = EWF * fs(3);
% 
% % --- Optional: store time ---
% ts(3) = t_climb;

% %% --- Climb (4 segments, simplified physics) ---
% 
% segments_ft = [ ...
%     0     1500;
%     1500  10000;
%     10000 20000;
%     20000 ADP.TLAR.Alt_cruise * SI.ft   % convert m → ft for consistency
% ];
% 
% % --- Placeholder values (keep simple for now) ---
% CL_climb = 0.5;
% CD_climb = 0.05;
% ROC      = 10;   % m/s (~2000 ft/min)
% 
% for i = 1:size(segments_ft,1)
% 
%     % --- Convert altitudes to meters ---
%     h_start = segments_ft(i,1) * 0.3048;
%     h_end   = segments_ft(i,2) * 0.3048;
% 
%     h_mid = 0.5 * (h_start + h_end);
% 
%     % --- Current mass ---
%     m_seg = EWF * M_TO;
%     W_seg = m_seg * g;
% 
%     % --- Atmosphere ---
%     [rho, a, ~, ~] = cast.atmos(h_mid);
% 
%     % --- Speed (use TLAR climb speed for now) ---
%     V = ADP.TLAR.(V_climb*0.9);%0.9 factor to not go above climb limit
%     Mach = V / a;
% 
%     % --- Aerodynamics (PLACEHOLDERS) ---
%     D = 0.5 * rho * V^2 * S * CD_climb;
% 
%     % --- Required thrust ---
%     T_req = D + W_seg * (ROC / V);
% 
%     % --- Time ---
%     dh = h_end - h_start;
%     t_seg = dh / ROC;
% 
%     % --- Fuel ---
%     TSFC = ADP.Engine.TSFC(Mach, h_mid);
%     Fuel = TSFC * T_req * t_seg;
% 
%     % --- Mass fraction ---
%     fs(2+i) = 1 - Fuel / m_seg;
%     EWF = EWF * fs(2+i);
% 
%     % --- Time tracking ---
%     ts(2+i) = t_seg;
% 
% end

% %% --- Climb (constraint-based 4 segments) ---
% 
% % Placeholder aero for now
% CD_climb = 0.05;
% 
% % Track climb end for Mf_TOC later
% idx_climb_start = length(fs) + 1;
% 
% %% Segment 1: 0 -> 1500 ft
% % Requirement: assume 1 minute at maximum power
% h1_start = 0;
% h1_end   = 1500 * 0.3048;
% t1       = 50;   % s
% 
% m_seg = EWF * M_TO;
% W_seg = m_seg * g;
% 
% h_mid = 0.5 * (h1_start + h1_end);
% [rho, a, ~, ~] = cast.atmos(h_mid);
% 
% % Use 250 kts as target climb/acceleration speed marker
% V1 = V_climb*0.9;   % m/s
% Mach1 = V1 / a;
% dh1 = h1_end-h1_start;
% ROC1 = dh1/t1;
% 
% V1x = sqrt(V1^2 - ROC1^2);
% R1 = V1x * t1;
% 
% % Approximate thrust at max power using design T/W
% T2W = 0.3;
% T_req = T2W * W_seg;
% 
% TSFC = ADP.Engine.TSFC(Mach1, h_mid);
% Fuel = TSFC * T_req * t1;
% 
% k = length(fs) + 1;
% fs(k) = 1 - Fuel / m_seg;
% EWF = EWF * fs(k);
% ts(k) = t1;
% 
% %% Segments 2 + 3: 1500 ft -> 20000 ft in <= 30 min
% % Split the 30 min requirement across the two altitude bands in proportion to dh
% 
% h2_start = 1500  * 0.3048;
% h2_end   = 10000 * 0.3048;
% 
% h3_start = 10000 * 0.3048;
% h3_end   = 20000 * 0.3048;
% 
% dh2 = h2_end - h2_start;
% dh3 = h3_end - h3_start;
% t_total_23 = 25 * 60;   % s
% 
% t2 = t_total_23 * dh2 / (dh2 + dh3);
% t3 = t_total_23 * dh3 / (dh2 + dh3);
% 
% % -------- Segment 2: 1500 -> 10000 ft @ 250 kts --------
% m_seg = EWF * M_TO;
% W_seg = m_seg * g;
% 
% h_mid = 0.5 * (h2_start + h2_end);
% [rho, a, ~, ~] = cast.atmos(h_mid);
% 
% V2 = V_climb*0.9;   % m/s
% Mach2 = V2 / a;
% ROC2 = dh2 / t2;
% 
% D2 = 0.5 * rho * V2^2 * S * CD_climb;
% T_req2 = D2 + W_seg * (ROC2 / V2);
% 
% TSFC2 = ADP.Engine.TSFC(Mach2, h_mid);
% Fuel2 = TSFC2 * T_req2 * t2;
% 
% k = length(fs) + 1;
% fs(k) = 1 - Fuel2 / m_seg;
% EWF = EWF * fs(k);
% ts(k) = t2;
% 
% % -------- Segment 3: 10000 -> 20000 ft --------
% m_seg = EWF * M_TO;
% W_seg = m_seg * g;
% 
% h_mid = 0.5 * (h3_start + h3_end);
% [rho, a, ~, ~] = cast.atmos(h_mid);
% 
% % Use not 300 kts, but do not exceed cruise speed
% V3 = M_c*a*0.9;
% 
% Mach3 = V3 / a;
% ROC3 = dh3 / t3;
% 
% D3 = 0.5 * rho * V3^2 * S * CD_climb;
% T_req3 = D3 + W_seg * (ROC3 / V3);
% 
% TSFC3 = ADP.Engine.TSFC(Mach3, h_mid);
% Fuel3 = TSFC3 * T_req3 * t3;
% 
% k = length(fs) + 1;
% fs(k) = 1 - Fuel3 / m_seg;
% EWF = EWF * fs(k);
% ts(k) = t3;
% 
% V2x = sqrt(V2^2 - ROC2^2);
% R2 = V2x * t2;
% V3x = sqrt(V3^2 - ROC3^2);
% R3 = V3x * t3;
% 
% %% Segment 4: 20000 ft -> cruise altitude
% % Keep this as a simple physics climb above initial cruise altitude
% 
% h4_start = 20000 * 0.3048;
% h4_end   = Alt_cruise;
% 
% if h4_end > h4_start
%     m_seg = EWF * M_TO;
%     W_seg = m_seg * g;
% 
%     h_mid = 0.5 * (h4_start + h4_end);
%     [rho, a, ~, ~] = cast.atmos(h_mid);
% 
%     % Use climb speed capped by cruise speed
%     V4 = M_c*a*0.9;
% 
%     Mach4 = V4 / a;
% 
%     ROC4 = ROC3*0.5;   % m/s placeholder above 20k ft
%     dh4 = h4_end - h4_start;
%     t4 = dh4 / ROC4;
% 
%     D4 = 0.5 * rho * V4^2 * S * CD_climb;
%     T_req4 = D4 + W_seg * (ROC4 / V4);
% 
%     TSFC4 = ADP.Engine.TSFC(Mach4, h_mid);
%     Fuel4 = TSFC4 * T_req4 * t4;
% 
%     k = length(fs) + 1;
%     fs(k) = 1 - Fuel4 / m_seg;
%     EWF = EWF * fs(k);
%     ts(k) = t4;
% end
% 
% V4x = sqrt(V4^2 - ROC4^2);
% R4 = V4x * t4;
% 
% idx_climb_end = length(fs);

%% --- Climb (500 ft steps WITH your 4 constraint regions) ---

CD_climb = 0.05;

idx_climb_start = length(fs) + 1;

dh = 500 * 0.3048;   % 500 ft step

R_climb = 0;








%% -----------------------------
% Segment 1: 0 -> 1500 ft (fixed time = 50 s)
% -----------------------------
h1_start = 0;
h1_end   = 1500 * 0.3048;

h_nodes = h1_start:dh:h1_end;
if h_nodes(end) ~= h1_end
    h_nodes = [h_nodes h1_end];
end

t_total = 50;   % your constraint
t_step = t_total / (length(h_nodes)-1);


for i = 1:(length(h_nodes)-1)

    h_mid = 0.5*(h_nodes(i)+h_nodes(i+1));

    m_seg = EWF * M_TO;
    W_seg = m_seg * g;

    [rho,a,~,~] = cast.atmos(h_mid);

    V = V_climb;
    Mach = V/a;

    ROC1 = (h_nodes(i+1)-h_nodes(i)) / t_step;

    Vx = sqrt(V^2 - ROC1^2);
    R_climb = R_climb + Vx * t_step;

        % --- Aerodynamics ---
    CD = CD_climb;
    D  = 0.5 * rho * V^2 * S * CD;
    
    % --- Required thrust from climb physics ---
    T_raw = D + W_seg * (ROC1 / V);
    

    % --- Idle thrust ---
    
    T_available = N_eng*ADP.Engine.T_Static;

    T_idle = IdleFrac * T_available;
    T_max  = T_available;

    % --- Final thrust (only enforce minimum, NOT max) ---
    T_req = max(T_idle, T_raw);
    
    % --- Thrust limit check (NO enforcement) ---

    if T_req > T_max
        warning('Segment1 thrust exceeded at %.0f m: ratio = %.2f', ...
            h_mid, T_req/T_max);
    end

    TSFC = ADP.Engine.TSFC(Mach,h_mid);
    Fuel = TSFC * T_req * t_step;

    k = length(fs)+1;
    fs(k) = 1 - Fuel/m_seg;
    ts(k) = t_step;

    EWF = EWF * fs(k);
end

%% -----------------------------
% Segment 2 + 3 total constraint (25 min)
%% -----------------------------
h2_start = 1500  * 0.3048;
h2_end   = 10000 * 0.3048;

h3_start = 10000 * 0.3048;
h3_end   = 20000 * 0.3048;

dh2 = h2_end - h2_start;
dh3 = h3_end - h3_start;

t_total_23 = 25*60;

t2_total = t_total_23 * dh2/(dh2+dh3);
t3_total = t_total_23 * dh3/(dh2+dh3);

%% -------- Segment 2 (1500–10k ft) --------
h_nodes = h2_start:dh:h2_end;
if h_nodes(end) ~= h2_end
    h_nodes = [h_nodes h2_end];
end

t_step = t2_total/(length(h_nodes)-1);

for i = 1:(length(h_nodes)-1)

    h_mid = 0.5*(h_nodes(i)+h_nodes(i+1));

    m_seg = EWF * M_TO;
    W_seg = m_seg * g;

    [rho,a,~,~] = cast.atmos(h_mid);

    % below 10k ft: keep prescribed climb speed
    V = V_climb;
    Mach = V/a;

    % keep time constraint FIXED
    dh_step = h_nodes(i+1) - h_nodes(i);
    ROC2 = dh_step / t_step;

    % --- Aerodynamics ---
    CD = CD_climb;
    D  = 0.5 * rho * V^2 * S * CD;

    % --- Required thrust from prescribed climb ---
    T_raw  = D + W_seg * (ROC2 / V);


    T_available = N_eng*ADP.Engine.T_Static;

    T_idle = IdleFrac * T_available;
    T_max  = ClimbFrac * T_available;

    T_req  = max(T_idle, T_raw);

    % --- Feasibility check only (NO enforcement) ---
    if T_req > T_max
        warning('Segment 2 thrust exceeded at %.0f m: T_req/T_max = %.2f', ...
            h_mid, T_req / T_max);
    end

    % --- Fuel ---
    TSFC = ADP.Engine.TSFC(Mach, h_mid);
    Fuel = TSFC * T_req * t_step;

    k = length(fs)+1;
    fs(k) = 1 - Fuel/m_seg;
    ts(k) = t_step;

    EWF = EWF * fs(k);

    Vx = sqrt(V^2 - ROC2^2);
    R_climb = R_climb + Vx * t_step;
end

%% -------- Segment 3 (10k–20k ft) --------
h_nodes = h3_start:dh:h3_end;
if h_nodes(end) ~= h3_end
    h_nodes = [h_nodes h3_end];
end

t_step = t3_total/(length(h_nodes)-1);

for i = 1:(length(h_nodes)-1)

    h_mid = 0.5*(h_nodes(i)+h_nodes(i+1));

    m_seg = EWF * M_TO;
    W_seg = m_seg * g;

    [rho,a,~,~] = cast.atmos(h_mid);

    % 10k–20k ft: prescribed higher-speed climb
    V = M_climb * a;
    Mach = V/a;

    % keep time constraint FIXED
    dh_step = h_nodes(i+1) - h_nodes(i);
    ROC3 = dh_step / t_step;

    % --- Aerodynamics ---
    CD = CD_climb;
    D  = 0.5 * rho * V^2 * S * CD;

    % --- Required thrust from prescribed climb ---
    T_raw  = D + W_seg * (ROC3 / V);
    
    T_available = N_eng*ADP.Engine.T_Static;

    T_idle = IdleFrac * T_available;
    T_max  = ClimbFrac * T_available;

    T_req  = max(T_idle, T_raw);

    % --- Feasibility check only (NO enforcement) ---
    if T_req > T_max
        warning('Segment 3 thrust exceeded at %.0f m: T_req/T_max = %.2f', ...
            h_mid, T_req / T_max);
    end

    % --- Fuel ---
    TSFC = ADP.Engine.TSFC(Mach, h_mid);
    Fuel = TSFC * T_req * t_step;

    k = length(fs)+1;
    fs(k) = 1 - Fuel/m_seg;
    ts(k) = t_step;

    EWF = EWF * fs(k);

    Vx = sqrt(V^2 - ROC3^2);
    R_climb = R_climb + Vx * t_step;
end

%% -------- Segment 4 (20k–cruise) --------
h4_start = 20000*0.3048;
h4_end   = Alt_cruise;

if h4_end > h4_start

    h_nodes = h4_start:dh:h4_end;
    if h_nodes(end) ~= h4_end
        h_nodes = [h_nodes h4_end];
    end

    ROC4 = ROC3 *1;

    for i = 1:(length(h_nodes)-1)

        h_mid = 0.5*(h_nodes(i)+h_nodes(i+1));

        m_seg = EWF * M_TO;
        W_seg = m_seg * g;

        [rho,a,~,~] = cast.atmos(h_mid);

        V = M_climb * a;
        Mach = V/a;

        dh_step = h_nodes(i+1) - h_nodes(i);
        t_step = dh_step / ROC4;

        % --- Aerodynamics ---
        CD = CD_climb;
        D  = 0.5 * rho * V^2 * S * CD;

        % --- Required thrust ---
        T_raw  = D + W_seg * (ROC4 / V);

        T_available = N_eng*ADP.Engine.T_Static;

        T_idle = IdleFrac * T_available;
        T_max  = ClimbFrac * T_available;

        T_req  = max(T_idle, T_raw);

        % --- Feasibility check ---
        if T_req > T_max
            warning('Segment 4 thrust exceeded at %.0f m: T_req/T_max = %.2f', ...
                h_mid, T_req / T_max);
        end

        % --- Fuel ---
        TSFC = ADP.Engine.TSFC(Mach, h_mid);
        Fuel = TSFC * T_req * t_step;

        k = length(fs)+1;
        fs(k) = 1 - Fuel/m_seg;
        ts(k) = t_step;

        EWF = EWF * fs(k);

        Vx = sqrt(V^2 - ROC4^2);
        R_climb = R_climb + Vx * t_step;
    end
end

idx_climb_end = length(fs);


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

%% THIS IS CODE IVE ADDED - OSCAR

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

% THIS IS CODE IVE ADDED - OSCAR

% cruise analysis (assume constant C_L)

% pick optimal altitude for cruise
alts = linspace(8000,11000,12); % OSCAR: Level cruise is in my mind could be anywhere from 8000m to 11000m
[rho,a,T,P] = cast.atmos(alts);
% [rho_s,a_s,~,P_s] = dcrg.aero.atmos(0);
M_cruise = ADP.TLAR.M_c;
% THIS IS CODE IVE ADDED - OSCAR
CL_c = interp1(A,CL,alpha,'linear','extrap');
CD_c = interp1(A,CD,alpha,'linear','extrap');
LD_c = CL_c./CD_c;
%% THIS IS CODE IVE ADDED - OSCAR
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
% fs(7) = exp(-tripRange*9.81*ADP.Engine.TSFC(M_cruise,alt)/(M_cruise*a*LD_c)); % Rearranged Brequet
% ts(7) = tripRange/(M_cruise*a); % time taken
% EWF = EWF*fs(7);

k = length(fs) + 1;

fs(k) = exp(-tripRange*9.81*ADP.Engine.TSFC(M_cruise,alt)/(M_cruise*a*LD_c)); % Rearranged Brequet
ts(k) = tripRange/(M_cruise*a); % time taken

EWF = EWF * fs(k);

%% --- Trip: descent ---

f_desc = 0.995;

% fs(8) = f_desc;
% EWF = EWF * fs(8);

k = length(fs) + 1;

fs(k) = f_desc;
%ts(k) = ;

EWF = EWF * fs(k);



%% approach

f_app  = 0.995;

%fs(9) = f_app;
%EWF = EWF * fs(9);

k = length(fs) + 1;

fs(k) = f_app;
%ts(k) = ;

EWF = EWF * fs(k);

idx_trip_end      = length(fs);   % end of main design mission


%% Contingency
df = (1-EWF)*0.03;
% fs(10) = 1-df/EWF;
% ts(10) = 5*60; % 5 minutes...
% EWF = EWF*fs(10);

k = length(fs) + 1;

fs(k) = 1-df/EWF;
ts(k) =  5*60; %.......... 5 mins?????

EWF = EWF * fs(k);

%% Reserve climb
% fs(11) = 0.985;
% EWF = EWF * fs(11);

k = length(fs) + 1;

fs(k) = 0.985;
%ts(k) = ;

EWF = EWF * fs(k);

%% reserve alternate mission analysis
[rho,a,~,P] = cast.atmos(ADP.TLAR.Alt_alternate);
% [rho_s,a_s,~,P_s] = dcrg.aero.atmos(0);
M_cruise = ADP.TLAR.M_c;

CL_c = EWF*M_TO*9.81/(1/2*rho*(a*M_cruise)^2*ADP.WingArea); % cruise C_L
CD_c = ADP.AeroPolar.CD(CL_c);
LD_c = CL_c/CD_c;

% account for fact I don't model climb with an "effective" trip range
altRange = ADP.TLAR.Range_alternate * 1;

%fs(12) = exp(-altRange*9.81*ADP.Engine.TSFC(M_cruise,ADP.TLAR.Alt_alternate)/(M_cruise*a*LD_c)); % Rearranged Brequet
%ts(12) = altRange/(M_cruise*a); % time taken
%EWF = EWF*fs(12);

k = length(fs) + 1;

fs(k) = exp(-altRange*9.81*ADP.Engine.TSFC(M_cruise,ADP.TLAR.Alt_alternate)/(M_cruise*a*LD_c)); % Rearranged Brequet
ts(k) = altRange/(M_cruise*a); % time taken

EWF = EWF * fs(k);

%% Reserve descent
%fs(13) = 0.995;
%EWF = EWF * fs(13);

k = length(fs) + 1;

fs(k) = 0.995;
%ts(k) = t_ground;

EWF = EWF * fs(k);

%% loiter
[rho,a,~,P] = cast.atmos(0);
Mach = 150/a;
CL = EWF*M_TO*9.81/(1/2*rho*(a*Mach)^2*ADP.WingArea);
CD = ADP.AeroPolar.CD(CL);
LD = CL/CD;

%fs(14) = exp(-ADP.TLAR.Loiter*9.81*ADP.Engine.TSFC(Mach,0)/LD); % Snorri
%ts(14) = ADP.TLAR.Loiter; % time taken
%EWF = EWF*fs(14);

k = length(fs) + 1;

fs(k) = exp(-ADP.TLAR.Loiter*9.81*ADP.Engine.TSFC(Mach,0)/LD); % Snorri
ts(k) = ADP.TLAR.Loiter; % time taken

EWF = EWF * fs(k);

%% Reserve approach
%fs(15) = 0.995;
%EWF = EWF * fs(15);

k = length(fs) + 1;

fs(k) = 0.995;
%ts(k) = ;

EWF = EWF * fs(k);

idx_alt_end       = length(fs);   % end of alternate mission


%% Taxi to gate
fs(16) = 0.997;
%EWF = EWF * fs(16);

k = length(fs) + 1;

fs(k) = 0.997;
%ts(k) = ;

EWF = EWF * fs(k);


idx_block_end     = length(fs);   % end of whole block mission

%% find fuel blocks

m_end_trip  = M_TO * prod(fs(1:idx_trip_end));
m_end_alt   = M_TO * prod(fs(1:idx_alt_end));
m_end_block = M_TO * prod(fs(1:idx_block_end));

TripFuel      = (M_TO - m_end_trip)+(m_end_alt-m_end_block);
ResFuel = m_end_trip - m_end_alt;
BlockFuel     = TripFuel + ResFuel;



% %% update model
% 
% BlockFuel = (1 - EWF) * M_TO;
% 
% % Trip fuel = taxi → climb → cruise → descent → approach → taxi to gate
% TripFuel = (1 - prod(fs([1:9 16]))) * M_TO;
% 
% % Reserve fuel = everything else
% ResFuel = BlockFuel - TripFuel;

Mf_TOC = prod(fs(1:3));

MissionTime = ts(4) + ts(7) + ts(9) + ts(11);
end