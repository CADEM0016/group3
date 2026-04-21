%% ==========================================
% CLIMB MODEL — 500 ft STEPPED, 4 SEGMENTS
% ==========================================

%clear all
%scripts.ExampleUnconventional

%% ---------------------- Mission range / cruise setup ----------------------
tripRange = ADP.TLAR.RangeDes;
%tripRange = 320000;
%tripRange = 960000; % worst case scneario


Range_km = tripRange / 1000;

if Range_km < 1500
    h4 = ADP.TLAR.Alt_alternate;   % short mission → lower altitude
else
    h4 = ADP.TLAR.Alt_cruise;      % everything else → normal cruise
end


Alt_max   = ADP.TLAR.Alt_max;
ft2m = 0.3048;


%h4 = ADP.TLAR.Alt_cruise;      % top of climb = cruise altitude
%dh_cruise_block = 1000*ft2m;   % for stepped cruise above TOC

%% ---------------------- Pull from ADP ----------------------
m01 = ADP.MTOM;
S   = ADP.WingArea;

knots2m_s = 0.5144;
g = 9.81;

dh = 500*ft2m;

aimFractClimb = 0.985; %#ok<NASGU>



%% ------------------ Low fidelity assumptions ----------------
%Cd = 0.03;

%% ---------------------- T_achievable ------------------------
h0 = 0;

[rho0,~,~,~] = cast.atmos(h0);

Cl_TO = 0.8;
Cl_max = 1.5; %#ok<NASGU> % not used for now
Cd_TO = 0.03;
L2D_TO = Cl_TO/Cd_TO;
mu = 0.04;

s_g = ADP.TLAR.GroundRun;

M_TO = m01; %#ok<NASGU>

T2W_crit = ((1.21/(g*rho0*Cl_TO*s_g))*(m01*g/S)) + ...
           (0.5*(1/L2D_TO)) + (0.5*mu);

T_TO = T2W_crit*m01*g;

%% ----------- THRUST MODEL SETUP (MATTINGLY, HIGH-BPR) -----------
F_SL = T_TO;   % total sea-level static thrust reference [N]

%% ==========================================
% TAXI + TAKEOFF MODEL (GROUND PHASE)
% ==========================================

IdleFrac = 0.1;

%% ---------------- TAXI ----------------
TaxiTime = 20*60;

T_idle = IdleFrac * T_TO;   % reuse your thrust scaling

TSFC_idle = ADP.Engine.TSFC(0,0);

FuelTaxi = TSFC_idle * T_idle * TaxiTime;

m_after_taxi = m01 - FuelTaxi;

%% ---------------- TAKEOFF ROLL ----------------

% reuse YOUR already computed thrust
W = m_after_taxi * g;

% liftoff speed (reuse your CL_TO etc.)
V_stall = sqrt(2 * W / (rho0 * S * Cl_TO));
V_lof   = 1.1 * V_stall;

% reuse YOUR runway length
a_avg = V_lof^2 / (2 * s_g);
t_TO  = V_lof / a_avg;

% fuel (reuse same thrust model)
TSFC_TO = ADP.Engine.TSFC(0,0);

FuelTO = TSFC_TO * T_TO * t_TO;

%% ---------------- MASS AT START OF CLIMB ----------------
m01 = m_after_taxi - FuelTO;

Mfn_taxi_TO   = m01 / ADP.MTOM;


%% ---------------- QUICK OUTPUT ----------------
fprintf('Taxi Fuel: %.0f kg\n', FuelTaxi)
fprintf('Takeoff Fuel: %.0f kg\n', FuelTO)
fprintf('Mass at liftoff (m01): %.0f kg\n', m01)

%V0 = V_lof; %double check if needed

%% ---------------------- Global stores -----------------------
h_vec    = [];
m_vec    = [];
V_vec    = [];
Mach_vec = [];
TSFC_vec = [];
Treq_vec = [];    
Tav_vec  = [];   % <<< ADD THIS LINE
t_vec = [];
R_vec = [];

% --- FIX 1: anchor start of climb (end of takeoff) ---
h_vec(end+1) = 0;
t_vec(end+1) = 0;
R_vec(end+1) = 0;

FuelClimb = 0;
TimeClimb = 0;
RangeClimb = 0;
Climb_t_123_actual = 0;





%% ============================================================
%% 0 --> 1500 ft
%% ============================================================

% --- FIX 2: force exact start of segment ---
h_vec(end+1) = h0;
t_vec(end+1) = t_vec(end);
R_vec(end+1) = R_vec(end);

h0 = 0;
h1 = 1500*ft2m;

dh01 = h1 - h0;
Climb_t01  = 50;                       % segment total time [s]
v01_cmd = 0.9*250*knots2m_s;     % commanded speed for this segment

h_nodes = h0:dh:h1;
if h_nodes(end) ~= h1
    h_nodes = [h_nodes h1];
end

Climb_DeltaM_01 = 0;
Climb_R01 = 0;

for i = 1:(length(h_nodes)-1)
    h_low  = h_nodes(i);
    h_high = h_nodes(i+1);
    h01    = 0.5*(h_low + h_high);

    dh_step = h_high - h_low;
    dt_step = Climb_t01 * (dh_step/dh01);

    [rho01,a01,~,~] = cast.atmos(h01);

    v01  = v01_cmd;
    M_01 = v01/a01;
    vy01 = dh_step/dt_step;

    if vy01 >= v01
        error('Segment 01 invalid: vy >= V at h = %.1f m', h01);
    end

    vx01 = sqrt(v01^2 - vy01^2);
    Climb_R01  = Climb_R01 + vx01*dt_step;


    m_curr = m01 - Climb_DeltaM_01;

    W = m_curr * g;
    
    CL = W / (0.5 * rho01 * v01^2 * S);
    
    CD0 = 0.02;
    k   = 0.045;
    
    CD = CD0 + k * CL^2;
    
    D01 = 0.5 * rho01 * v01^2 * S * CD;

    T01_req = D01 + (m_curr*g)*(vy01/v01);
    T01_av  = thrustMattinglyHighBPR(F_SL, h01, M_01);
    
    TSFC_01 = ADP.Engine.TSFC(M_01, h01);
    dM_step = TSFC_01 * T01_req * dt_step;

    Climb_DeltaM_01 = Climb_DeltaM_01 + dM_step;

    % stores
    % cumulative time + range
    if isempty(t_vec)
        t_cum = dt_step;
        R_cum = vx01*dt_step;
    else
        t_cum = t_vec(end) + dt_step;
        R_cum = R_vec(end) + vx01*dt_step;
    end
    
    t_vec(end+1) = t_cum;
    R_vec(end+1) = R_cum;
    
    % existing stores
    h_vec(end+1)    = h01;
    m_vec(end+1)    = m_curr - dM_step;
    V_vec(end+1)    = v01;
    Mach_vec(end+1) = M_01;
    TSFC_vec(end+1) = TSFC_01;
    Treq_vec(end+1) = T01_req;
    Tav_vec(end+1)  = T01_av;
end

m12 = m01 - Climb_DeltaM_01;

% --- FIX 3: force segment boundary at 1500 ft ---
h_vec(end+1) = h1;
t_vec(end+1) = t_vec(end);
R_vec(end+1) = R_vec(end);

DeltaT_01 = T01_av - T01_req;

FuelClimb  = FuelClimb + Climb_DeltaM_01;
TimeClimb  = TimeClimb + Climb_t01;
RangeClimb = RangeClimb + Climb_R01;

h_rep = (h0 + h1)/2;

fprintf('\n[0–1500 ft]\n');
fprintf('h ≈ %.0f ft | V = %.1f m/s | Vy ≈ %.1f m/s | Mach ≈ %.3f\n', ...
    h_rep/ft2m, v01, vy01, M_01);

fprintf('Treq = %.0f N | Tav = %.0f N | Margin = %.0f N\n', ...
    T01_req, T01_av, T01_av - T01_req);

%% ============================================================
%% 1500 --> 10000 ft
%% ============================================================
h1 = 1500*ft2m;
h2 = 10000*ft2m;

dh12 = h2 - h1;

%% time setup shared with next segment
h2_tmp = 10000*ft2m;
h3_tmp = 20000*ft2m;
dh23_tmp = h3_tmp - h2_tmp;

dh_total = (20000 - 1500)*ft2m;
t_target = 30*60;

ROC_base = dh_total / t_target;

ROC_factor = 3.2;   % <-- THIS is your control
%ROC_factor = 3.5;   % <-- THIS is your control
%ROC_factor = 2.5;   % <-- THIS is your control


ROC_used = ROC_base * ROC_factor;


%Climb_t123 = 25*60; % total time 1500 --> 20000
%Climb_t12 = Climb_t123*(dh12/(dh12 + dh23_tmp));
%t12_cmd = Climb_t12; %#ok<NASGU>

v12_cmd = 0.95*250*knots2m_s;

h_nodes = h1:dh:h2;
if h_nodes(end) ~= h2
    h_nodes = [h_nodes h2];
end

Climb_DeltaM_12 = 0;
Climb_R12 = 0;

for i = 1:(length(h_nodes)-1)
    h_low  = h_nodes(i);
    h_high = h_nodes(i+1);
    h12    = 0.5*(h_low + h_high);

    dh_step = h_high - h_low;
%    dt_step = Climb_t12 * (dh_step/dh12);

    [rho12,a12,~,~] = cast.atmos(h12);

    v12  = v12_cmd;
    M_12 = v12/a12;
%    vy12 = dh_step/dt_step;

    vy12 = ROC_used;
    dt_step = dh_step / vy12;

    if vy12 >= v12
        error('Segment 12 invalid: vy >= V at h = %.1f m', h12);
    end

    vx12 = sqrt(v12^2 - vy12^2);
    Climb_R12  = Climb_R12 + vx12*dt_step;

    m_curr = m12 - Climb_DeltaM_12;


    W = m_curr * g;
    
    CL = W / (0.5 * rho12 * v12^2 * S);
    
    CD0 = 0.02;
    k   = 0.045;
    
    CD = CD0 + k * CL^2;
    
    D12 = 0.5 * rho12 * v12^2 * S * CD;

    T12_req = D12 + (m_curr*g)*(vy12/v12);
    T12_av  = thrustMattinglyHighBPR(F_SL, h12, M_12);

    TSFC_12 = ADP.Engine.TSFC(M_12, h12);
    dM_step = TSFC_12 * T12_req * dt_step;

    Climb_DeltaM_12 = Climb_DeltaM_12 + dM_step;

    % stores
    if isempty(t_vec)
        t_cum = dt_step;
        R_cum = vx12*dt_step;
    else
        t_cum = t_vec(end) + dt_step;
        R_cum = R_vec(end) + vx12*dt_step;
    end
    
    t_vec(end+1) = t_cum;
    R_vec(end+1) = R_cum;
    
    h_vec(end+1)    = h12;
    m_vec(end+1)    = m_curr - dM_step;
    V_vec(end+1)    = v12;
    Mach_vec(end+1) = M_12;
    TSFC_vec(end+1) = TSFC_12;
    Treq_vec(end+1) = T12_req;
    Tav_vec(end+1)  = T12_av;

    Climb_t_123_actual = Climb_t_123_actual + dt_step;
end

m23 = m12 - Climb_DeltaM_12;
DeltaT_12 = T12_av - T12_req;

FuelClimb  = FuelClimb + Climb_DeltaM_12;
TimeClimb = TimeClimb + dt_step;
RangeClimb = RangeClimb + Climb_R12;

h_rep = (h1 + h2)/2;

fprintf('\n[1500–10000 ft]\n');
fprintf('h ≈ %.0f ft | V = %.1f m/s | Vy ≈ %.1f m/s | Mach ≈ %.3f\n', ...
    h_rep/ft2m, v12, vy12, M_12);

fprintf('Treq = %.0f N | Tav = %.0f N | Margin = %.0f N\n', ...
    T12_req, T12_av, T12_av - T12_req);

%% ============================================================
%% 10000 --> 20000 ft
%% ============================================================
h2 = 10000*ft2m;
h3 = 20000*ft2m;

dh23 = h3 - h2;
%Climb_t23 = Climb_t123*(dh23/(dh12 + dh23));

%v23_cmd = 250*knots2m_s;
% If you want 0.9*250 here as well, change previous line to:
%v23_cmd = 0.9*250*knots2m_s;
h23_rep = 0.5*(h2 + h3);   % midpoint altitude
[~,a23_rep,~,~] = cast.atmos(h23_rep);

M_23_cmd = ADP.TLAR.M_c * 0.8;
v23_cmd  = M_23_cmd*a23_rep;

%v23_cmd = 0.9*250*knots2m_s;


h_nodes = h2:dh:h3;
if h_nodes(end) ~= h3
    h_nodes = [h_nodes h3];
end

Climb_DeltaM_23 = 0;
Climb_R23 = 0;

for i = 1:(length(h_nodes)-1)
    h_low  = h_nodes(i);
    h_high = h_nodes(i+1);
    h23    = 0.5*(h_low + h_high);

    dh_step = h_high - h_low;
    vy23 = ROC_used;
    dt_step = dh_step / vy23;
    [rho23,a23,~,~] = cast.atmos(h23);

    %v23  = v23_cmd;
    M_23 = M_23_cmd;
    v23  = M_23 * a23;

    if vy23 >= v23
        error('Segment 23 invalid: vy >= V at h = %.1f m', h23);
    end

    vx23 = sqrt(v23^2 - vy23^2);
    Climb_R23  = Climb_R23 + vx23*dt_step;

    m_curr = m23 - Climb_DeltaM_23;


    W = m_curr * g;
    
    CL = W / (0.5 * rho23 * v23^2 * S);
    
    CD0 = 0.02;
    k   = 0.045;
    
    CD = CD0 + k * CL^2;
    
    D23 = 0.5 * rho23 * v23^2 * S * CD;

    T23_req = D23 + (m_curr*g)*(vy23/v23);
    T23_av  = thrustMattinglyHighBPR(F_SL, h23, M_23);

    TSFC_23 = ADP.Engine.TSFC(M_23, h23);
    dM_step = TSFC_23 * T23_req * dt_step;

    Climb_DeltaM_23 = Climb_DeltaM_23 + dM_step;

    % stores
    if isempty(t_vec)
        t_cum = dt_step;
        R_cum = vx23*dt_step;
    else
        t_cum = t_vec(end) + dt_step;
        R_cum = R_vec(end) + vx23*dt_step;
    end
    
    t_vec(end+1) = t_cum;
    R_vec(end+1) = R_cum;
    
    h_vec(end+1)    = h23;
    m_vec(end+1)    = m_curr - dM_step;
    V_vec(end+1)    = v23;
    Mach_vec(end+1) = M_23;
    TSFC_vec(end+1) = TSFC_23;
    Treq_vec(end+1) = T23_req;
    Tav_vec(end+1)  = T23_av;

    Climb_t_123_actual = Climb_t_123_actual + dt_step;
end

m34 = m23 - Climb_DeltaM_23;
DeltaT_23 = T23_av - T23_req;

FuelClimb  = FuelClimb + Climb_DeltaM_23;
TimeClimb = TimeClimb + dt_step;
RangeClimb = RangeClimb + Climb_R23;


if Climb_t_123_actual > 30*60
    warning('Climb 1500–20000 ft exceeds 30 min (%.1f min)', Climb_t_123_actual/60)
end

h_rep = (h2 + h3)/2;

fprintf('\n[10000–20000 ft]\n');
fprintf('h ≈ %.0f ft | V = %.1f m/s | Vy ≈ %.1f m/s | Mach ≈ %.3f\n', ...
    h_rep/ft2m, v23, vy23, M_23);

fprintf('Treq = %.0f N | Tav = %.0f N | Margin = %.0f N\n', ...
    T23_req, T23_av, T23_av - T23_req);

%% ============================================================
%% 20000 ft --> cruise
%% ============================================================
h3 = 20000*ft2m;
%h4 = ADP.TLAR.Alt_alternate;
%h4 = ADP.TLAR.Alt_cruise;      % top of climb = cruise altitude
%h4 = 20500*ft2m;   % top of climb = cruise altitude



dh34 = h4 - h3;

%M_34_cmd = 0.84;
% If you want to tie this to TLAR instead:
M_34_cmd = ADP.TLAR.M_c * 0.95;

% carry forward previous segment ROC
%vy34_cmd = (dh23/Climb_t23);

%Climb_t34 = dh34/vy34_cmd;

ROC_factor_34 = 1;
ROC_used_34 = ROC_base * ROC_factor_34;
vy34_cmd = ROC_used_34;

%vy34_cmd = ROC_used;

h_nodes = h3:dh:h4;
if h_nodes(end) ~= h4
    h_nodes = [h_nodes h4];
end

Climb_DeltaM_34 = 0;
Climb_R34 = 0;

for i = 1:(length(h_nodes)-1)
    h_low  = h_nodes(i);
    h_high = h_nodes(i+1);
    h34    = 0.5*(h_low + h_high);

    dh_step = h_high - h_low;
    dt_step = dh_step/vy34_cmd;

    [rho34,a34,~,~] = cast.atmos(h34);

    M_34 = M_34_cmd;
    v34  = M_34*a34;
    %vy34 = dh_step/dt_step;
    vy34 = vy34_cmd;

    if vy34 >= v34
        error('Segment 34 invalid: vy >= V at h = %.1f m', h34);
    end

    vx34 = sqrt(v34^2 - vy34^2);
    Climb_R34  = Climb_R34 + vx34*dt_step;

    m_curr = m34 - Climb_DeltaM_34;


    W = m_curr * g;
    
    CL = W / (0.5 * rho34 * v34^2 * S);
    
    CD0 = 0.02;
    k   = 0.045;
    
    CD = CD0 + k * CL^2;
    
    D34 = 0.5 * rho34 * v34^2 * S * CD;

    T34_req = D34 + (m_curr*g)*(vy34/v34);
    T34_av  = thrustMattinglyHighBPR(F_SL, h34, M_34);

    TSFC_34 = ADP.Engine.TSFC(M_34, h34);
    dM_step = TSFC_34 * T34_req * dt_step;

    Climb_DeltaM_34 = Climb_DeltaM_34 + dM_step;

    % stores
    if isempty(t_vec)
        t_cum = dt_step;
        R_cum = vx34*dt_step;
    else
        t_cum = t_vec(end) + dt_step;
        R_cum = R_vec(end) + vx34*dt_step;
    end
    
    t_vec(end+1) = t_cum;
    R_vec(end+1) = R_cum;
    
    h_vec(end+1)    = h34;
    m_vec(end+1)    = m_curr - dM_step;
    V_vec(end+1)    = v34;
    Mach_vec(end+1) = M_34;
    TSFC_vec(end+1) = TSFC_34;
    Treq_vec(end+1) = T34_req;
    Tav_vec(end+1)  = T34_av;
end

m4c = m34 - Climb_DeltaM_34;
DeltaT_34 = T34_av - T34_req;

FuelClimb  = FuelClimb + Climb_DeltaM_34;
%TimeClimb  = TimeClimb + Climb_t34;
TimeClimb = TimeClimb + dt_step;
RangeClimb = RangeClimb + Climb_R34;


% --- FORCE EXACT TOP OF CLIMB POINT ---
h_vec(end+1) = h4;
m_vec(end+1) = m4c;
V_vec(end+1) = V_vec(end);
Mach_vec(end+1) = Mach_vec(end);
TSFC_vec(end+1) = TSFC_vec(end);
Treq_vec(end+1) = Treq_vec(end);
Tav_vec(end+1)  = Tav_vec(end);

t_vec(end+1) = t_vec(end);
R_vec(end+1) = R_vec(end);

h_rep = (h3 + h4)/2;

fprintf('\n[20000–Cruise]\n');
fprintf('h ≈ %.0f ft | V = %.1f m/s | Vy ≈ %.1f m/s | Mach ≈ %.3f\n', ...
    h_rep/ft2m, v34, vy34, M_34);

fprintf('Treq = %.0f N | Tav = %.0f N | Margin = %.0f N\n', ...
    T34_req, T34_av, T34_av - T34_req);

%% -------------------------- Outputs -------------------------
Climb_m_TOTAL = Climb_DeltaM_01 + Climb_DeltaM_12 + Climb_DeltaM_23 + Climb_DeltaM_34;

Mfn_climb = m4c/m01;

%Climb_t_TOTAL = Climb_t01 + Climb_t12 + Climb_t23 + Climb_t34;
Climb_t_TOTAL = t_vec(end);
Climb_R_TOTAL = Climb_R01 + Climb_R12 + Climb_R23 + Climb_R34;

% feasibility checks
if DeltaT_01 < 0, warning('0-1500 ft infeasible'); end
if DeltaT_12 < 0, warning('1500-10000 ft infeasible'); end
if DeltaT_23 < 0, warning('10000-20000 ft infeasible'); end
if DeltaT_34 < 0, warning('20000-cruise infeasible'); end

fprintf('Climb Fuel: %.0f kg\n', Climb_m_TOTAL)
fprintf('Climb Fuel Fraction: %.4f\n', Mfn_climb)
fprintf('Climb Time: %.1f min\n', Climb_t_TOTAL/60)
fprintf('Climb Range: %.0f km\n', Climb_R_TOTAL/1000)



fprintf('Wf_taxi_TO:   %.4f\n', Mfn_taxi_TO)
fprintf('Wf_climb:     %.4f\n', Mfn_climb)

h_ceiling = findOperationalCeiling(ADP, m4c, S, F_SL);
fprintf('Operational ceiling (ROC = 300 ft/min): %.0f ft\n', h_ceiling/ft2m)

function h_ceiling = findOperationalCeiling(ADP, m, S, F_SL)

    ft2m = 0.3048;
    g = 9.81;

    ROC_limit = 300 * ft2m / 60;   % 300 ft/min in m/s

    h_test = 20000*ft2m : 500*ft2m : 50000*ft2m;

    M = ADP.TLAR.M_c;

    h_ceiling = NaN;

    for i = 1:length(h_test)

        h = h_test(i);

        [rho,a,~,~] = cast.atmos(h);

        V = M * a;
        W = m * g;

        CL = W / (0.5 * rho * V^2 * S);

        CD0 = 0.02;
        k   = 0.045;

        CD = CD0 + k * CL^2;

        D = 0.5 * rho * V^2 * S * CD;

        T_av = thrustMattinglyHighBPR(F_SL, h, M);

        ROC = ((T_av - D) * V) / W;

        if ROC <= ROC_limit
            h_ceiling = h;
            break
        end

    end
end




function F = thrustMattinglyHighBPR(F_SL, h, M)

    gamma = 1.4;
    R = 287.05;

    T_std = 288.15;     % K
    p_std = 101325;     % Pa

    [rho,a,~,~] = cast.atmos(h);

    T = a^2 / (gamma * R);
    p = rho * R * T;

    theta0 = (T / T_std) * (1 + ((gamma - 1)/2) * M^2);
    delta0 = (p / p_std) * (1 + ((gamma - 1)/2) * M^2)^(gamma/(gamma - 1));

    %#ok<NASGU> theta0   % retained for clarity / future branch extension

    F = F_SL * delta0 * (1 - 0.49 * sqrt(max(M,0)));

    F = max(F,0);
end




%% ==========================================
%% CRUISE MODEL (1000 FT BLOCK-CLIMB FROM TOC)
%% ==========================================

% --- anchor cruise EXACTLY at TOC ---


Cruise_t_vec = 0;
Cruise_R_vec = 0;

Descent_R = 0;

m_cruise0 = m4c;
h_cruise_start = h4;          % TOC = Alt_cruise
h_cruise = h_cruise_start;
M_cruise = ADP.TLAR.M_c;

if h_cruise_start < 20000*ft2m
    error('Cruise altitude must be at least 20000 ft')
end

% exact 1000 ft blocks only
dh_cruise_block = 1000*ft2m;
h_cruise_upper = floor(min(h_ceiling, Alt_max)/dh_cruise_block) * dh_cruise_block;
h_cruise_upper = max(h_cruise_upper, h_cruise_start);

if h_cruise_start > h_ceiling
    warning('Chosen cruise start altitude is above operational ceiling')
end

Cruise_R_target = tripRange - 2*Climb_R_TOTAL;

R_min_cruise = 50e3;   % [m] minimum cruise distance

if Cruise_R_target < 0
    error('❌ CRITICAL: Climb range (%.0f km) exceeds total mission range (%.0f km). Model is invalid.', ...
        Climb_R_TOTAL/1000, tripRange/1000);
end

% if Cruise_R_target < R_min_cruise
%     warning('⚠️ Short mission: computed cruise = %.0f km < minimum %.0f km. Forcing minimum cruise.', ...
%         Cruise_R_target/1000, R_min_cruise/1000);
% 
%     Cruise_R_target = R_min_cruise;
% end

N_cruise_segments = 6;   % you choose this (20–100 is sensible)
dR_cruise = Cruise_R_target / N_cruise_segments;

Cruise_DeltaM = 0;
Cruise_t_TOTAL = 0;
Cruise_R_TOTAL = 0;

m_curr = m_cruise0;

Cruise_h_vec    = [];
Cruise_m_vec    = [];
Cruise_V_vec    = [];
Cruise_Mach_vec = [];
Cruise_TSFC_vec = [];
Cruise_Treq_vec = [];
Cruise_Tav_vec  = [];
Cruise_t_vec    = [];
Cruise_R_vec    = [];

while Cruise_R_TOTAL < Cruise_R_target

    dR_step = min(dR_cruise, Cruise_R_target - Cruise_R_TOTAL);

    % -------- current cruise segment --------
    [rho_c,a_c,~,~] = cast.atmos(h_cruise);
    V_cruise = M_cruise * a_c;
    dt_step = dR_step / V_cruise;

    W = m_curr * g;

    CL = W / (0.5 * rho_c * V_cruise^2 * S);

    CD0 = 0.02;
    k   = 0.045;
    CD  = CD0 + k * CL^2;

    D_cruise = 0.5 * rho_c * V_cruise^2 * S * CD;

    Treq_cruise = D_cruise;
    Tav_cruise  = thrustMattinglyHighBPR(F_SL, h_cruise, M_cruise);

    if Tav_cruise < Treq_cruise
        warning('Cruise infeasible at current altitude/Mach')
    end

    ROC_cruise = ((Tav_cruise - Treq_cruise) * V_cruise) / W;

    if ROC_cruise < 300*ft2m/60
        warning('Cruise point fails 300 ft/min climb-rate criterion')
    end

    TSFC_cruise = ADP.Engine.TSFC(M_cruise, h_cruise);
    dM_step = TSFC_cruise * Treq_cruise * dt_step;

    m_curr = m_curr - dM_step;

    Cruise_DeltaM = Cruise_DeltaM + dM_step;
    Cruise_t_TOTAL = Cruise_t_TOTAL + dt_step;
    Cruise_R_TOTAL = Cruise_R_TOTAL + dR_step;

    Cruise_h_vec(end+1)    = h_cruise;
    Cruise_m_vec(end+1)    = m_curr;
    Cruise_V_vec(end+1)    = V_cruise;
    Cruise_Mach_vec(end+1) = M_cruise;
    Cruise_TSFC_vec(end+1) = TSFC_cruise;
    Cruise_Treq_vec(end+1) = Treq_cruise;
    Cruise_Tav_vec(end+1)  = Tav_cruise;
    Cruise_t_vec(end+1)    = Cruise_t_TOTAL;
    Cruise_R_vec(end+1)    = Cruise_R_TOTAL;

    % -------- try next 1000 ft block for following segment --------
    h_next = h_cruise + dh_cruise_block;

    if h_next <= h_cruise_upper

        [rho_n,a_n,~,~] = cast.atmos(h_next);
        V_next = M_cruise * a_n;
        W_next = m_curr * g;

        CL_next = W_next / (0.5 * rho_n * V_next^2 * S);
        CD_next = CD0 + k * CL_next^2;
        D_next  = 0.5 * rho_n * V_next^2 * S * CD_next;

        Tav_next = thrustMattinglyHighBPR(F_SL, h_next, M_cruise);
        ROC_next = ((Tav_next - D_next) * V_next) / W_next;

        % only step up if the next block is feasible and beneficial
        if (Tav_next >= D_next) && (ROC_next >= 300*ft2m/60) && (D_next < D_cruise)
            h_cruise = h_next;
        end
    end
end

m_end_cruise = m_curr;
Mfn_cruise = m_end_cruise / m_cruise0;

fprintf('Cruise Fuel: %.0f kg\n', Cruise_DeltaM)
fprintf('Cruise Fuel Fraction: %.4f\n', Mfn_cruise)
fprintf('Cruise Time: %.1f hr\n', Cruise_t_TOTAL/3600)
fprintf('Cruise Range: %.0f km\n', Cruise_R_TOTAL/1000)


%% ==========================================
%% DESCENT MODEL (mirror of climb)
%% ==========================================

% --- descent storage (SEPARATE from climb/cruise) ---
Descent_h_vec = [];
Descent_t_vec = [];
Descent_R_vec = [];

h_start = h_cruise;      % start from last cruise altitude
h_end   = 0;

dh_des = -500*ft2m;      % NEGATIVE step

Descent_DeltaM = 0;
Descent_R = 0;
Descent_t_TOTAL = 0;

h_nodes = h_start:dh_des:h_end;
if h_nodes(end) ~= h_end
    h_nodes = [h_nodes h_end];
end

for i = 1:(length(h_nodes)-1)

    h_high = h_nodes(i);
    h_low  = h_nodes(i+1);
    h_mid  = 0.5*(h_high + h_low);

    dh_step = h_low - h_high;   % NEGATIVE
    dt_step = 20;               % keep simple (constant time step)

    [rho,a,~,~] = cast.atmos(h_mid);

    % --- use SAME speed law as climb ---
    if h_mid > 20000*ft2m
        M = ADP.TLAR.M_c * 0.9;
        V = M * a;
    else
        V = 0.9*250*knots2m_s;
        M = V/a;
    end

    vy = dh_step/dt_step;   % NEGATIVE

    if abs(vy) >= V
        error('Descent invalid: |vy| >= V at h = %.1f m', h_mid);
    end

    vx = sqrt(V^2 - vy^2);
    Descent_R = Descent_R + vx*dt_step;

    % --- mass ---
    m_curr = m_end_cruise - Descent_DeltaM;

    W = m_curr * g;

    CL = W / (0.5 * rho * V^2 * S);

    CD0 = 0.02;
    k   = 0.045;
    CD  = CD0 + k * CL^2;

    D = 0.5 * rho * V^2 * S * CD;

    % 🔴 KEY DIFFERENCE (gravity helps)
    T_req = D - W*(abs(vy)/V);

    % don't allow negative thrust
    T_req = max(T_req, 0);

    TSFC = ADP.Engine.TSFC(M, h_mid);
    dM_step = TSFC * T_req * dt_step;

    Descent_DeltaM = Descent_DeltaM + dM_step;

    % --- time + range ---
    Descent_t_TOTAL = Descent_t_TOTAL + dt_step;

    % --- store ---
    Descent_t_vec(end+1) = Descent_t_TOTAL;
    Descent_R_vec(end+1) = Descent_R;
    Descent_h_vec(end+1) = h_mid;

end

Mfn_descent = (m_end_cruise - Descent_DeltaM) / m_end_cruise;


fprintf('Descent Fuel: %.0f kg\n', Descent_DeltaM)
fprintf('Descent Time: %.1f min\n', Descent_t_TOTAL/60)
fprintf('Descent Range: %.0f km\n', Descent_R/1000)

TotalRange = Climb_R_TOTAL + Cruise_R_TOTAL + Descent_R;

fprintf('Total Mission Range (reconstructed): %.1f km\n', TotalRange/1000)
fprintf('Target Mission Range: %.1f km\n', tripRange/1000)


fprintf('End of ground roll velocity (V_lof): %.1f m/s\n', V_lof)
fprintf('End of ground roll velocity (V_lof): %.1f knots\n', V_lof/knots2m_s)
fprintf('Ground roll end speed: %.1f m/s (%.0f kt)\n', V_lof, V_lof/knots2m_s)


% ========================
% CONTINGENCY FUEL
% ========================

m_landing_dest = m_end_cruise - Descent_DeltaM;



% 5 min loiter estimate (use your loiter model assumptions)
t_cont = 5*60;

h_cont = 1500*ft2m;
[rho,a,~,~] = cast.atmos(h_cont);

M_cont = 0.3;
V_cont = M_cont * a;

W_cont = m_landing_dest * g;

CL = W_cont / (0.5 * rho * V_cont^2 * S);
CD = 0.02 + 0.045*CL^2;

D = 0.5 * rho * V_cont^2 * S * CD;

TSFC_cont = ADP.Engine.TSFC(M_cont, h_cont);

Fuel_cont_time = TSFC_cont * D * t_cont;

% 3% rule
%Fuel_cont_3pct = 0.03 * Fuel_trip;

% FINAL contingency
% Fuel_cont = max(Fuel_cont_time, Fuel_cont_3pct);

Fuel_cont = Fuel_cont_time;

% ADD CONTINGENCY HERE
m_with_cont = m_landing_dest - Fuel_cont;
%% ==========================================
%% ALTERNATE MISSION (BOLT-ON)
%% no loiter, no contingency
%% approach already assumed inside descent
%% ==========================================



% Range_alternate = 350000;     % [m]
% altRange = Range_alternate;

% If you already added this TLAR field, use it instead:
altRange = ADP.TLAR.Range_alternate * 1;

h_alt_cruise = ADP.TLAR.Alt_alternate;  % you can set this independently if you want
ROC_factor_alt = ROC_factor;            % independent control knob for alternate climb
ROC_used_alt   = ROC_base * ROC_factor_alt;

% -------- alternate climb: start from end of destination descent --------
altClimb = runReserveClimb(ADP, m_with_cont, S, F_SL, h_alt_cruise, ...
    ROC_used_alt, dh, ft2m, knots2m_s, g);

% -------- estimate alternate descent range first --------
% this lets cruise target close the alternate mission range properly
altDesc_est = runReserveDescent(ADP, altClimb.m_end, S, h_alt_cruise, ...
    knots2m_s, g, ft2m);

AltCruise_R_target = altRange - altClimb.R_TOTAL - altDesc_est.R_TOTAL;

if AltCruise_R_target < 0
    warning(['Alternate range too short: climb + descent already exceed ', ...
             'the target. Forcing zero alternate cruise.'])
    AltCruise_R_target = 0;
end

% -------- alternate cruise --------
altCruise = runReserveCruise(ADP, altClimb.m_end, S, F_SL, h_alt_cruise, ...
    h_ceiling, Alt_max, AltCruise_R_target, ft2m, g);

% -------- alternate descent to landing --------
altDesc = runReserveDescent(ADP, altCruise.m_end, S, h_alt_cruise, ...
    knots2m_s, g, ft2m);

% -------- reserve totals --------
Reserve_m_TOTAL = altClimb.DeltaM + altCruise.DeltaM + altDesc.DeltaM;
Reserve_t_TOTAL = altClimb.t_TOTAL + altCruise.t_TOTAL + altDesc.t_TOTAL;
Reserve_R_TOTAL = altClimb.R_TOTAL + altCruise.R_TOTAL + altDesc.R_TOTAL;

Mfn_reserve = altDesc.m_end / m_landing_dest;
%ADP.Mf_res  = Reserve_m_TOTAL / ADP.MTOM;
ReserveFuelFrac_MTOM = Reserve_m_TOTAL / ADP.MTOM;


fprintf('\n--- ALTERNATE MISSION ---\n')
fprintf('Alternate climb fuel:   %.0f kg\n', altClimb.DeltaM)
fprintf('Alternate cruise fuel:  %.0f kg\n', altCruise.DeltaM)
fprintf('Alternate descent fuel: %.0f kg\n', altDesc.DeltaM)
fprintf('Reserve fuel total:     %.0f kg\n', Reserve_m_TOTAL)
fprintf('Reserve mass fraction:  %.5f\n', Mfn_reserve)
fprintf('Reserve / MTOM:         %.5f\n', ReserveFuelFrac_MTOM)
fprintf('Alternate time:         %.1f min\n', Reserve_t_TOTAL/60)
fprintf('Alternate range:        %.1f km\n', Reserve_R_TOTAL/1000)
fprintf('Alternate target:       %.1f km\n', altRange/1000)

% overwrite landing mass so your landing constraint and taxi-in use the
% actual final landing mass at the alternate airport
m_landing = altDesc.m_end;




function out = runReserveClimb(ADP, m0, S, F_SL, h4, ROC_used, dh, ft2m, knots2m_s, g)

    CD0 = 0.02;
    k   = 0.045;

    out.DeltaM = 0;
    out.R_TOTAL = 0;
    out.t_TOTAL = 0;

    m_curr = m0;

    % ---------------- 0 -> 1500 ft ----------------
    if h4 > 0
        h0 = 0;
        h1 = min(1500*ft2m, h4);
        dh01 = h1 - h0;
        Climb_t01 = 50;
        v01_cmd = 0.9 * 250 * knots2m_s;

        h_nodes = h0:dh:h1;
        if h_nodes(end) ~= h1, h_nodes = [h_nodes h1]; end

        for i = 1:(length(h_nodes)-1)
            h_low  = h_nodes(i);
            h_high = h_nodes(i+1);
            h_mid  = 0.5*(h_low + h_high);

            dh_step = h_high - h_low;
            dt_step = Climb_t01 * (dh_step/dh01);

            [rho,a,~,~] = cast.atmos(h_mid);

            V  = v01_cmd;
            M  = V/a;
            vy = dh_step/dt_step;

            vx = sqrt(V^2 - vy^2);
            out.R_TOTAL = out.R_TOTAL + vx*dt_step;
            out.t_TOTAL = out.t_TOTAL + dt_step;

            W  = m_curr * g;
            CL = W / (0.5 * rho * V^2 * S);
            CD = CD0 + k * CL^2;
            D  = 0.5 * rho * V^2 * S * CD;

            Treq = D + W*(vy/V);
            TSFC = ADP.Engine.TSFC(M, h_mid);
            dM   = TSFC * Treq * dt_step;

            out.DeltaM = out.DeltaM + dM;
            m_curr = m_curr - dM;
        end
    end

    % ---------------- 1500 -> 10000 ft ----------------
    if h4 > 1500*ft2m
        h0 = 1500*ft2m;
        h1 = min(10000*ft2m, h4);
        v_cmd = 0.9 * 250 * knots2m_s;

        h_nodes = h0:dh:h1;
        if h_nodes(end) ~= h1, h_nodes = [h_nodes h1]; end

        for i = 1:(length(h_nodes)-1)
            h_low  = h_nodes(i);
            h_high = h_nodes(i+1);
            h_mid  = 0.5*(h_low + h_high);

            dh_step = h_high - h_low;
            dt_step = dh_step / ROC_used;

            [rho,a,~,~] = cast.atmos(h_mid);

            V  = v_cmd;
            M  = V/a;
            vy = ROC_used;

            vx = sqrt(V^2 - vy^2);
            out.R_TOTAL = out.R_TOTAL + vx*dt_step;
            out.t_TOTAL = out.t_TOTAL + dt_step;

            W  = m_curr * g;
            CL = W / (0.5 * rho * V^2 * S);
            CD = CD0 + k * CL^2;
            D  = 0.5 * rho * V^2 * S * CD;

            Treq = D + W*(vy/V);
            TSFC = ADP.Engine.TSFC(M, h_mid);
            dM   = TSFC * Treq * dt_step;

            out.DeltaM = out.DeltaM + dM;
            m_curr = m_curr - dM;
        end
    end

    % ---------------- 10000 -> 20000 ft ----------------
    if h4 > 10000*ft2m
        h0 = 10000*ft2m;
        h1 = min(20000*ft2m, h4);
        v_cmd = 0.9 * 250 * knots2m_s;

        h_nodes = h0:dh:h1;
        if h_nodes(end) ~= h1, h_nodes = [h_nodes h1]; end

        for i = 1:(length(h_nodes)-1)
            h_low  = h_nodes(i);
            h_high = h_nodes(i+1);
            h_mid  = 0.5*(h_low + h_high);

            dh_step = h_high - h_low;
            dt_step = dh_step / ROC_used;

            [rho,a,~,~] = cast.atmos(h_mid);

            V  = v_cmd;
            M  = V/a;
            vy = ROC_used;

            vx = sqrt(V^2 - vy^2);
            out.R_TOTAL = out.R_TOTAL + vx*dt_step;
            out.t_TOTAL = out.t_TOTAL + dt_step;

            W  = m_curr * g;
            CL = W / (0.5 * rho * V^2 * S);
            CD = CD0 + k * CL^2;
            D  = 0.5 * rho * V^2 * S * CD;

            Treq = D + W*(vy/V);
            TSFC = ADP.Engine.TSFC(M, h_mid);
            dM   = TSFC * Treq * dt_step;

            out.DeltaM = out.DeltaM + dM;
            m_curr = m_curr - dM;
        end
    end

    % ---------------- 20000 ft -> cruise ----------------
    if h4 > 20000*ft2m
        h0 = 20000*ft2m;
        h1 = h4;
        M_cmd = ADP.TLAR.M_c * 0.9;

        h_nodes = h0:dh:h1;
        if h_nodes(end) ~= h1, h_nodes = [h_nodes h1]; end

        for i = 1:(length(h_nodes)-1)
            h_low  = h_nodes(i);
            h_high = h_nodes(i+1);
            h_mid  = 0.5*(h_low + h_high);

            dh_step = h_high - h_low;
            dt_step = dh_step / ROC_used;

            [rho,a,~,~] = cast.atmos(h_mid);

            M  = M_cmd;
            V  = M * a;
            vy = ROC_used;

            vx = sqrt(V^2 - vy^2);
            out.R_TOTAL = out.R_TOTAL + vx*dt_step;
            out.t_TOTAL = out.t_TOTAL + dt_step;

            W  = m_curr * g;
            CL = W / (0.5 * rho * V^2 * S);
            CD = CD0 + k * CL^2;
            D  = 0.5 * rho * V^2 * S * CD;

            Treq = D + W*(vy/V);
            TSFC = ADP.Engine.TSFC(M, h_mid);
            dM   = TSFC * Treq * dt_step;

            out.DeltaM = out.DeltaM + dM;
            m_curr = m_curr - dM;
        end
    end

    out.m_end = m_curr;
end


function out = runReserveCruise(ADP, m0, S, F_SL, h_start, h_ceiling, Alt_max, rangeTarget, ft2m, g)

    dh_cruise_block = 1000*ft2m;
    M_cruise = ADP.TLAR.M_c;
    h_cruise = h_start;

    h_cruise_upper = floor(min(h_ceiling, Alt_max)/dh_cruise_block) * dh_cruise_block;
    h_cruise_upper = max(h_cruise_upper, h_start);

    N_cruise_segments = 6;
    dR_cruise = max(rangeTarget,0) / max(N_cruise_segments,1);

    CD0 = 0.02;
    k   = 0.045;

    out.DeltaM = 0;
    out.t_TOTAL = 0;
    out.R_TOTAL = 0;

    m_curr = m0;

    while out.R_TOTAL < rangeTarget

        dR_step = min(dR_cruise, rangeTarget - out.R_TOTAL);

        [rho,a,~,~] = cast.atmos(h_cruise);
        V = M_cruise * a;
        dt_step = dR_step / V;

        W  = m_curr * g;
        CL = W / (0.5 * rho * V^2 * S);
        CD = CD0 + k * CL^2;
        D  = 0.5 * rho * V^2 * S * CD;

        Treq = D;
        Tav  = thrustMattinglyHighBPR(F_SL, h_cruise, M_cruise);

        if Tav < Treq
            warning('Alternate cruise infeasible at current altitude/Mach')
        end

        TSFC = ADP.Engine.TSFC(M_cruise, h_cruise);
        dM   = TSFC * Treq * dt_step;

        m_curr = m_curr - dM;
        out.DeltaM = out.DeltaM + dM;
        out.t_TOTAL = out.t_TOTAL + dt_step;
        out.R_TOTAL = out.R_TOTAL + dR_step;

        h_next = h_cruise + dh_cruise_block;

        if h_next <= h_cruise_upper
            [rho_n,a_n,~,~] = cast.atmos(h_next);
            Vn = M_cruise * a_n;
            Wn = m_curr * g;

            CLn = Wn / (0.5 * rho_n * Vn^2 * S);
            CDn = CD0 + k * CLn^2;
            Dn  = 0.5 * rho_n * Vn^2 * S * CDn;

            Tavn = thrustMattinglyHighBPR(F_SL, h_next, M_cruise);
            ROCn = ((Tavn - Dn) * Vn) / Wn;

            if (Tavn >= Dn) && (ROCn >= 300*ft2m/60) && (Dn < D)
                h_cruise = h_next;
            end
        end
    end

    out.m_end = m_curr;
end


function out = runReserveDescent(ADP, m0, S, h_start, knots2m_s, g, ft2m)

    h_end   = 0;
    dh_des  = -500*ft2m;

    CD0 = 0.02;
    k   = 0.045;

    out.DeltaM = 0;
    out.R_TOTAL = 0;
    out.t_TOTAL = 0;

    m_curr = m0;

    h_nodes = h_start:dh_des:h_end;
    if h_nodes(end) ~= h_end
        h_nodes = [h_nodes h_end];
    end

    for i = 1:(length(h_nodes)-1)

        h_high = h_nodes(i);
        h_low  = h_nodes(i+1);
        h_mid  = 0.5*(h_high + h_low);

        dh_step = h_low - h_high;   % negative
        dt_step = 20;

        [rho,a,~,~] = cast.atmos(h_mid);

        % if h_mid > 20000*ft2m
        %     M = ADP.TLAR.M_c * 0.9;
        %     V = M * a;
        % else
        %     V = 0.9 * 250 * knots2m_s;
        %     M = V / a;
        % end

        M = ADP.TLAR.M_c * 0.9;
        V = M * a;

        vy = dh_step / dt_step;     % negative
        vx = sqrt(V^2 - vy^2);

        out.R_TOTAL = out.R_TOTAL + vx*dt_step;
        out.t_TOTAL = out.t_TOTAL + dt_step;

        W  = m_curr * g;
        CL = W / (0.5 * rho * V^2 * S);
        CD = CD0 + k * CL^2;
        D  = 0.5 * rho * V^2 * S * CD;

        Treq = D - W*(abs(vy)/V);
        Treq = max(Treq, 0);

        TSFC = ADP.Engine.TSFC(M, h_mid);
        dM   = TSFC * Treq * dt_step;

        out.DeltaM = out.DeltaM + dM;
        m_curr = m_curr - dM;
    end

    out.m_end = m_curr;
end


%% ==========================================
%% LANDING CONSTRAINT (CS-25 APPROACH SPEED)
%% ==========================================

% ---------------- INPUTS ----------------
V_app_max = 145 * knots2m_s;   % requirement from spec
CL_max_land = 2.8;             % realistic landing config (2.2–2.8 range)
% CL_max_land = 1.5;             % realistic landing config (2.2–2.8 range)


% landing happens at sea level ISA
h_land = 0;
[rho_land,~,~,~] = cast.atmos(h_land);

% ---------------- MASS AT LANDING ----------------
% use FINAL landing mass (after alternate mission)
W_landing = m_landing * g;

% ---------------- STALL SPEED ----------------
V_stall_land = V_app_max / 1.3;

% ---------------- CONSTRAINT ----------------
WS_limit = 0.5 * rho_land * V_stall_land^2 * CL_max_land;

% ---------------- ACTUAL ----------------
WS_actual = W_landing / S;

% ---------------- CHECK ----------------
fprintf('\n--- LANDING CONSTRAINT ---\n')
fprintf('Max allowed W/S: %.0f N/m^2\n', WS_limit)
fprintf('Actual W/S:      %.0f N/m^2\n', WS_actual)

if WS_actual > WS_limit
    warning('❌ Landing constraint VIOLATED (wing too small or aircraft too heavy)')
else
    fprintf('✅ Landing constraint satisfied\n')
end

% ---------------- EXTRA (nice insight) ----------------
V_stall_actual = sqrt((2*W_landing)/(rho_land*S*CL_max_land));
V_app_actual   = 1.3 * V_stall_actual;

fprintf('Actual approach speed: %.1f knots\n', V_app_actual/knots2m_s)
fprintf('Limit approach speed:  %.1f knots\n', V_app_max/knots2m_s)


%% ==========================================
%% FINAL TAXI-IN + SHUTDOWN (BOLT-ON)
%% ==========================================

IdleFrac_land = 0.08;        % slightly lower than taxi-out
TaxiTime_land = 10*60;       % [s] reasonable assumption

T_idle_land = IdleFrac_land * T_TO;

TSFC_idle_land = ADP.Engine.TSFC(0,0);

FuelTaxi_land = TSFC_idle_land * T_idle_land * TaxiTime_land;

% --- FINAL MASS ---
m_final = m_landing - FuelTaxi_land;

% --- MASS FRACTION ---
Mfn_landing_taxi = m_final / m_landing;

% --- TIME ---
t_taxi_in = TaxiTime_land;

fprintf('Landing Taxi Fuel: %.0f kg\n', FuelTaxi_land)
fprintf('Final Mass: %.0f kg\n', m_final)

%% ==========================================
%% ABSOLUTE AXES (INCLUDING YOUR EXISTING GROUND PHASE)
%% ==========================================

% --- START OFFSET (you already computed these) ---
R0 = s_g;        % takeoff ground run distance
t0 = TaxiTime + t_TO;

% shift climb to start AFTER ground phase
R_climb_abs = R0 + R_vec;
t_climb_abs = t0 + t_vec;

% cruise
Cruise_R_abs = R_climb_abs(end) + Cruise_R_vec;
Cruise_t_abs = t_climb_abs(end) + Cruise_t_vec;

% descent
Descent_R_abs = Cruise_R_abs(end) + Descent_R_vec;
Descent_t_abs = Cruise_t_abs(end) + Descent_t_vec;

% --- LANDING (reuse same runway assumption, no new physics) ---
R_land_abs = [Descent_R_abs(end), Descent_R_abs(end) + linspace(0, s_g, 30)];
t_land_abs = [Descent_t_abs(end), Descent_t_abs(end) + linspace(0, t_TO, 30)];

h_land_vec = [Descent_h_vec(end), zeros(1,30)];

figure; hold on;

% ground roll (flat)
plot([0 R0]/1000, [0 0], 'LineWidth',2)

% climb
plot(R_climb_abs/1000, h_vec/ft2m, 'LineWidth',2)

% cruise
stairs([R_climb_abs(end), Cruise_R_abs]/1000, ...
       [h_vec(end), Cruise_h_vec]/ft2m, 'LineWidth',2)

% descent
plot(Descent_R_abs/1000, Descent_h_vec/ft2m, 'LineWidth',2)

% landing roll
plot(R_land_abs/1000, h_land_vec/ft2m, 'LineWidth',2)

xlabel('Mission Range [km]')
ylabel('Altitude [ft]')
title('Mission Profile (Ground → Air → Ground)')
grid on


figure; hold on;

% ground phase
plot([0 t0]/3600, [0 0], 'LineWidth',2)

% climb
plot(t_climb_abs/3600, h_vec/ft2m, 'LineWidth',2)

% cruise
stairs([t_climb_abs(end), Cruise_t_abs]/3600, ...
       [h_vec(end), Cruise_h_vec]/ft2m, 'LineWidth',2)

% descent
plot(Descent_t_abs/3600, Descent_h_vec/ft2m, 'LineWidth',2)

% landing roll
plot(t_land_abs/3600, h_land_vec/ft2m, 'LineWidth',2)

xlabel('Mission Time [hr]')
ylabel('Altitude [ft]')
title('Mission Profile (Time)')
grid on



% ========================
% LOITER (30 min @ 1500 ft)
% ========================

h_loiter = 1500*ft2m;
t_loiter = 30*60;

[rho,a,~,~] = cast.atmos(h_loiter);

M_loiter = 0.3; % reasonable assumption or optimise
V_loiter = M_loiter * a;

W = altDesc.m_end * g;

CL = W / (0.5 * rho * V_loiter^2 * S);
CD = 0.02 + 0.045*CL^2;

D = 0.5 * rho * V_loiter^2 * S * CD;

TSFC = ADP.Engine.TSFC(M_loiter, h_loiter);

Fuel_loiter = TSFC * D * t_loiter;

m_after_loiter = altDesc.m_end - Fuel_loiter;



%% ---------------- MASS TRACKING FIX (BOLT-ON) ----------------
% ================================
% CLEAN MASS CHECKPOINTS
% ================================

m0 = ADP.MTOM;                       % start

m1 = m01;                            % after taxi + takeoff
m2 = m4c;                            % after climb
m3 = m_end_cruise;                   % after cruise
m4 = m_end_cruise - Descent_DeltaM;  % after descent (DESTINATION)

m5 = m_landing;                      % after alternate mission
m6 = m_final;                        % after taxi-in (FINAL)

% ================================
% SEGMENT FUEL FRACTIONS (CORRECT)
% ================================

Mfn_taxi_TO = m1 / m0;
Mfn_climb   = m2 / m1;
Mfn_cruise  = m3 / m2;
Mfn_descent = m4 / m3;

% everything after destination landing = reserves
Mfn_reserve = m6 / m4;




t_taxi_TO = TaxiTime + t_TO;
t_climb    = Climb_t_TOTAL;
t_cruise   = Cruise_t_TOTAL;
t_descent  = Descent_t_TOTAL;

d_climb   = Climb_R_TOTAL;
d_cruise  = Cruise_R_TOTAL;
d_descent = Descent_R;

Mfn_taxi_TO = Mfn_taxi_TO;
Mfn_climb   = Mfn_climb;
Mfn_cruise  = Mfn_cruise;

% ================================
% FINAL FUEL FRACTIONS
% ================================

% ---- TRIP (NO RESERVES) ----
Mfn_trip = Mfn_taxi_TO * Mfn_climb * Mfn_cruise * Mfn_descent;
FuelFrac_trip = 1 - Mfn_trip;

% ---- BLOCK (WITH RESERVES) ----
Mfn_block = Mfn_trip * Mfn_reserve;
FuelFrac_block = 1 - Mfn_block;

% optional full block-fuel fraction

fprintf('Block fuel fraction:    %.5f\n', Mfn_block)

fprintf('\n\n================ MISSION DEBUG DUMP ================\n');

% ------------------ TIMES ------------------
fprintf('\n--- TIME BREAKDOWN ---\n');
fprintf('Taxi + TO Time      : %.2f min\n', t_taxi_TO/60);
fprintf('Climb Time          : %.2f min\n', t_climb/60);
fprintf('Cruise Time         : %.2f min\n', t_cruise/60);
fprintf('Descent Time        : %.2f min\n', t_descent/60);
fprintf('----------------------------------------\n');
fprintf('TOTAL MISSION TIME  : %.2f min\n', ...
    (t_taxi_TO + t_climb + t_cruise + t_descent)/60);

% ------------------ DISTANCES ------------------
fprintf('\n--- DISTANCE BREAKDOWN ---\n');
fprintf('Climb Range         : %.1f km\n', d_climb/1000);
fprintf('Cruise Range        : %.1f km\n', d_cruise/1000);
fprintf('Descent Range       : %.1f km\n', d_descent/1000);
fprintf('----------------------------------------\n');
fprintf('TOTAL RANGE         : %.1f km\n', ...
    (d_climb + d_cruise + d_descent)/1000);

% ------------------ FUEL FRACTIONS ------------------
fprintf('\n=== FINAL FUEL FRACTIONS ===\n')

fprintf('Trip Fuel Fraction (NO reserve): %.4f\n', FuelFrac_trip)
fprintf('Block Fuel Fraction (WITH reserve): %.4f\n', FuelFrac_block)
fprintf('Reserve contribution: %.4f\n', FuelFrac_block - FuelFrac_trip)

% ------------------ MASSES ------------------
fprintf('\n--- MASS TRACKING ---\n');
fprintf('MTOM                : %.0f kg\n', ADP.MTOM);
fprintf('After Taxi+TO       : %.0f kg\n', m1);
fprintf('After Climb         : %.0f kg\n', m2);
fprintf('After Cruise        : %.0f kg\n', m3);
fprintf('After Descent       : %.0f kg\n', m4);
fprintf('Final Mass          : %.0f kg\n', m_final);


% ------------------ SANITY CHECKS ------------------
fprintf('\n--- SANITY CHECKS ---\n');
fprintf('Climb Range / Total Range : %.2f\n', d_climb/(d_climb+d_cruise+d_descent));
fprintf('Cruise Fraction of Range  : %.2f\n', d_cruise/(d_climb+d_cruise+d_descent));

if d_climb > (d_climb + d_cruise + d_descent)
    fprintf('❌ ERROR: Climb exceeds total mission!\n');
end

if t_climb/60 > 40
    fprintf('⚠️ WARNING: Climb time very high\n');
end

fprintf('\n====================================================\n\n');



figure; hold on;



% =========================
% MAIN MISSION
% =========================

% --- ground ---
plot(linspace(0,s_g,20)/1000, zeros(1,20), 'k','LineWidth',2)

% --- climb ---
stairs((s_g + R_vec)/1000, h_vec/ft2m, 'b','LineWidth',2)

% --- cruise (STEPPED) ---
R_cruise_plot = [s_g + R_vec(end), s_g + R_vec(end) + Cruise_R_vec];
h_cruise_plot = [h_vec(end), Cruise_h_vec];

stairs(R_cruise_plot/1000, h_cruise_plot/ft2m, 'r','LineWidth',2)



% --- descent ---
R_descent_start = R_cruise_plot(end);
plot((R_descent_start + Descent_R_vec)/1000, ...
     Descent_h_vec/ft2m, 'g','LineWidth',2)

% --- connect cruise to descent (simple visual fix) ---
stairs([R_cruise_plot(end), R_descent_start + Descent_R_vec(1)]/1000, ...
     [Cruise_h_vec(end), Descent_h_vec(1)]/ft2m, ...
     'r','LineWidth',2)

% =========================
% ALTERNATE MISSION
% =========================

R_alt_start = R_descent_start + Descent_R_vec(end);

% --- alternate climb ---
R_alt_climb = linspace(0, altClimb.R_TOTAL, 50);
h_alt_climb = linspace(0, h_alt_cruise, 50);

stairs((R_alt_start + R_alt_climb)/1000, h_alt_climb/ft2m, ...
     'b--','LineWidth',2)

% --- alternate cruise (STEPPED) ---
R_alt_cruise = linspace(0, altCruise.R_TOTAL, length(Cruise_h_vec));
h_alt_cruise_vec = h_alt_cruise * ones(size(R_alt_cruise));

stairs((R_alt_start + altClimb.R_TOTAL + R_alt_cruise)/1000, ...
       h_alt_cruise_vec/ft2m, 'r--','LineWidth',2)

% --- alternate descent ---
R_alt_desc_start = R_alt_start + altClimb.R_TOTAL + altCruise.R_TOTAL;

stairs((R_alt_desc_start + linspace(0, altDesc.R_TOTAL, 50))/1000, ...
     linspace(h_alt_cruise,0,50)/ft2m, ...
     'g--','LineWidth',2)

% --- final landing ---
R_final = R_alt_desc_start + altDesc.R_TOTAL;
plot((R_final + linspace(0,s_g,20))/1000, zeros(1,20), ...
     'k','LineWidth',2)

xlabel('Range [km]')
ylabel('Altitude [ft]')
title('Full Mission Profile (Main + Alternate)')
grid on

% =========================
% ZOOM INSET (CLIMB REGION)
% =========================

ax_main = gca;  % current axes

% create small inset axes (position = [x y width height])
ax_zoom = axes('Position',[0.55 0.55 0.3 0.3]); 
hold(ax_zoom, 'on')

% plot SAME data into inset
stairs(ax_zoom, R_climb_abs/1000, h_vec/ft2m, 'b','LineWidth',2)
stairs(ax_zoom, [R_climb_abs(end), Cruise_R_abs]/1000, ...
       [h_vec(end), Cruise_h_vec]/ft2m, 'r','LineWidth',2)
plot(ax_zoom, Descent_R_abs/1000, Descent_h_vec/ft2m, 'g','LineWidth',2)

% zoom limits (climb region)
xlim(ax_zoom, [0 250])     % <-- adjust
ylim(ax_zoom, [0 35000])

grid(ax_zoom, 'on')
title(ax_zoom, 'Climb Zoom')

figure; hold on;

% =========================
% MAIN MISSION
% =========================

% ground
plot(linspace(0,t0,20)/3600, zeros(1,20), 'k','LineWidth',2)

% climb
stairs((t0 + t_vec)/3600, h_vec/ft2m, 'b','LineWidth',2)

% cruise (STEPPED)
t_cruise_plot = [t0 + t_vec(end), t0 + t_vec(end) + Cruise_t_vec];
h_cruise_plot = [h_vec(end), Cruise_h_vec];

stairs(t_cruise_plot/3600, h_cruise_plot/ft2m, 'r','LineWidth',2)


% descent
t_descent_start = t_cruise_plot(end);
stairs((t_descent_start + Descent_t_vec)/3600, ...
     Descent_h_vec/ft2m, 'g','LineWidth',2)

% --- tiny connector (actual join point) ---
plot([t_descent_start, t_descent_start + Descent_t_vec(1)]/3600, ...
     [Cruise_h_vec(end), Descent_h_vec(1)]/ft2m, ...
     'r','LineWidth',2)



% =========================
% ALTERNATE
% =========================

t_alt_start = t_descent_start + Descent_t_vec(end);

% alternate climb
stairs((t_alt_start + linspace(0,altClimb.t_TOTAL,50))/3600, ...
     linspace(0,h_alt_cruise,50)/ft2m, 'b--','LineWidth',2)

% alternate cruise (STEPPED)
t_alt_cruise = linspace(0, altCruise.t_TOTAL, length(Cruise_h_vec));
stairs((t_alt_start + altClimb.t_TOTAL + t_alt_cruise)/3600, ...
       (h_alt_cruise*ones(size(t_alt_cruise)))/ft2m, ...
       'r--','LineWidth',2)

% alternate descent
t_alt_desc_start = t_alt_start + altClimb.t_TOTAL + altCruise.t_TOTAL;

stairs((t_alt_desc_start + linspace(0,altDesc.t_TOTAL,50))/3600, ...
     linspace(h_alt_cruise,0,50)/ft2m, ...
     'g--','LineWidth',2)

% landing
t_final = t_alt_desc_start + altDesc.t_TOTAL;
plot((t_final + linspace(0,t_TO,20))/3600, zeros(1,20), ...
     'k','LineWidth',2)

xlabel('Time [hr]')
ylabel('Altitude [ft]')
title('Full Mission Profile (Time)')
grid on

% =========================
% ZOOM INSET (TIME - CLIMB REGION)
% =========================

ax_main = gca;

% create inset axes
ax_zoom_t = axes('Position',[0.55 0.55 0.3 0.3]); 
hold(ax_zoom_t, 'on')

% plot SAME data (time version)
stairs(ax_zoom_t, t_climb_abs/3600, h_vec/ft2m, 'b','LineWidth',2)

stairs(ax_zoom_t, [t_climb_abs(end), Cruise_t_abs]/3600, ...
       [h_vec(end), Cruise_h_vec]/ft2m, 'r','LineWidth',2)

plot(ax_zoom_t, Descent_t_abs/3600, ...
     Descent_h_vec/ft2m, 'g','LineWidth',2)

% zoom limits (KEY PART)
xlim(ax_zoom_t, [0 0.25])   % ~ first 15 min (adjust if needed)
ylim(ax_zoom_t, [0 35000])

grid(ax_zoom_t, 'on')
title(ax_zoom_t, 'Climb Zoom (Time)')


fprintf('\n\n================ FULL MISSION BREAKDOWN ================\n');

% ---------------- MAIN MISSION ----------------
fprintf('\n--- MAIN MISSION ---\n');

fprintf('\n[Taxi Out]\n');
fprintf('Fuel: %.0f kg | Time: %.1f min\n', FuelTaxi, TaxiTime/60);

fprintf('\n[Takeoff]\n');
fprintf('Fuel: %.0f kg | Time: %.1f s | Distance: %.0f m\n', ...
    FuelTO, t_TO, s_g);

fprintf('\n[Climb Segments]\n');
fprintf('0–1500 ft    : Fuel = %.0f kg | Range = %.1f km\n', Climb_DeltaM_01, Climb_R01/1000);
fprintf('1500–10000 ft: Fuel = %.0f kg | Range = %.1f km\n', Climb_DeltaM_12, Climb_R12/1000);
fprintf('10000–20000 ft: Fuel = %.0f kg | Range = %.1f km\n', Climb_DeltaM_23, Climb_R23/1000);
fprintf('20000–Cruise : Fuel = %.0f kg | Range = %.1f km\n', Climb_DeltaM_34, Climb_R34/1000);

fprintf('TOTAL CLIMB  : Fuel = %.0f kg | Time = %.1f min | Range = %.1f km\n', ...
    Climb_m_TOTAL, Climb_t_TOTAL/60, Climb_R_TOTAL/1000);

fprintf('\n[Cruise]\n');
fprintf('Fuel: %.0f kg | Time: %.2f hr | Range: %.1f km\n', ...
    Cruise_DeltaM, Cruise_t_TOTAL/3600, Cruise_R_TOTAL/1000);

fprintf('\n[Descent]\n');
fprintf('Fuel: %.0f kg | Time: %.1f min | Range: %.1f km\n', ...
    Descent_DeltaM, Descent_t_TOTAL/60, Descent_R/1000);

% ---------------- CONTINGENCY ----------------
fprintf('\n[Contingency - 5 min hold]\n');
fprintf('Fuel: %.0f kg | Time: %.1f min\n', Fuel_cont, t_cont/60);

% ---------------- ALTERNATE ----------------
fprintf('\n--- ALTERNATE MISSION ---\n');

fprintf('\n[Alt Climb]\n');
fprintf('Fuel: %.0f kg | Time: %.1f min | Range: %.1f km\n', ...
    altClimb.DeltaM, altClimb.t_TOTAL/60, altClimb.R_TOTAL/1000);

fprintf('\n[Alt Cruise]\n');
fprintf('Fuel: %.0f kg | Time: %.1f min | Range: %.1f km\n', ...
    altCruise.DeltaM, altCruise.t_TOTAL/60, altCruise.R_TOTAL/1000);

fprintf('\n[Alt Descent]\n');
fprintf('Fuel: %.0f kg | Time: %.1f min | Range: %.1f km\n', ...
    altDesc.DeltaM, altDesc.t_TOTAL/60, altDesc.R_TOTAL/1000);

fprintf('\nTOTAL ALTERNATE:\n');
fprintf('Fuel: %.0f kg | Time: %.1f min | Range: %.1f km\n', ...
    Reserve_m_TOTAL, Reserve_t_TOTAL/60, Reserve_R_TOTAL/1000);

% ---------------- FINAL ----------------
fprintf('\n[Taxi In]\n');
fprintf('Fuel: %.0f kg | Time: %.1f min\n', FuelTaxi_land, t_taxi_in/60);

fprintf('\n--- TOTALS ---\n');
fprintf('Trip Fuel (no reserve): %.0f kg\n', ADP.MTOM*(1 - Mfn_trip));
fprintf('Block Fuel (with reserve): %.0f kg\n', ADP.MTOM*(1 - Mfn_block));

fprintf('\n========================================================\n');