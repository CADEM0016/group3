%% ==========================================
% CLIMB MODEL — 500 ft STEPPED, 4 SEGMENTS
% ==========================================

%clear all
%scripts.ExampleUnconventional

%% ---------------------- Pull from ADP ----------------------
m01 = ADP.MTOM;
S   = ADP.WingArea;

ft2m = 0.3048;
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

Mf_taxi_TO   = m01 / ADP.MTOM;


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

FuelClimb = 0;
TimeClimb = 0;
RangeClimb = 0;




%% ============================================================
%% 0 --> 1500 ft
%% ============================================================
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
DeltaT_01 = T01_av - T01_req;

FuelClimb  = FuelClimb + Climb_DeltaM_01;
TimeClimb  = TimeClimb + Climb_t01;
RangeClimb = RangeClimb + Climb_R01;

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

Climb_t123 = 25*60; % total time 1500 --> 20000
Climb_t12 = Climb_t123*(dh12/(dh12 + dh23_tmp));
t12_cmd = Climb_t12; %#ok<NASGU>

v12_cmd = 0.9*250*knots2m_s;

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
    dt_step = Climb_t12 * (dh_step/dh12);

    [rho12,a12,~,~] = cast.atmos(h12);

    v12  = v12_cmd;
    M_12 = v12/a12;
    vy12 = dh_step/dt_step;

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
end

m23 = m12 - Climb_DeltaM_12;
DeltaT_12 = T12_av - T12_req;

FuelClimb  = FuelClimb + Climb_DeltaM_12;
TimeClimb  = TimeClimb + Climb_t12;
RangeClimb = RangeClimb + Climb_R12;

%% ============================================================
%% 10000 --> 20000 ft
%% ============================================================
h2 = 10000*ft2m;
h3 = 20000*ft2m;

dh23 = h3 - h2;
Climb_t23 = Climb_t123*(dh23/(dh12 + dh23));

%v23_cmd = 250*knots2m_s;
% If you want 0.9*250 here as well, change previous line to:
v23_cmd = 0.9*250*knots2m_s;

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
    dt_step = Climb_t23 * (dh_step/dh23);

    [rho23,a23,~,~] = cast.atmos(h23);

    v23  = v23_cmd;
    M_23 = v23/a23;
    vy23 = dh_step/dt_step;

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
end

m34 = m23 - Climb_DeltaM_23;
DeltaT_23 = T23_av - T23_req;

FuelClimb  = FuelClimb + Climb_DeltaM_23;
TimeClimb  = TimeClimb + Climb_t23;
RangeClimb = RangeClimb + Climb_R23;

%% ============================================================
%% 20000 ft --> cruise
%% ============================================================
h3 = 20000*ft2m;
h4 = ADP.TLAR.Alt_cruise;   % was 34000*ft2m in your old script

dh34 = h4 - h3;

%M_34_cmd = 0.84;
% If you want to tie this to TLAR instead:
M_34_cmd = ADP.TLAR.M_c * 0.9;

% carry forward previous segment ROC
vy34_cmd = 0.5*(dh23/Climb_t23);

Climb_t34 = dh34/vy34_cmd;

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
    vy34 = dh_step/dt_step;

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
TimeClimb  = TimeClimb + Climb_t34;
RangeClimb = RangeClimb + Climb_R34;

%% -------------------------- Outputs -------------------------
Climb_m_TOTAL = Climb_DeltaM_01 + Climb_DeltaM_12 + Climb_DeltaM_23 + Climb_DeltaM_34;

Mf_climb = m4c/m01;

Climb_t_TOTAL = Climb_t01 + Climb_t12 + Climb_t23 + Climb_t34;
Climb_R_TOTAL = Climb_R01 + Climb_R12 + Climb_R23 + Climb_R34;

% feasibility checks
if DeltaT_01 < 0, warning('0-1500 ft infeasible'); end
if DeltaT_12 < 0, warning('1500-10000 ft infeasible'); end
if DeltaT_23 < 0, warning('10000-20000 ft infeasible'); end
if DeltaT_34 < 0, warning('20000-cruise infeasible'); end

fprintf('Climb Fuel: %.0f kg\n', Climb_m_TOTAL)
fprintf('Climb Fuel Fraction: %.4f\n', Mf_climb)
fprintf('Climb Time: %.1f min\n', Climb_t_TOTAL/60)
fprintf('Climb Range: %.0f km\n', Climb_R_TOTAL/1000)



figure;
plot(t_vec/60, h_vec/ft2m, 'LineWidth', 2);
xlabel('Time [min]');
ylabel('Altitude [ft]');
title('Climb Profile: Altitude vs Time');
grid on;


figure;
plot(R_vec/1000, h_vec/ft2m, 'LineWidth', 2);
xlabel('Range [km]');
ylabel('Altitude [ft]');
title('Climb Profile: Altitude vs Range');
grid on;



fprintf('Wf_taxi_TO:   %.4f\n', Mf_taxi_TO)
fprintf('Wf_climb:     %.4f\n', Mf_climb)

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