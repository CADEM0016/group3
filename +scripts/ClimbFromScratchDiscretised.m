%% ==========================================
% CLIMB MODEL — 500 ft STEPPED, 4 SEGMENTS
% ==========================================

% clear all
% scripts.ExampleUnconventional

%% ---------------------- Pull from ADP ----------------------
m01 = ADP.MTOM;
S   = ADP.WingArea;
D   = ADP.Engine.Diameter;   % if this fails, check engine field name

ft2m = 0.3048;
knots2m_s = 0.5144;
g = 9.81;

A = pi*(D/2)^2;

dh = 500*ft2m;

aimFractClimb = 0.985; %#ok<NASGU>

%% ------------------ Low fidelity assumptions ----------------
Cd = 0.03;

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

V0 = 85; % arbitrary calibration speed
DeltaV = T_TO/(rho0*A*V0);

%% ---------------------- Global stores -----------------------
h_vec    = [];
m_vec    = [];
V_vec    = [];
Mach_vec = [];
TSFC_vec = [];
Treq_vec = [];
Tav_vec  = [];

FuelClimb = 0;
TimeClimb = 0;
RangeClimb = 0;

%% ============================================================
%% 0 --> 1500 ft
%% ============================================================
h0 = 0;
h1 = 1500*ft2m;

dh01 = h1 - h0;
t01  = 50;                       % segment total time [s]
v01_cmd = 0.9*250*knots2m_s;     % commanded speed for this segment

h_nodes = h0:dh:h1;
if h_nodes(end) ~= h1
    h_nodes = [h_nodes h1];
end

DeltaM_01 = 0;
R01 = 0;

for i = 1:(length(h_nodes)-1)
    h_low  = h_nodes(i);
    h_high = h_nodes(i+1);
    h01    = 0.5*(h_low + h_high);

    dh_step = h_high - h_low;
    dt_step = t01 * (dh_step/dh01);

    [rho01,a01,~,~] = cast.atmos(h01);

    v01  = v01_cmd;
    M_01 = v01/a01;
    vy01 = dh_step/dt_step;

    if vy01 >= v01
        error('Segment 01 invalid: vy >= V at h = %.1f m', h01);
    end

    vx01 = sqrt(v01^2 - vy01^2);
    R01  = R01 + vx01*dt_step;

    D01 = 0.5*rho01*(v01^2)*S*Cd;

    m_curr = m01 - DeltaM_01;

    T01_req = D01 + (m_curr*g)*(vy01/v01);
    T01_av  = rho01*A*v01*DeltaV;

    TSFC_01 = ADP.Engine.TSFC(M_01, h01);
    dM_step = TSFC_01 * T01_req * dt_step;

    DeltaM_01 = DeltaM_01 + dM_step;

    % stores
    h_vec(end+1)    = h01;
    m_vec(end+1)    = m_curr - dM_step;
    V_vec(end+1)    = v01;
    Mach_vec(end+1) = M_01;
    TSFC_vec(end+1) = TSFC_01;
    Treq_vec(end+1) = T01_req;
    Tav_vec(end+1)  = T01_av;
end

m12 = m01 - DeltaM_01;
DeltaT_01 = T01_av - T01_req;

FuelClimb  = FuelClimb + DeltaM_01;
TimeClimb  = TimeClimb + t01;
RangeClimb = RangeClimb + R01;

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

t123 = 25*60; % total time 1500 --> 20000
t12 = t123*(dh12/(dh12 + dh23_tmp));
t12_cmd = t12; %#ok<NASGU>

v12_cmd = 0.9*250*knots2m_s;

h_nodes = h1:dh:h2;
if h_nodes(end) ~= h2
    h_nodes = [h_nodes h2];
end

DeltaM_12 = 0;
R12 = 0;

for i = 1:(length(h_nodes)-1)
    h_low  = h_nodes(i);
    h_high = h_nodes(i+1);
    h12    = 0.5*(h_low + h_high);

    dh_step = h_high - h_low;
    dt_step = t12 * (dh_step/dh12);

    [rho12,a12,~,~] = cast.atmos(h12);

    v12  = v12_cmd;
    M_12 = v12/a12;
    vy12 = dh_step/dt_step;

    if vy12 >= v12
        error('Segment 12 invalid: vy >= V at h = %.1f m', h12);
    end

    vx12 = sqrt(v12^2 - vy12^2);
    R12  = R12 + vx12*dt_step;

    D12 = 0.5*rho12*(v12^2)*S*Cd;

    m_curr = m12 - DeltaM_12;

    T12_req = D12 + (m_curr*g)*(vy12/v12);
    T12_av  = rho12*A*v12*DeltaV;

    TSFC_12 = ADP.Engine.TSFC(M_12, h12);
    dM_step = TSFC_12 * T12_req * dt_step;

    DeltaM_12 = DeltaM_12 + dM_step;

    % stores
    h_vec(end+1)    = h12;
    m_vec(end+1)    = m_curr - dM_step;
    V_vec(end+1)    = v12;
    Mach_vec(end+1) = M_12;
    TSFC_vec(end+1) = TSFC_12;
    Treq_vec(end+1) = T12_req;
    Tav_vec(end+1)  = T12_av;
end

m23 = m12 - DeltaM_12;
DeltaT_12 = T12_av - T12_req;

FuelClimb  = FuelClimb + DeltaM_12;
TimeClimb  = TimeClimb + t12;
RangeClimb = RangeClimb + R12;

%% ============================================================
%% 10000 --> 20000 ft
%% ============================================================
h2 = 10000*ft2m;
h3 = 20000*ft2m;

dh23 = h3 - h2;
t23 = t123*(dh23/(dh12 + dh23));

%v23_cmd = 250*knots2m_s;
% If you want 0.9*250 here as well, change previous line to:
v23_cmd = 0.9*250*knots2m_s;

h_nodes = h2:dh:h3;
if h_nodes(end) ~= h3
    h_nodes = [h_nodes h3];
end

DeltaM_23 = 0;
R23 = 0;

for i = 1:(length(h_nodes)-1)
    h_low  = h_nodes(i);
    h_high = h_nodes(i+1);
    h23    = 0.5*(h_low + h_high);

    dh_step = h_high - h_low;
    dt_step = t23 * (dh_step/dh23);

    [rho23,a23,~,~] = cast.atmos(h23);

    v23  = v23_cmd;
    M_23 = v23/a23;
    vy23 = dh_step/dt_step;

    if vy23 >= v23
        error('Segment 23 invalid: vy >= V at h = %.1f m', h23);
    end

    vx23 = sqrt(v23^2 - vy23^2);
    R23  = R23 + vx23*dt_step;

    D23 = 0.5*rho23*(v23^2)*S*Cd;

    m_curr = m23 - DeltaM_23;

    T23_req = D23 + (m_curr*g)*(vy23/v23);
    T23_av  = rho23*A*v23*DeltaV;

    TSFC_23 = ADP.Engine.TSFC(M_23, h23);
    dM_step = TSFC_23 * T23_req * dt_step;

    DeltaM_23 = DeltaM_23 + dM_step;

    % stores
    h_vec(end+1)    = h23;
    m_vec(end+1)    = m_curr - dM_step;
    V_vec(end+1)    = v23;
    Mach_vec(end+1) = M_23;
    TSFC_vec(end+1) = TSFC_23;
    Treq_vec(end+1) = T23_req;
    Tav_vec(end+1)  = T23_av;
end

m34 = m23 - DeltaM_23;
DeltaT_23 = T23_av - T23_req;

FuelClimb  = FuelClimb + DeltaM_23;
TimeClimb  = TimeClimb + t23;
RangeClimb = RangeClimb + R23;

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
vy34_cmd = dh23/t23;

t34 = dh34/vy34_cmd;

h_nodes = h3:dh:h4;
if h_nodes(end) ~= h4
    h_nodes = [h_nodes h4];
end

DeltaM_34 = 0;
R34 = 0;

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
    R34  = R34 + vx34*dt_step;

    D34 = 0.5*rho34*(v34^2)*S*Cd;

    m_curr = m34 - DeltaM_34;

    T34_req = D34 + (m_curr*g)*(vy34/v34);
    T34_av  = rho34*A*v34*DeltaV;

    TSFC_34 = ADP.Engine.TSFC(M_34, h34);
    dM_step = TSFC_34 * T34_req * dt_step;

    DeltaM_34 = DeltaM_34 + dM_step;

    % stores
    h_vec(end+1)    = h34;
    m_vec(end+1)    = m_curr - dM_step;
    V_vec(end+1)    = v34;
    Mach_vec(end+1) = M_34;
    TSFC_vec(end+1) = TSFC_34;
    Treq_vec(end+1) = T34_req;
    Tav_vec(end+1)  = T34_av;
end

m4c = m34 - DeltaM_34;
DeltaT_34 = T34_av - T34_req;

FuelClimb  = FuelClimb + DeltaM_34;
TimeClimb  = TimeClimb + t34;
RangeClimb = RangeClimb + R34;

%% -------------------------- Outputs -------------------------
m_TOTAL = DeltaM_01 + DeltaM_12 + DeltaM_23 + DeltaM_34;

FUELFRACTION = m4c/m01;

t_TOTAL = t01 + t12 + t23 + t34;
R_TOTAL = R01 + R12 + R23 + R34;

% feasibility checks
if DeltaT_01 < 0, warning('0-1500 ft infeasible'); end
if DeltaT_12 < 0, warning('1500-10000 ft infeasible'); end
if DeltaT_23 < 0, warning('10000-20000 ft infeasible'); end
if DeltaT_34 < 0, warning('20000-cruise infeasible'); end

fprintf('Climb Fuel: %.0f kg\n', m_TOTAL)
fprintf('Climb Fuel Fraction: %.4f\n', FUELFRACTION)
fprintf('Climb Time: %.1f min\n', t_TOTAL/60)
fprintf('Climb Range: %.0f km\n', R_TOTAL/1000)