function takeoff_landing(obj)
% -----------------------------
% Example aircraft inputs
% -----------------------------
MTOM   = obj.MTOM; % mass [kg]
WingArea   = obj.WingArea; % wing area [m^2]
Thrust   = obj.Thrust; % total thrust [N]
ac.CD0_TO = 0.045; % zero-lift drag coeff in takeoff config
ac.CD0_LD = 0.090; % zero-lift drag coeff in landing config
ac.K   = 0.045; % induced drag factor

% probably fine constants 
ac.mu_roll  = 0.02;  % rolling friction
ac.mu_brake = 0.35;  % braking friction
ac.rho = 1.225; % air density [kg/m^3]
ac.g   = 9.81; % gravity [m/s^2]

% High-lift data
ac.CLmax_TO = 2.4;
ac.CLmax_LD = 3.0;

% Operational assumptions
ac.screen_h_TO = 10.7;   % 35 ft obstacle [m]
ac.screen_h_LD = 15.24;  % 50 ft obstacle [m]
ac.gamma_climb = deg2rad(8);   % assumed climb flight path angle after lift-off
ac.gamma_app   = deg2rad(3);   % assumed landing approach angle
ac.n_rot = 1.10;         % rotation speed multiple of stall
ac.n_lof = 1.20;         % lift-off speed multiple of stall
ac.n_app = 1.30;         % approach speed multiple of stall
ac.n_td  = 1.15;         % touchdown speed multiple of stall

% -----------------------------
% Run
% -----------------------------
TO = takeoff_distance(ac);
LD = landing_distance(ac);

disp('--- TAKEOFF ---')
disp(TO)

disp('--- LANDING ---')
disp(LD)

end


function TO = takeoff_distance(ac)
% TAKEOFF_DISTANCE
% Simple conceptual takeoff model:
% 1) Ground roll from rest to lift-off
% 2) Transition/climb to screen height

W = MTOM * ac.g;
rho = ac.rho;
S = WingArea;

% -----------------------------
% Characteristic speeds
% -----------------------------
Vs_to  = sqrt(2*W/(rho*S*ac.CLmax_TO));
Vr     = ac.n_rot * Vs_to;
Vlof   = ac.n_lof * Vs_to;

% -----------------------------
% Ground roll integration
% -----------------------------
dt = 0.1;
V  = 0.1;
x  = 0.0;
t  = 0.0;

while V < Vlof
    % crude CL build-up during roll
    CL = ac.CLmax_TO * min((V/Vlof)^2, 1.0) * 0.7;

    CD = ac.CD0_TO + ac.K * CL^2;

    q = 0.5 * rho * V^2;
    L = q * S * CL;
    D = q * S * CD;

    N = max(W - L, 0);
    F_fric = ac.mu_roll * N;

    a = (Thrust - D - F_fric) / MTOM;

    if a < 0.05
        a = 0.05;
    end

    V = V + a*dt;
    x = x + V*dt;
    t = t + dt;
end

Sg = x;
tg = t;

% -----------------------------
% Airborne distance to screen height
% -----------------------------
% Very simple: assume straight climb at gamma_climb
Sc = ac.screen_h_TO / tan(ac.gamma_climb);

TO.Vs_to = Vs_to;
TO.Vr    = Vr;
TO.Vlof  = Vlof;
TO.Sg    = Sg;
TO.Sc    = Sc;
TO.STO   = Sg + Sc;
TO.tg    = tg;
end


function LD = landing_distance(ac)
% LANDING_DISTANCE
% Simple conceptual landing model:
% 1) Approach from 50 ft on straight glide path
% 2) Ground roll braking from touchdown speed to zero

W = MTOM * ac.g;
rho = ac.rho;
S = WingArea;

% -----------------------------
% Characteristic speeds
% -----------------------------
Vs_ld = sqrt(2*W/(rho*S*ac.CLmax_LD));
Vapp  = ac.n_app * Vs_ld;
Vtd   = ac.n_td  * Vs_ld;

% -----------------------------
% Air distance from screen height
% -----------------------------
Sa = ac.screen_h_LD / tan(ac.gamma_app);

% -----------------------------
% Ground braking integration
% -----------------------------
dt = 0.1;
V  = Vtd;
x  = 0.0;
t  = 0.0;

while V > 0.5
    % During landing roll, assume lift decays with speed
    CL = ac.CLmax_LD * (V/Vtd)^2 * 0.3;
    CD = ac.CD0_LD + ac.K * CL^2;

    q = 0.5 * rho * V^2;
    L = q * S * CL;
    D = q * S * CD;

    N = max(W - L, 0);
    F_brake = ac.mu_brake * N;

    a = -(D + F_brake) / MTOM;

    V = max(V + a*dt, 0);
    x = x + V*dt;
    t = t + dt;
end

Sbr = x;

LD.Vs_ld = Vs_ld;
LD.Vapp  = Vapp;
LD.Vtd   = Vtd;
LD.Sa    = Sa;
LD.Sbr   = Sbr;
LD.SLND  = Sa + Sbr;
LD.tbr   = t;
end