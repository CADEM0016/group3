%clear all
%scripts.ExampleUnconventional



m01 = ADP.MTOM;
S   = ADP.WingArea;
D   = ADP.Engine.Diameter;   % <-- check exact field name

% MTOM_climb = 482513;
% m01 = MTOM_climb;
% S = 755.5;

ft2m = 0.3048;
knots2m_s = 0.5144;
g = 9.81;
% Diameter = 2.9951;
A = pi*(D/2)^2;

dh = 500*ft2m;

aimFractClimb = 0.985;

%% low fidelity assumptions

Cd = 0.03;


%% T_achievable

h0 = 0;

[rho0,~,~,~] = cast.atmos(h0);

Cl_TO = 0.8;
Cl_max = 1.5; % not used for now
Cd_TO = 0.03;
L2D_TO = Cl_TO/Cd_TO;
mu = 0.04;
V0 = 85; %ARBIRTRARY

s_g = 2900;

M_TO = m01;

T2W_crit = ((1.21/(g*rho0*Cl_TO*s_g))*(m01*g/S))+(0.5*(1/L2D_TO))+(0.5*mu);

T_TO = T2W_crit*m01*g;


DeltaV = T_TO/(rho0*A*V0);



%% 0--> 1500

h0 = 0;
h1 = 1500*ft2m;

dh01 = h1-h0;

h01 = ((dh01)/2) + h0;


[rho01,a01,~,~] = cast.atmos(h01);

t01 = 50; %VARIABLE CONNSTRAINT

v01 = 0.9*250*knots2m_s; %VARIABLE CONSTRAINT
M_01 = v01/a01; %mach at altitude
vy01 = dh01/t01;

vx01 = ((v01^2) - (vy01^2))^0.5;
R01 = vx01*t01;

D01 = 0.5*rho01*(v01^2)*S*Cd; %VARIABLE CD

T01_req = D01 + (m01*g)*(vy01/v01);

T01_av = rho01*A*v01*DeltaV;

DeltaT_01 = T01_av - T01_req; %feasiblity check

TSFC_01 = ADP.Engine.TSFC(M_01, h01);
DeltaM_01 = TSFC_01 * T01_req * t01;

m12 = m01-DeltaM_01;

%% 1500--> 10,000, 10,000 --> 20,000 time setup

h1 = 1500*ft2m;
h2 = 10000*ft2m;

dh12 = h2-h1;

h12 = ((dh12)/2) + h1;


[rho12,a12,~,~] = cast.atmos(h12);


h2 = 10000*ft2m;
h3 = 20000*ft2m;

dh23 = h3-h2;

h23 = ((dh23)/2) + h2;


[rho23,a23,~,~] = cast.atmos(h23);

t123 = 25*60; %VARIABLE CONNSTRAINT
t12 = t123*(dh12/(dh12+dh23)); %VARIABLE CONNSTRAINT
t23 = t123*(dh23/(dh12+dh23)); %VARIABLE CONNSTRAINT


%% 1500--> 10,000


v12 = 0.9*250*knots2m_s; %VARIABLE CONSTRAINT
M_12 = v12/a12; %mach at altitude
vy12 = dh12/t12;

vx12 = ((v12^2) - (vy12^2))^0.5;
R12 = vx12*t12;



D12 = 0.5*rho12*(v12^2)*S*Cd; %VARIABLE CD

T12_req = D12 + (m12*g)*(vy12/v12);

T12_av = rho12*A*v12*DeltaV;

DeltaT_12 = T12_av - T12_req; %feasiblity check

TSFC_12 = ADP.Engine.TSFC(M_12, h12);
DeltaM_12 = TSFC_12 * T12_req * t12;

m23 = m12-DeltaM_12;


%% 10,000 --> 20,000


v23 = 0.9*250*knots2m_s; %VARIABLE CONSTRAINT
M_23 = v23/a23; %mach at altitude
vy23 = dh23/t23;

vx23 = ((v23^2) - (vy23^2))^0.5;
R23 = vx23*t23;


D23 = 0.5*rho23*(v23^2)*S*Cd; %VARIABLE CD

T23_req = D23 + (m23*g)*(vy23/v23);

T23_av = rho23*A*v23*DeltaV;

DeltaT_23 = T23_av - T23_req; %feasiblity check

TSFC_23 = ADP.Engine.TSFC(M_23, h23);
DeltaM_23 = TSFC_23 * T23_req * t23;

m34 = m23-DeltaM_23;



%% 20,000 --> cruise

h3 = 20000*ft2m;
h4 = 34000*ft2m;

dh34 = h4-h3;

h34 = ((dh34)/2) + h3;


[rho34,a34,~,~] = cast.atmos(h34);


M_34 = 0.84*0.9; %mach at altitude
v34 = M_34*a34; %VARIABLE CONSTRAINT


vy34 = vy23;

t34 = dh34/vy34;


vx34 = ((v34^2) - (vy34^2))^0.5;
R34 = vx34*t34;



D34 = 0.5*rho34*(v34^2)*S*Cd; %VARIABLE CD

T34_req = D34 + (m34*g)*(vy34/v34);

T34_av = rho34*A*v34*DeltaV;

DeltaT_34 = T34_av - T34_req; %feasiblity check

TSFC_34 = ADP.Engine.TSFC(M_34, h34);
DeltaM_34 = TSFC_34 * T34_req * t34;

m4c = m34-DeltaM_34;



m_TOTAL = DeltaM_01 + DeltaM_12 + DeltaM_23 + DeltaM_34;

FUELFRACTION = m4c/m01;

t_TOTAL = t01+t12+t23+t34;

R_TOTAL = R01+R12+R23+R34;

% feasibility checks
if DeltaT_01 < 0, warning('0-1500 ft infeasible'); end
if DeltaT_12 < 0, warning('1500-10000 ft infeasible'); end
if DeltaT_23 < 0, warning('10000-20000 ft infeasible'); end
if DeltaT_34 < 0, warning('20000-cruise infeasible'); end% feasibility checks

