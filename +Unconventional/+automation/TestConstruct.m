%% Test script for construct_avl

clear
clc

% ---------------------------
% Define aircraft parameters
% ---------------------------

obj = struct();

% Reference geometry
obj.WingArea = 600; % m^2
obj.CMAC = 5; % MAC
obj.WingSpan = 76.0; % span

% Wing placement
obj.WingStart = 30.0;
obj.WingHeight = 0.0;
obj.AOA_incident = 2.0;

% Wing geometry
obj.RootChord = 15.0;
obj.TipChord  = 1.8;
obj.SweepAngle= 32; % Degrees
obj.DihedralAngle= 4; % Degrees
obj.Kinky = 8;


% Wing geometery - X/Z offsets
[Tipx,Tipz] = Unconventional.automation.leading_offset(obj.SweepAngle,obj.DihedralAngle,(obj.WingSpan./2));
[Kinkx,Kinkz] = Unconventional.automation.leading_offset(obj.SweepAngle,obj.DihedralAngle,obj.Kinky);


obj.Tipx = Tipx;
obj.Tipz = Tipz;

obj.KinkChord  = obj.RootChord - Kinkx;
obj.Kinkx = Kinkx;
obj.Kinkz = Kinkz;

% Control surfaces
obj.FlapStart    = 0.75;
obj.AileronStart = 0.75;

% ---------------------------
% Horizontal tail
% ---------------------------

obj.HStabStart = 60.0;

obj.HRootChord = 6.0;
obj.HTipChord  = 1.5;

obj.TailSpan = 25.0;
obj.HalfTailSpan = obj.TailSpan/2;

obj.HSweepAngle= 40; % Degrees
obj.HDihedralAngle= 2; % Degrees

% Wing geometery - X/Z offsets
[HTailx,HTailz] = Unconventional.automation.leading_offset(obj.HSweepAngle,obj.HDihedralAngle,(obj.TailSpan./2));

obj.HTipx = HTailx;
obj.HTipz = HTailz;

obj.ElevatorHinge = 0.7;

% ---------------------------
% Vertical tail
% ---------------------------

obj.VStabStart = 61;

obj.VRootChord = 5.0;
obj.VTipChord  = 1.8;

obj.VSweepAngle = 40;

[VTailx,VTailz] = Unconventional.automation.leading_offset(obj.VSweepAngle,0.0,(obj.TailSpan./2));

obj.VTailSpan = 16.0;
obj.VTipX = VTailx;

% Rudder hinge
obj.RudderHinge = 0.7;

% ---------------------------
% Airfoils
% ---------------------------

obj.HorAerofoil = 'naca0012.dat';
obj.VerAerofoil = 'naca0012.dat';

WingAirfoil = 'SC(2)-0610.dat';

% ---------------------------
% Flight condition
% ---------------------------

Mach = 0.78;
Alt  = 11000;

% ---------------------------
% Run AVL file generator
% ---------------------------

Unconventional.automation.construct_avl(obj, Mach, Alt, WingAirfoil);

disp('AVL geometry file generated: unconventional.avl')


% ------------------------------------------------
% Additional Aircraft Parameters (Drag + Performance)
% ------------------------------------------------

%% Aerodynamic properties

obj.tc       = 0.12;   % Wing thickness-to-chord ratio
obj.tc_ht    = 0.10;   % Horizontal tail thickness ratio
obj.tc_vt    = 0.10;   % Vertical tail thickness ratio

obj.CLmax_clean = 1.5; % Clean configuration
obj.CLmax_flap  = 2.6; % Landing configuration

%% Fuselage geometry

obj.FuselageLength   = 75;   % m
obj.FuselageDiameter = 7.0;  % m

%% Engine / nacelle parameters

obj.N_eng = 2;               % Number of engines

obj.NacelleLength   = 8.0;   % m
obj.NacelleDiameter = 3.5;   % m

%% Aircraft mass properties

obj.MTOM    = 490000;  % kg (Maximum takeoff mass)
obj.Payload = 85000;   % kg
obj.OEW     = 250000;  % kg

%% Derived aerodynamic quantities

obj.AR = obj.WingSpan^2 / obj.WingArea;         % Aspect ratio
obj.WingLoading = (obj.MTOM * 9.81) / obj.WingArea; % Wing loading (N/m^2)

%% Cruise condition

obj.CruiseMach = 0.78;
obj.CruiseAlt  = 11000;

%% Performance reference

obj.g = 9.81;  % Gravity

CD0 = Unconventional.aerodynamics.testdragbuildup(obj,11000,0.78)