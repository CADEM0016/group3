% Size an Unconventional at a Mach number of 0.84

% Instantiate an instance of the Unconventional class add define some initial
% parameters
%<<<<<<< Updated upstream
ADP0 = Unconventional.ADP();
ADP0.TLAR = cast.TLAR.Unconventional(); % sets top level aircraft requirements
ADP0.TLAR.M_c = 0.84;
% Unconventional.aerodynamics.high_lift(ADP)
%=======
%ADP = Unconventional.ADP();
%ADP.TLAR = cast.TLAR.Unconventional(); % sets top level aircraft requirements
%ADP.TLAR.M_c = 0.84;
%Unconventional.aerodynamics.high_lift(ADP)
%>>>>>>> Stashed changes

% --------------------- set Unconventional specific parameters ---------------------
ADP0.FuselageLength = 65; % Total fuselage length (m)
ADP0.KinkPos = 10;       % spanwise position of TE kink in wing planform
ADP0.CabinRadius = 6.3;
ADP0.CabinLength = 50;
ADP0.CockpitLength = 5;
ADP0.WingPos = 0.44*ADP0.FuselageLength; % normalised wing position (% of fuselage length)
ADP0.V_HT = 0.97; % horizontal tail volume coefficent
ADP0.V_VT = 0.072; % vertical tail volume coefficent
ADP0.HtpPos = 0.85*ADP0.FuselageLength;% normalised HTP position (% of fuselage length)
ADP0.VtpPos = 0.82*ADP0.FuselageLength;% normalised VTP position (% of fuselage length)

ADP0.WingArea = 750; % m^2 guess

% ------------------------- set Hyper-parameters -------------------------
ADP0.Span = 74; % Gate code + Folding tips
% ADP0.FleetSize = 6;

% -------------------------- class-I estimates ---------------------------
% initial mission analysis to estimate MTOM
ADP0.MTOM = 490000; % VERY basic guess of MTOM from payload

% initial estimate of fuel mass ( % of MTOM)
ADP0.Mf_Fuel = 0.32; % maximum fuel mass
ADP0.Mf_res = 0.03;  % reserve fuel mass

% initial estimate of mass fractions at important flight phases
ADP0.Mf_Ldg = 0.62;  % maximum landing mass
ADP0.Mf_TOC = 0.975;  % mass at the Top of Climb (TOC)

% -------------------------------- Sizing --------------------------------
% Note - see the "size" function at the bottum of this script
ADP = Unconventional.Size(ADP0);

%% build the "Sized" geometry and plot it
[B7Geom,B7Mass] = Unconventional.BuildGeometry(ADP); % get list of components geometries and masses

% plot the geometry (ontop of an image of a B777F for reference)
f = figure(1);
clf;
img = imread('B777F_planform.png'); 
imshow(img, 'XData', [0 63.7], 'YData', [-64.8 64.8]/2); 

cast.draw(B7Geom,B7Mass,[0,0]) % CHANGE
ax = gca;
ax.XAxis.Visible = "on";
ax.YAxis.Visible = "on";
axis equal
ylim([-0.5 0.5]*ADP0.Span)

% print some key data points
d = B7Mass.GetData;
% fprintf('MTOM: %0.0f t, Fuel Mass: %0.0f t, Wing Mass %0.0f t\n',ADP0.MTOM/1e3,ADP0.Mf_Fuel*ADP0.MTOM/1e3,double(d(strcmp(d(:,1),"Wing"),2)));
% fprintf('CD0: %0.3f, CD (CL=0.5): %0.3f \n',ADP0.AeroPolar.CD(0),ADP0.AeroPolar.CD(0.5));

%% Example call to mission analysis discipline
[BlockFuel,TripFuel,ResFuel,Mf_TOC,MissionTime] = Unconventional.MissionAnalysis_oscar(ADP,ADP0.TLAR.RangeDes, ADP0.MTOM);
%[BlockFuel,TripFuel,ResFuel,Mf_TOC,MissionTime] = Unconventional.MissionAnalysis_PhysicsV1(ADP,ADP0.TLAR.RangeDes, ADP0.MTOM);
%[BlockFuel, TripFuel, ResFuel, Mf_TOC, MissionTime, Mission, CriticalTW, CriticalWS] = Unconventional.physics_model_v1(ADP,ADP0.TLAR.RangeDes, ADP0.MTOM);

%% Example Trade study, comparing MTOM and Block Fuel as a function of wing span
% predefine spans to test
%Spans = 50:5:100;
M_c = 0.6:0.01:0.9;


% pre-allocate arrays for results
mtoms = zeros(size(M_c));
fuels = zeros(size(M_c));

% loop over M_c and size aircraft for each span
for i = 1:length(M_c)
    ADP = ADP0;              % reset to baseline each time
    ADP.TLAR.M_c = M_c(i);
    ADP = Unconventional.Size(ADP);

    mtoms(i) = ADP.MTOM;
    fuels(i) = ADP.Mf_Fuel * ADP.MTOM;
end

f = figure(2);
clf;
tt = tiledlayout(2,1);

nexttile(1);
plot(M_c,mtoms/1e3,'-s')
xlabel('M_c [m]')
ylabel('MTOM [t]')

nexttile(2);
plot(M_c,fuels/1e3,'-o')
xlabel('M_c [m]')
ylabel('Block Fuel [t]')

%% Sizing Function


%% B vs Mach using BPR-derived SFC (UltraFan + T977B)


%% B vs Mach using BPR-derived SFC (UltraFan + T977B)


% --- Load aero lookup table ---
baseDir = fileparts(matlab.desktop.editor.getActiveFilename);
lookupPath = fullfile(baseDir, '../+Unconventional/+lookup/+aerodynamics/cruise_lookup_table.mat');
S = load(lookupPath);
R = S.Results;


% -----------------------------
% ALTITUDE
% -----------------------------
alt_SM = 35e3 ./ SI.ft;
T_SM  = cast.atmosT(alt_SM);
T0_SM = cast.atmosT(0);

theta_SM = sqrt(T_SM/T0_SM);

% -----------------------------
% MACH SWEEP
% -----------------------------
M_SM = 0.6:0.01:0.9;

% =============================
% ULTRAFAN
% =============================
BPR_UF_SM = (15 + 12) * 0.5;

A_UF_SM = 19 * exp(-0.12 * BPR_UF_SM) * 1e-6;
SFCc_UF_SM = 25 * exp(-0.05 * BPR_UF_SM) * 1e-6;

B_UF_SM_SM = (SFCc_UF_SM / theta_SM - A_UF_SM) ./ M_SM;

DesRange = 10888000;

a = sqrt(1.4 * 287 * T_SM);

V = (M_SM.*a);

t_flight_UF = DesRange./V;

T_static_UF = 4.118204039170605e+05;


T_cruise_UF = 0.35 * T_static_UF^0.9 * exp(0.02 * BPR_UF_SM);


% Fuel flow
mdot_UF = B_UF_SM_SM .* T_cruise_UF;

% Block fuel (cruise-only!)
Fuel_UF = mdot_UF .* t_flight_UF;


% =============================
% T977B
% =============================
BPR_T_SM = 8.5;

A_T_SM = 19 * exp(-0.12 * BPR_T_SM) * 1e-6;
SFCc_T_SM = 25 * exp(-0.05 * BPR_T_SM) * 1e-6;

B_T_SM = (SFCc_T_SM / theta_SM - A_T_SM) ./ M_SM;

% -----------------------------
% PLOT (REPORT READY)
% -----------------------------



figure('Color','w')

plot(M_SM, Fuel_UF/1000, 'b', 'LineWidth', 2)

grid on
box on

xlabel('Cruise Mach', 'FontSize', 20)
ylabel('Block Fuel (tonnes)', 'FontSize', 20)
title('Block Fuel vs Cruise Mach (UltraFan)', 'FontSize', 24)

set(gca, 'FontSize', 20)



function plotOverlay_Fuel(M_c, fuels, M_SM, Fuel_UF)

    % Ensure same Mach grid (important)
    if length(M_c) ~= length(M_SM)
        error('Mach vectors must match')
    end

    figure('Color','w'); hold on

    % Convert to tonnes
    fuel_sizing = fuels / 1e3;
    fuel_sfc    = Fuel_UF / 1e3;

    % Plot BOTH on same axis
    plot(M_c, fuel_sizing, 'k-', 'LineWidth', 2)
    plot(M_SM, fuel_sfc,   'b--', 'LineWidth', 2)

    grid on
    box on

    xlabel('Cruise Mach', 'FontSize', 20)
    ylabel('Block Fuel [t]', 'FontSize', 20)

    title('Block Fuel vs Mach (Model Comparison)', 'FontSize', 24)

    legend('Sizing Tool (Mission Analysis)', ...
           'SFC Model (UltraFan)', ...
           'FontSize', 18, 'Location','best')

    set(gca, 'FontSize', 20)

end

plotOverlay_Fuel(M_c, fuels, M_SM, Fuel_UF)