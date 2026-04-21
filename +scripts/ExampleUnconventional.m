%% Size an Unconventional at a Mach number of 0.84

% Instantiate an instance of the Unconventional class add define some initial
% parameters
ADP0 = Unconventional.ADP();
ADP0.TLAR = cast.TLAR.Unconventional(); % sets top level aircraft requirements
ADP0.TLAR.M_c = 0.84;
% Unconventional.aerodynamics.high_lift(ADP)

% --------------------- set Unconventional specific parameters ---------------------
ADP0.FuselageLength = 65; % Total fuselage length (m)
ADP0.KinkPos = 10;       % spanwise position of TE kink in wing planform
ADP0.CabinRadius = 6.3;
ADP0.CabinLength = 50;
ADP0.CockpitLength = 5;
ADP0.WingPos = 0.44*ADP0.FuselageLength; % normalised wing position (% of fuselage length)
ADP0.V_HT = 0.97; % horizontal tail volume coefficent
ADP0.V_VT = 0.072; % vertical tail volume coefficent
ADP0.HtpPos = 0.85*ADP0.FuselageLength; % normalised HTP position (% of fuselage length)
ADP0.VtpPos = 0.82*ADP0.FuselageLength; % normalised VTP position (% of fuselage length)

ADP0.WingArea = 750; % m^2 guess

% ------------------------- set Hyper-parameters -------------------------
ADP0.Span = 74; % Gate code + Folding tips
% ADP0.FleetSize = 6;

% -------------------------- class-I estimates ---------------------------
% initial mission analysis to estimate MTOM

ADP0.MTOM = 490000; % VERY basic guess of MTOM from payload

% initial estimate of fuel mass ( % of MTOM)
ADP0.Mf_Fuel = 0.32; % maximum fuel mass
ADP0.Mf_res  = 0.03; % reserve fuel mass

% initial estimate of mass fractions at important flight phases
ADP0.Mf_Ldg = 0.62;   % maximum landing mass
ADP0.Mf_TOC = 0.975;  % mass at the Top of Climb (TOC)

% -------------------------------- Sizing --------------------------------
% Note - see the "size" function at the bottom of this script
ADP = Unconventional.Size(ADP0);

%% build the "Sized" geometry and plot it
[B7Geom,B7Mass] = Unconventional.BuildGeometry(ADP);

f = figure(1);
clf

ax = axes(f);
hold(ax,'on')
axis(ax,'equal')
set(ax,'YDir','normal')

% ------------------- background PNG FIRST -------------------
%img = imread("C:\Users\OscarAntill\OneDrive - University of Bristol\group3\B777F_planform.png");   % use your new image
% 
% % set image extent to match your aircraft drawing coordinates
% % adjust these numbers if needed to line up perfectly
% xImg = [0 80];
% yImg = [-40 40];
% 
% hImg = image(ax, ...
%     'XData', xImg, ...
%     'YData', yImg, ...
%     'CData', img);
% 
% set(hImg,'AlphaData',0.45)   % transparency
% uistack(hImg,'bottom')       % keep image behind geometry
% 
% % ------------------- draw geometry on top -------------------
% cast.draw(B7Geom,B7Mass,[0,0])
% 
% ax.XAxis.Visible = "on";
% ax.YAxis.Visible = "on";
% xlim(ax, xImg)
% ylim(ax, yImg)
% 
% exportgraphics(ax,'B777F_overlay.png','Resolution',300)

% print some key data points
%d = B7Mass.GetData;
% fprintf('MTOM: %0.0f t, Fuel Mass: %0.0f t, Wing Mass %0.0f t\n',ADP.MTOM/1e3,ADP.Mf_Fuel*ADP.MTOM/1e3,double(d(strcmp(d(:,1),"Wing"),2)));
% fprintf('CD0: %0.3f, CD (CL=0.5): %0.3f \n',ADP.AeroPolar.CD(0),ADP.AeroPolar.CD(0.5));

%% Example call to mission analysis discipline
%[BlockFuel,TripFuel,ResFuel,Mf_TOC,MissionTime] = Unconventional.MissionAnalysis_oscar(ADP, ADP.TLAR.RangeDes, ADP.MTOM);
%[BlockFuel,TripFuel,ResFuel,Mf_TOC,MissionTime,cruise_FL] = Unconventional.MissionAnalysis_PhysicsFinal(ADP, ADP.TLAR.RangeDes, ADP.MTOM);
[BlockFuel,TripFuel,ResFuel,Mf_TOC,MissionTime,cruise_FL] = Unconventional.MissionAnalysis_PhysicsFinal(ADP, ADP.TLAR.RangeDes, ADP.MTOM);

%% Example Trade study, comparing MTOM and Block Fuel as a function of wing span
%predefine spans to test
Spans = 50:1:100;

% pre-allocate arrays for results
mtoms = zeros(size(Spans));
fuels = zeros(size(Spans));

% Use the converged baseline aircraft as the starting point for each case,
% then re-size to convergence at each span
% for i = 1:length(Spans)
%     ADPi = ADP;                 % start from converged baseline aircraft
%     ADPi.Span = Spans(i);       % apply new span
%     ADPi = Unconventional.Size(ADPi);   % re-converge aircraft at this span
% 
%     mtoms(i) = ADPi.MTOM;
%     fuels(i) = ADPi.Mf_Fuel * ADPi.MTOM;
% end

% out = Unconventional.plotCGbubble(ADP0, ...
%     FuelFractions = 0:0.1:1, ...
%     PalletCounts = 0:45, ...
%     CargoStart = 7);

f = figure(2);
clf;
tt = tiledlayout(2,1);

nexttile(1);
plot(Spans, mtoms/1e3, '-s')
xlabel('Span [m]')
ylabel('MTOM [t]')

nexttile(2);
plot(Spans, fuels/1e3, '-o')
xlabel('Span [m]')
ylabel('Block Fuel [t]')

%% Sizing Function