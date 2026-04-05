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

%% Example Trade study, comparing MTOM and Block Fuel as a function of wing span
% predefine spans to test
Spans = 50:5:100;

% pre-allocate arrays for results
mtoms = zeros(size(Spans));
fuels = zeros(size(Spans));

% loop over spans and size aircraft for each span
for i = 1:length(Spans)
    ADP = ADP0;              % reset to baseline each time
    ADP.Span = Spans(i);
    ADP = Unconventional.Size(ADP);

    mtoms(i) = ADP.MTOM;
    fuels(i) = ADP.Mf_Fuel * ADP.MTOM;
end

f = figure(2);
clf;
tt = tiledlayout(2,1);

nexttile(1);
plot(Spans,mtoms/1e3,'-s')
xlabel('Span [m]')
ylabel('MTOM [t]')

nexttile(2);
plot(Spans,fuels/1e3,'-o')
xlabel('Span [m]')
ylabel('Block Fuel [t]')

%% Sizing Function