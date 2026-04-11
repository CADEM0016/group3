function [ThrustToWeightRatio,WingLoading] = ConstraintAnalysis(obj)

%% estimate T/W and W/S from constraint analysis (Old Code)
% for now just setting to those of B777
% ---------------------- TODO -----------------------
% --------- update with constraint analysis ---------
% obj.ThrustToWeightRatio = 0.3; % ASSUMPTION: hardcoded for now based of CDR
% obj.WingLoading = 765*9.81; % ASSUMPTION: hardcoded for now based of CDR
% % obj.WingLoading = (347815*9.81)/436.8;
% 
% % set Wing Area and Thrust
% SweepQtrChord = real(acosd(0.75.*obj.Mstar./obj.TLAR.M_c)); % quarter chord sweep angle
% obj.WingArea = obj.MTOM*9.81/obj.WingLoading/cosd(SweepQtrChord);
% fprintf('this is constaint analysis WingArea = %g\n', obj.WingArea)
% obj.Thrust = obj.ThrustToWeightRatio * obj.MTOM * 9.81;


%% estimate T/W and W/S from constraint analysis

%TL = 100:1000;  % Wing Loading (W/S)
%% Take-Off Values
g = 1;         % Gravity
CL = 0.8;      % Coefficient of Lift  
Cd = 0.03;     % Coefficient of Drag
Q = 34.6;      % Dynamic pressure at VLOF
CLMax = 1.5;   % Max Coefficient of Lift
Sg = 2500;     % Ground Run (m)
u = 0.04;      % Ground Friction Constant

%TOV = ((1.21/(g*Sg*CLMax*1.225)*TL) + ((0.605/CLMax)*(Cd + (u*CL))) + u );  %Take Off Values
%% Climb Values
Vv = 15.6;     % Vertical Speed (m/s)
As = 128.61;   % Airspeed @ 250kts (m/s)
q = 1128;      % Dynamic Pressure at the Selected Airspeed and Altitude 20000ft (N/M)
CDmin = 0.02;   % Minimum Coefficient of Drag
AR = 10.5;      % Aspect Ratio
e = 0.8 ;      % Oswald Efficiency Factor
k= 1/(pi*e*AR); % Induced Drag Factor

%CV = (Vv/As) + ((q./TL)*CDmin) + ((k/q)*TL); % Climb Values

%% Landing Values
Vapp = 74.6;    % Approach Velocity
Vs = 1.3*Vapp;  % Stl Velocity
Density = 1.225;% Density At Sea Level

L = ((1/2) * Density * (Vs^2) * CLMax)/9.81; % Landing Values

%% Sustained Turn
n = 1/(cos(30*pi/180)); % Bank Angle
qt = 3407; % Dynamic Pressure at the Selected Airspeed and Altitude 10000ft

%ST = qt*((CDmin./TL) + k*(((n/qt)^2)*TL)); % Sustained Turn Values
%% Plots
<<<<<<< Updated upstream
%plot(TL , TOV, TL,CV, 'm' , TL, ST, 'c',   'LineWidth', 2)
%xline(L,'k', "LineWidth",3)
%xlabel('Wing Loading (KG/M^2)')
%ylabel('Thrust-to-Weight Ratio')
%title("Constraint Analysis")
=======
plot(TL , TOV,'b', TL,CV, 'm' , 'LineWidth', 2)
xline(L,'k', "LineWidth",3)
xlabel('Wing Loading')
ylabel('Thrust-to-Weight Ratio')
title("Constraint Analysis")

%% PLEASE CHANGE THIS KAMRAN - OSCAR
ThrustToWeightRatio = 0.3;
WingLoading = 765*9.81;
obj.ThrustToWeightRatio = ThrustToWeightRatio; % ASSUMPTION: hardcoded for now based of CDR
obj.WingLoading = WingLoading; % ASSUMPTION: hardcoded for now based of CDR

% set Wing Area and Thrust
SweepQtrChord = real(acosd(0.75.*obj.Mstar./obj.TLAR.M_c)); % quarter chord sweep angle
obj.WingArea = obj.MTOM*9.81/obj.WingLoading/cosd(SweepQtrChord);
fprintf('this is constaint analysis WingArea = %g\n', obj.WingArea)
obj.Thrust = obj.ThrustToWeightRatio * obj.MTOM * 9.81;

end
>>>>>>> Stashed changes
