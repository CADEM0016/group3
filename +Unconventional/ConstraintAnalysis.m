function [ThrustToWeightRatio,WingLoading] = ConstraintAnalysis(obj)

%% estimate T/W and W/S from constraint analysis
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

TL = 100:1000;  % Wing Loading (W/S)

%% Take-Off Values
g = 1;         % Gravity
CL = 2.5;      % Coefficient of Lift  
Cd = 0.12;     % Coefficient of Drag
Q = 34.6;      % Dynamic pressure at VLOF
CLMax = 1.3;   % Max Coefficient of Lift
Sg = 2500;     % Ground Run (m)
u = 0.04;      % Ground Friction Constant

TOV = ((1.21/(g*Sg*CLMax*1.225)*TL) + ((1/2)*(Cd/CLMax))+((1/2)*u)) %Take Off Values

%% Climb Values
Vv = 15.6;     % Vertical Speed (m/s)
As = 126;      % Airspeed (m/s)
q = 1128;      % Dynamic Pressure at the Selected Airspeed and Altitude (N/M)

CV = (Vv/As) + ((q./TL)*0.0396) + ((0.0433/q)*TL);

%% Landing
Vapp = 74.6;    % Approach Velocity
Vs = 1.3*Vapp;  % Stl Velocity
Density = 1.225;% Density At Sea Level

L = ((1/2) * Density * (Vs^2) * CLMax) / 9.81; 

%% Plots
plot(TL , TOV,'b', TL,CV, 'm' , 'LineWidth', 2)
xline(L,'k', "LineWidth",3)
xlabel('Wing Loading')
ylabel('Thrust-to-Weight Ratio')
title("Constraint Analysis")

end