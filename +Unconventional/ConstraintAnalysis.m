function [ThrustToWeightRatio,WingLoading] = ConstraintAnalysis(obj)

%% estimate T/W and W/S from constraint analysis
% for now just setting to those of B777
% ---------------------- TODO -----------------------
% --------- update with constraint analysis ---------
obj.ThrustToWeightRatio = 0.3; % ASSUMPTION: hardcoded for now based of CDR
obj.WingLoading = 765*9.81; % ASSUMPTION: hardcoded for now based of CDR
% obj.WingLoading = (347815*9.81)/436.8;

% set Wing Area and Thrust
SweepQtrChord = real(acosd(0.75.*obj.Mstar./obj.TLAR.M_c)); % quarter chord sweep angle
obj.WingArea = obj.MTOM*9.81/obj.WingLoading/cosd(SweepQtrChord);
fprintf('this is constaint analysis WingArea = %g\n', obj.WingArea)
obj.Thrust = obj.ThrustToWeightRatio * obj.MTOM * 9.81;
end