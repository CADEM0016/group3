function C = constants()
% All fixed numerical values for Raymer Section 15
% and unit conversions
% Imperial regression constants

% UNIT CONVERSIONS
C.ft  = 0.3048;       % m per ft
C.lb  = 0.453592;     % kg per lb

%% Flight Controls
C.FC.coeff      = 145.9;
C.FC.exp_Nf     = 0.554;
C.FC.exp_mech   = -1.0;
C.FC.exp_Scs    = 0.20;
C.FC.exp_Iyaw   = 0.07;
C.FC.Iyaw_scale = 1e-6;


%% Instruments
C.INST.coeff    = 4.509;
C.INST.exp_Nc   = 0.541;
C.INST.exp_geom = 0.5;

% Jet aircraft factors
C.INST.K_r  = 1.0;
C.INST.K_tp = 1.0;

%% Avionics

C.AV.coeff     = 1.73;
C.AV.exp_Wuav  = 0.983;

% SIMPLE GEOMETRY PLACEHOLDERS
% (for visual mass boxes only)
C.geom.fc_width   = 1.5;  % m
C.geom.inst_len   = 2.0;  % m
C.geom.av_len     = 2.5;  % m

end