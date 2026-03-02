function C = constants()
% All fixed numerical values for Raymer Section 15
% and unit conversions
% Imperial regression constants


%% General
C.N_en = 2; % Number of Engines

%% Flight Controls
C.FC.coeff      = 145.9;
C.N_f = 6; % Guess at number of functions aileron + rudder etc.
C.FC.Iyaw_scale = 1e-6;


%% Instruments
% Jet aircraft factors
C.INST.K_r  = 1.0; % Non recieprocating engine
C.INST.K_tp = 1.0; % Non turbo prop


%% Avionics
C.W_uav = 1400; % lbs Raymer estimate uninstalled avionics 

% SIMPLE GEOMETRY PLACEHOLDERS
% (for visual mass boxes only)
C.geom.fc_width   = 1.5;  % m
C.geom.inst_len   = 2.0;  % m
C.geom.av_len     = 2.5;  % m

end