function [GeomObj,massObj] = controlsSystems(obj)
% controls_systems:
% - flight controls,
% - instruments
% - avionics for widebody freighter

% All Raymer equations assume imperial units

c = constants();

%% Raymer Mass Equations

% Flight Controls
W_fc_lb = 145.9 *c.N_f^0.554 *(1 + c.N_m / c.N_f)^(-1.0)*c.S_cs^0.20 *(c.I_yaw * 1e-6)^0.07;

% Instruments
W_inst_lb = 4.509*c.K_r*c.K_tp*c.N_c^0.541*c.N_en*(c.L_f + c.B_w)^0.5;

% Avionics
W_av_lb = 1.73 * c.W_uav^0.983;

% Convert lb to kg
W_fc   = W_fc_lb   * 0.453592;
W_inst = W_inst_lb * 0.453592;
W_av   = W_av_lb   * 0.453592;

%% Geometry

% Flight Controls
% Represent as wing-spanning box near 30% MAC

fcLength = obj.b;          % spanwise distribution
fcWidth  = 1.5;            % arbitrary small chordwise depth

Xs = [-0.5  0.5;
       0.5  0.5;
       0.5 -0.5;
      -0.5 -0.5];
Xs_fc = Xs .* [fcWidth, fcLength];
offsetFC = [obj.x_ac - 0.1*obj.c_ac, 0];
GeomObj = cast.GeomObj(Name="FlightControls",Xs=Xs_fc+offsetFC);

% Instruments
% Cockpit block

instLength = 2.0;
instWidth  = obj.CabinRadius*1.2;
Xs_inst = Xs .* [instLength, instWidth];
offsetInst = [obj.x_nose + instLength/2, 0];
GeomObj(end+1) = cast.GeomObj(Name="Instruments",Xs=Xs_inst+offsetInst);

% Avionics
% Electronics bay under cockpit floor

avLength = 2.5;
avWidth  = obj.CabinRadius*0.8;
Xs_av = Xs .* [avLength, avWidth];
offsetAv = [obj.x_nose + 3.0, 0];
GeomObj(end+1) = cast.GeomObj(Name="Avionics",Xs=Xs_av+offsetAv);

%% Mass Objects

massObj = cast.MassObj(Name="Flight Controls",m=W_fc,X=offsetFC);
massObj(end+1) = cast.MassObj(Name="Instruments",m=W_inst,X=offsetInst);
massObj(end+1) = cast.MassObj(Name="Avionics", m=W_av, X=offsetAv);

end