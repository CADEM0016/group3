function [GeomObj,massObj] = avionics(obj)
% controls_systems:
% - instruments
% - avionics 

% NOTE: All Raymer equations assume imperial units

c = constants();
tlar = cast.TLAR.Unconventional;

% --------------------------- Create Geometry ----------------------------
% Instruments
% Cockpit ish area

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

% ------------------------- Create Mass Objects --------------------------
% Instruments
W_inst_lb = 4.509*c.K_r*c.K_tp*tlar.Crew^0.541*c.N_en*(obj.FuselageLength + obj.Span)^0.5;

% Avionics
W_av_lb = 1.73 * c.W_uav^0.983;

% Convert lb to kg
W_inst = W_inst_lb * 1/SI.lb;
W_av   = W_av_lb   * 1/SI.lb;

massObj(end+1) = cast.MassObj(Name="Instruments",m=W_inst,X=offsetInst);
massObj(end+1) = cast.MassObj(Name="Avionics", m=W_av, X=offsetAv);

end