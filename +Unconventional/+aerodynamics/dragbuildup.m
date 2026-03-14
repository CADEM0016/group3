function [] = dragbuildup(obj,alt,flap_d)
% Function for finding the CD0's of the flight
% altitude is given as an input parameter so that this can 
% be found at every stage of the mission
% obj: is the ADP object
% Alt is the target altitude
% flaps is the flap deflection in deg

if nargin < 3
    flap_d = 0;
end

%% Atmospheric conditions
[rho,a,mu] = cast.atmos(alt); 
V = alt * a;

% WING
Re_w = reynolds(V,obj.c_ac,alt);
% Friction coeff. - Simple Prandtl (instructed to do this by snorri)
Cf_w = 0.455 / (log10(Re_w)^2.58); % Prantl turbulent flat plate approximation (NOTE: valid 5 x10^5 < Re < 10^9
% Wetted area
Swet_w = 2 * obj.WingArea; % Exposed surface area
% Form factor - Shevelle accounts for compressible + sweep effects 
term1 = (2 - M.^2) .* cos(Lambda_c4);
term2 = sqrt(1 - M.^2 .* cos(Lambda_c4).^2);
FF_w = 1 + (term1 ./ term2) .* tc + 100 .* tc.^4;
IF_w = 1.0;
Drag.CD_wing = Cf_w * FF_w * IF_w * (Swet_w / obj.WingArea);

% FUSELAGE
L_f = obj.FuselageLength;
D_f = obj.FuselageDiameter;
Re_f = rho * V * L_f / mu;
Cf_f = 0.455 / (log10(Re_f)^2.58);
Swet_f = pi * D_f * L_f;
f = L_f / D_f;
FF_f = 2.939 - 0.7666 * f + 0.1328 * f.^2 - 0.01074 * f.^3 + 3.275e-4 * f.^4;
IF_f = 1.0;
Drag.CD_fuselage = Cf_f * FF_f * IF_f * (Swet_f / obj.WingArea);

% HORIZONTAL TAIL
L_ht = obj.c_ht;
Re_ht = rho * V * L_ht / mu;
Cf_ht = 0.455 / (log10(Re_ht)^2.58);
tc_ht = obj.tc_ht;
Swet_ht = 2 * obj.HtpArea * (1 + 0.25 * tc_ht);
FF_ht = 1 + (0.6/0.3)*tc_ht + 100*tc_ht^4;
IF_ht = 1.05;
Drag.CD_ht = Cf_ht * FF_ht * IF_ht * (Swet_ht / obj.WingArea);

% VERTICAL TAIL
L_vt = obj.c_vt;
Re_vt = rho * V * L_vt / mu;
Cf_vt = 0.455 / (log10(Re_vt)^2.58);
tc_vt = obj.tc_vt;
Swet_vt = 2 * obj.VtpArea * (1 + 0.25 * tc_vt);
FF_vt = 1 + (0.6/0.3)*tc_vt + 100*tc_vt^4;
Q_vt = 1.05;
Drag.CD_vt = Cf_vt * FF_vt * Q_vt * (Swet_vt / obj.WingArea);

% NACELLES
L_n = obj.NacelleLength;
D_n = obj.NacelleDiameter;
Re_n = rho * V * L_n / mu;
Cf_n = 0.455 / (log10(Re_n)^2.58);
Swet_n = pi * D_n * L_n;
FF_n = 1.5;
Q_n = 1.3;
Drag.CD_nacelle = Cf_n * FF_n * Q_n * (Swet_n / obj.WingArea) * obj.N_eng;

% LANDING GEAR (retracted estimate)
Drag.CD_lg = 0.002;

% MISCELLANEOUS EXCRESCENCE DRAG
Drag.CD_misc = 0.001;

% TOTAL PARASITE DRAG
ADP.AeroPolar.CD0 = Drag.CD_wing + ...
           Drag.CD_fuselage + ...
           Drag.CD_ht + ...
           Drag.CD_vt + ...
           Drag.CD_nacelle + ...
           Drag.CD_lg + ...
           Drag.CD_misc;

end