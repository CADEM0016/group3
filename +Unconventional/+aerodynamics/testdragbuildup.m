function [CD0] = dragbuildup(obj,alt,Mach)

if nargin < 3
    Mach = 0.78;
end

%% Atmospheric conditions
[rho,a,~,~,nu] = cast.atmos(alt);
V = Mach * a;

%% -------------------------
% WING
%% -------------------------

Re_w = V * obj.CMAC / nu;

Cf_w = 0.455 / (log10(Re_w)^2.58);

Swet_w = 2 * obj.WingArea;

tc = 0.12; % assumed thickness ratio

Lambda = deg2rad(obj.SweepAngle);

term1 = (2 - Mach^2) * cos(Lambda);
term2 = sqrt(1 - Mach^2 * cos(Lambda)^2);

FF_w = 1 + (term1/term2)*tc + 100*tc^4;

Drag.CD_wing = Cf_w * FF_w * (Swet_w / obj.WingArea);

%% -------------------------
% FUSELAGE
%% -------------------------

L_f = obj.WingStart * 2;     % rough estimate
D_f = obj.RootChord / 3;     % rough estimate

Re_f = V * L_f / nu;

Cf_f = 0.455 / (log10(Re_f)^2.58);

Swet_f = pi * D_f * L_f;

f = L_f / D_f;

FF_f = 2.939 - 0.7666*f + 0.1328*f^2 - 0.01074*f^3 + 3.275e-4*f^4;

Drag.CD_fuselage = Cf_f * FF_f * (Swet_f / obj.WingArea);

%% -------------------------
% HORIZONTAL TAIL
%% -------------------------

HtpArea = obj.TailSpan * (obj.HRootChord + obj.HTipChord) / 2;

Re_ht = V * obj.HRootChord / nu;

Cf_ht = 0.455 / (log10(Re_ht)^2.58);

tc_ht = 0.10;

Swet_ht = 2 * HtpArea * (1 + 0.25 * tc_ht);

FF_ht = 1 + 2*tc_ht + 100*tc_ht^4;

Drag.CD_ht = Cf_ht * FF_ht * (Swet_ht / obj.WingArea);

%% -------------------------
% VERTICAL TAIL
%% -------------------------

VtpArea = obj.VTailSpan * (obj.VRootChord + obj.VTipChord) / 2;

Re_vt = V * obj.VRootChord / nu;

Cf_vt = 0.455 / (log10(Re_vt)^2.58);

tc_vt = 0.10;

Swet_vt = 2 * VtpArea * (1 + 0.25 * tc_vt);

FF_vt = 1 + 2*tc_vt + 100*tc_vt^4;

Drag.CD_vt = Cf_vt * FF_vt * (Swet_vt / obj.WingArea);

%% -------------------------
% NACELLES (simple estimate)
%% -------------------------

Drag.CD_nacelle = 0.002;

%% -------------------------
% LANDING GEAR
%% -------------------------

Drag.CD_lg = 0.002;

%% -------------------------
% MISC
%% -------------------------

Drag.CD_misc = 0.001;

%% -------------------------
% TOTAL PARASITE DRAG
%% -------------------------


Drag.CD_wing
Drag.CD_fuselage
Drag.CD_ht
Drag.CD_vt
Drag.CD_nacelle
Drag.CD_lg
Drag.CD_misc

disp(['Estimated CDwing = ', num2str(Drag.CD_wing)])
disp(['Estimated CDfuselage = ', num2str(Drag.CD_fuselage)])
disp(['Estimated CDht = ', num2str(Drag.CD_ht)])
disp(['Estimated CDvt = ', num2str(Drag.CD_vt)])
disp(['Estimated CDnacelle = ', num2str(Drag.CD_nacelle)])
disp(['Estimated CDlg = ', num2str(Drag.CD_lg)])
disp(['Estimated CDmisc = ', num2str(Drag.CD_misc)])


CD0 = Drag.CD_wing + ...
      Drag.CD_fuselage + ...
      Drag.CD_ht + ...
      Drag.CD_vt + ...
      Drag.CD_nacelle + ...
      Drag.CD_lg + ...
      Drag.CD_misc;

disp(['Estimated CD0 = ', num2str(CD0)])

end