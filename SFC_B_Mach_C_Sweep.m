%% B vs Mach using BPR-derived SFC (UltraFan + T977B)



% -----------------------------
% ALTITUDE
% -----------------------------
alt_SM = 35e3 ./ SI.ft;
T_SM  = cast.atmosT(alt_SM);
T0_SM = cast.atmosT(0);

theta_SM = sqrt(T_SM/T0_SM);

% -----------------------------
% MACH SWEEP
% -----------------------------
M_SM = 0.6:0.01:0.9;

% =============================
% ULTRAFAN
% =============================
BPR_UF_SM = (15 + 12) * 0.5;

A_UF_SM = 19 * exp(-0.12 * BPR_UF_SM) * 1e-6;
SFCc_UF_SM = 25 * exp(-0.05 * BPR_UF_SM) * 1e-6;

B_UF_SM_SM = (SFCc_UF_SM / theta_SM - A_UF_SM) ./ M_SM;

DesRange = 10888000;

a = sqrt(1.4 * 287 * T_SM);

V = (M_SM.*a);

t_flight_UF = DesRange./V;

T_static_UF = 4.118204039170605e+05;


T_cruise_UF = 0.35 * T_static_UF^0.9 * exp(0.02 * BPR_UF_SM);


% Fuel flow
mdot_UF = B_UF_SM_SM .* T_cruise_UF;

% Block fuel (cruise-only!)
Fuel_UF = mdot_UF .* t_flight_UF;


% =============================
% T977B
% =============================
BPR_T_SM = 8.5;

A_T_SM = 19 * exp(-0.12 * BPR_T_SM) * 1e-6;
SFCc_T_SM = 25 * exp(-0.05 * BPR_T_SM) * 1e-6;

B_T_SM = (SFCc_T_SM / theta_SM - A_T_SM) ./ M_SM;

% -----------------------------
% PLOT (REPORT READY)
% -----------------------------



figure('Color','w')

plot(M_SM, Fuel_UF/1000, 'b', 'LineWidth', 2)

grid on
box on

xlabel('Cruise Mach', 'FontSize', 20)
ylabel('Block Fuel (tonnes)', 'FontSize', 20)
title('Block Fuel vs Cruise Mach (UltraFan)', 'FontSize', 24)

set(gca, 'FontSize', 20)