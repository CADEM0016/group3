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
figure('Color','w')   % white background

plot(M_SM, B_UF_SM_SM, 'b', 'LineWidth', 2); hold on
plot(M_SM, B_T_SM, 'r', 'LineWidth', 2);

grid on
box on

xlabel('Cruise Mach', 'FontSize', 20)
ylabel('SFC', 'FontSize', 20)
title('SFC vs Cruise Mach (BPR-derived at Cruise Altitude)', 'FontSize', 24)

legend('UltraFan','T977B', 'FontSize', 18, 'Location','northeast')

set(gca, 'FontSize', 20)   % axis tick font