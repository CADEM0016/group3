%% PAYLOAD RANGE DIAGRAM (FULL - WITH MZFW)

clear; clc; close all;

set(0,'DefaultFigureColor','w')
set(0,'DefaultAxesFontSize',18)
set(0,'DefaultTextFontSize',18)

%% -------------------- INPUTS --------------------


% Masses (tonnes)
MTOM = 560;
OEM  = 265;

maxPayload_struct = 138.5;
maxFuel           = MTOM*0.4;

% Convert to kg
MTOM = MTOM * 1e3;
OEM  = OEM  * 1e3;
maxPayload_struct = maxPayload_struct * 1e3;
maxFuel = maxFuel * 1e3;

% Performance assumptions
LD  = 16;
M_cruise   = 0.72;
alt = 11000;

% ---- create engines ----
eng_UF   = cast.eng.TurboFan.UF(1, alt, M_cruise);
eng_977B = cast.eng.TurboFan.T_977B(1, alt, M_cruise);

% ---- TSFC ----
TSFC_UF   = eng_UF.TSFC(M_cruise, alt);
TSFC_977B = eng_977B.TSFC(M_cruise, alt);

fprintf('\n--- CRUISE TSFC ---\n')
fprintf('UltraFan TSFC:   %.3e kg/N/s\n', TSFC_UF)
fprintf('Trent 977B TSFC: %.3e kg/N/s\n', TSFC_977B)

improvement = (TSFC_977B - TSFC_UF)/TSFC_977B * 100;
fprintf('TSFC reduction: %.1f %%\n', improvement)

% Atmosphere
T0 = 288.15;
a0 = 340;
a  = a0 * sqrt(1 - 0.0065*alt/T0);
V  = M_cruise * a;

%% -------------------- MASS LIMITS --------------------

usefulLoad = MTOM - OEM;
MZFW = OEM + maxPayload_struct;

%% -------------------- BREGUET FUNCTION --------------------

breguet = @(fuel, TSFC) (V/TSFC)*LD*log(MTOM/(MTOM - fuel)) / 1000;

%% -------------------- COMMON POINTS --------------------

R_A = 0;
P_A = maxPayload_struct;

fuel_B = MTOM - MZFW;
P_B = maxPayload_struct;

fuel_C = maxFuel;
P_C = usefulLoad - fuel_C;

Wi_C = MTOM;
Wf_C = MTOM - maxFuel;

Wi_D = OEM + maxFuel;
Wf_D = OEM;

P_D = 0;

%% -------------------- ULTRAFAN --------------------

R_B_UF = breguet(fuel_B, TSFC_UF);
R_C_UF = (V/TSFC_UF)*LD*log(Wi_C / Wf_C) / 1000;
R_D_UF = (V/TSFC_UF)*LD*log(Wi_D / Wf_D) / 1000;

ranges_UF = [R_A; R_B_UF; R_C_UF; R_D_UF];
payloads_UF = [P_A; P_B; P_C; P_D];

%% -------------------- 977B --------------------

R_B_977B = breguet(fuel_B, TSFC_977B);
R_C_977B = (V/TSFC_977B)*LD*log(Wi_C / Wf_C) / 1000;
R_D_977B = (V/TSFC_977B)*LD*log(Wi_D / Wf_D) / 1000;

ranges_977B = [R_A; R_B_977B; R_C_977B; R_D_977B];
payloads_977B = [P_A; P_B; P_C; P_D];

delta_range_pct = (R_C_UF - R_C_977B) / R_C_977B * 100;

%% -------------------- PLOT --------------------

figure; hold on; grid on

plot(ranges_UF, payloads_UF/1000, '-ob','LineWidth',2,'DisplayName','UltraFan')
plot(ranges_977B, payloads_977B/1000, '-or','LineWidth',2,'DisplayName','Trent 977B')

xlabel('Range (km)')
ylabel('Payload (tonnes)')
title('Payload–Range Diagram (UF vs 977B)')
legend show

xlim([0 max([ranges_UF; ranges_977B])*1.1])
ylim([0 max(payloads_UF/1000)*1.1])

%% -------------------- ROUTE PAYLOAD ANALYSIS --------------------

disp(' ')
disp('--- ROUTE PAYLOAD RESULTS ---')

for i = 1:length(dist_km)

    R_target = dist_km(i)*1000;

    % ---------- ULTRAFAN ----------
    payload_fun_UF = @(payload) ...
        MTOM - OEM - payload - ...
        MissionAnalysis(eng_UF, R_target, MTOM);

    payload_UF = fzero(payload_fun_UF, maxPayload_struct*0.8);
    payload_UF = min(payload_UF, maxPayload_struct);

    % ---------- 977B ----------
    payload_fun_977B = @(payload) ...
        MTOM - OEM - payload - ...
        MissionAnalysis(eng_977B, R_target, MTOM);

    payload_977B = fzero(payload_fun_977B, maxPayload_struct*0.8);
    payload_977B = min(payload_977B, maxPayload_struct);

    fprintf('\n%s (%.0f km)\n', AirportPairs(i), dist_km(i))
    fprintf('  UF Payload:   %.1f t\n', payload_UF/1000)
    fprintf('  977B Payload: %.1f t\n', payload_977B/1000)

end

%% -------------------- ULTRAFAN ROUTE PLOT --------------------

payloads_UF_routes = zeros(length(dist_km),1);

for i = 1:length(dist_km)

    R_target = dist_km(i)*1000;

    payload_fun_UF = @(payload) ...
        MTOM - OEM - payload - ...
        MissionAnalysis(eng_UF, R_target, MTOM);

    payload_UF = fzero(payload_fun_UF, maxPayload_struct*0.8);
    payload_UF = min(payload_UF, maxPayload_struct);

    payloads_UF_routes(i) = payload_UF;

end

figure; hold on; grid on

plot(ranges_UF, payloads_UF/1000, '-ob','LineWidth',2,'DisplayName','UltraFan Envelope')

plot(dist_km, payloads_UF_routes/1000, 'ks', ...
    'MarkerFaceColor','y','DisplayName','Routes')

xlabel('Range (km)')
ylabel('Payload (tonnes)')
title('UltraFan Payload–Range with Mission Routes')
legend show

xlim([0 max(ranges_UF)*1.1])
ylim([0 max(payloads_UF/1000)*1.1])