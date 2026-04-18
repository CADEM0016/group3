%% PAYLOAD RANGE DIAGRAM (FULL - WITH MZFW)

clear; clc; close all;

set(0,'DefaultFigureColor','w')          % all figures white
set(0,'DefaultAxesFontSize',18)          % all axis ticks
set(0,'DefaultTextFontSize',18)          % general text

%% -------------------- INPUTS --------------------

%
%Block fuel conversion
%


t_flight = 4.550462799885724e+04;
T_static = 4.118204039170605e+05;

BPR = ((15+12)*0.5);   % <-- CHANGE THIS (UltraFan ~12-15, T977B ~8.5)

T_cruise = 0.35 * T_static^0.9 * exp(0.02 * BPR);



%



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

breguet = @(fuel, TSFC) (V/TSFC)*LD*log(MTOM/(MTOM - fuel));

m2km = @(x) x/1000;

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
R_C_UF = (V/TSFC_UF)*LD*log(Wi_C / Wf_C);
R_D_UF = (V/TSFC_UF)*LD*log(Wi_D / Wf_D);


ranges_UF = [R_A; R_B_UF; R_C_UF; R_D_UF];
payloads_UF = [P_A; P_B; P_C; P_D];

%% -------------------- 977B --------------------

R_B_977B = breguet(fuel_B, TSFC_977B);
R_C_977B = (V/TSFC_977B)*LD*log(Wi_C / Wf_C);
R_D_977B = (V/TSFC_977B)*LD*log(Wi_D / Wf_D);

ranges_977B = [R_A; R_B_977B; R_C_977B; R_D_977B];
payloads_977B = [P_A; P_B; P_C; P_D];


delta_range_pct = (R_C_UF - R_C_977B) / R_C_977B * 100;

%% -------------------- PLOT --------------------

figure; hold on; grid on

plot(ranges_UF/1000, payloads_UF/1000, '-ob','LineWidth',2,'DisplayName','UltraFan')
plot(ranges_977B/1000, payloads_977B/1000, '-or','LineWidth',2,'DisplayName','Trent 977B')



xlabel('Range (km)')
ylabel('Payload (tonnes)')
title('Payload–Range Diagram (UF vs 977B)')
legend show


xlim([0 max([ranges_UF; ranges_977B])*1.5])
ylim([0 max(payloads_UF/1000)*1.5])

x_mid = (R_C_UF + R_C_977B)/2;
y_mid = P_C/1000 * 0.4;


%% -------------------- RANGE IMPROVEMENT (TEXT ONLY) --------------------

% --- choose a clean payload level
P_arrow = (P_C/1000) + 5;

% --- faint guide line (optional but recommended)
plot([R_C_977B R_C_UF], [P_arrow P_arrow], ...
    '--', 'Color',[0.6 0.6 0.6], 'HandleVisibility','off')

% --- label (same placement, but clearer wording)
text((R_C_UF + R_C_977B)/2, P_arrow + 3, ...
    sprintf('+%.1f%% range', delta_range_pct), ...
    'HorizontalAlignment','center', ...
    'FontSize',14, ...
    'FontWeight','bold')
%% -------------------- DESIGN TOOL --------------------

payload_design = 123 * 1e3;

fuel_design = MTOM - OEM - payload_design;

if payload_design > maxPayload_struct
    error('Payload exceeds structural limit (MZFW)')
end

if fuel_design > maxFuel
    warning('Fuel capped by tank')
    fuel_design = maxFuel;
end

if fuel_design <= 0
    error('No fuel possible')
end

% ---- ranges ----
R_design_UF   = breguet(fuel_design, TSFC_UF);
R_design_977B = breguet(fuel_design, TSFC_977B);

% ---- plot ----
plot(R_design_UF, payload_design/1000, 'bx','MarkerSize',10,'LineWidth',2,'DisplayName','Design Point')
plot(R_design_977B, payload_design/1000, 'rx','MarkerSize',10,'LineWidth',2,'DisplayName','Design Point')

text(R_design_UF, (payload_design)/1000, '  UF')
text(R_design_977B, (0.95*payload_design)/1000, '  977B')

% projections
plot([R_design_UF R_design_UF], [0 payload_design/1000], 'b--','HandleVisibility','off')
plot([0 R_design_UF], [payload_design/1000 payload_design/1000], 'b--','HandleVisibility','off')

plot([R_design_977B R_design_977B], [0 payload_design/1000], 'r--','HandleVisibility','off')
plot([0 R_design_977B], [payload_design/1000 payload_design/1000], 'r--','HandleVisibility','off')

text(R_B_UF*0.2, P_A/1000 + 5, 'Max Payload', ...
    'FontSize',16,'FontWeight','bold')

text(R_C_UF * 0.8, P_C/1000 * 0.2, 'Max Fuel Capacity', ...
    'FontWeight','bold','HorizontalAlignment','left')

text(R_B_UF + ((R_C_UF-R_B_UF)*1.5)/2, ...
     ((P_B/1000 + P_C/1000)*1.5)/2, ...
     'MTOM Limit', ...
     'FontSize',16,'FontWeight','bold','Rotation',-45)

plot(R_B_UF, P_B/1000, 'ko','MarkerFaceColor','k','HandleVisibility','off')
plot(R_C_UF, P_C/1000, 'ko','MarkerFaceColor','k','HandleVisibility','off')
plot(R_D_UF, P_D/1000, 'ko','MarkerFaceColor','k','HandleVisibility','off')

%% -------------------- PRINT --------------------

fprintf('\n--- DESIGN RESULT ---\n')
fprintf('Payload: %.1f t\n', payload_design/1000)
fprintf('Fuel: %.1f t\n', fuel_design/1000)
fprintf('Range (UF): %.0f km\n', R_design_UF)
fprintf('Range (977B): %.0f km\n', R_design_977B)

fprintf('\n--- KEY POINTS ---\n')
fprintf('UF Max Payload Range: %.0f km\n', R_B_UF)
fprintf('977B Max Payload Range: %.0f km\n', R_B_977B)
fprintf('UF Max Fuel Range: %.0f km\n', R_C_UF)
fprintf('977B Max Fuel Range: %.0f km\n', R_C_977B)
fprintf('Payload at max fuel: %.1f t\n', P_C/1000);

%% -------------------- ROUTES ON PAYLOAD-RANGE DIAGRAM --------------------




%% -------------------- ROUTE PAYLOAD ANALYSIS --------------------

AirportPairs = [
"LHR-MEL"
"MEL-PVG"
"PVG-SUZ"
"SUZ-BAH"
"BAH-JED"
"JED-MIA"
"MIA-YUL"
"YUL-MCM"
"MCM-MAD"
"MAD-GYD"
"GYD-SIN"
"SIN-AUS"
"AUS-MEX"
"MEX-GRU"
"GRU-LAS"
"LAS-LUA"
"LUA-AUH"
"AUH-LHR"
];

dist_km = [
16909.38
8017.62
1456.91
8052.37
1272.47
11621.60
2264.78
6128.98
956.83
4462.04
6941.58
15823.46
1205.03
7433.53
9782.64
13052.79
320.63
5515.99
];

disp(' ')
disp('--- ROUTE PAYLOAD RESULTS ---')



% --- storage ---
payloads_UF_routes   = zeros(length(dist_km),1);
payloads_977B_routes = zeros(length(dist_km),1);

fprintf('\n--- ROUTE PAYLOAD RESULTS ---\n')

for i = 1:length(dist_km)

    R_target = dist_km(i)*1000;
    fuel_guess = 0.5*maxFuel;

    % ---------- ULTRAFAN ----------
    fuel_fun_UF = @(fuel) (V/TSFC_UF)*LD*log(MTOM/(MTOM - fuel)) - R_target;

    try
        fuel_UF = fzero(fuel_fun_UF, fuel_guess);
    catch
        fuel_UF = maxFuel;
    end

    fuel_UF = min(max(fuel_UF,0), maxFuel);

    payload_UF = MTOM - OEM - fuel_UF;
    payload_UF = min(payload_UF, maxPayload_struct);

    payloads_UF_routes(i) = payload_UF;


    % ---------- 977B ----------
    fuel_fun_977B = @(fuel) (V/TSFC_977B)*LD*log(MTOM/(MTOM - fuel)) - R_target;

    try
        fuel_977B = fzero(fuel_fun_977B, fuel_guess);
    catch
        fuel_977B = maxFuel;
    end

    fuel_977B = min(max(fuel_977B,0), maxFuel);

    payload_977B = MTOM - OEM - fuel_977B;
    payload_977B = min(payload_977B, maxPayload_struct);

    payloads_977B_routes(i) = payload_977B;


    % ---------- PRINT ----------
    fprintf('\n%s (%.0f km)\n', AirportPairs(i), dist_km(i))
    fprintf('  UF Payload:   %.1f t\n', payload_UF/1000)
    fprintf('  977B Payload: %.1f t\n', payload_977B/1000)

end

figure; hold on; grid on

% Plot UltraFan envelope
plot(ranges_UF/1000, payloads_UF/1000, '-ob','LineWidth',2,'DisplayName','UltraFan Envelope')

% Plot route points
plot(dist_km, payloads_UF_routes/1000, 'ks', ...
    'MarkerFaceColor','y','DisplayName','Routes')

xlabel('Range (km)')
ylabel('Payload (tonnes)')
title('UltraFan Payload–Range with Mission Routes')
legend show

xlim([0 max(ranges_UF/1000)*1.5])
ylim([0 max(payloads_UF/1000)*1.5])

