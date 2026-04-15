%% PAYLOAD RANGE DIAGRAM (FULL - WITH MZFW)

clear; clc; close all;

%% -------------------- INPUTS --------------------

% Masses (tonnes)
MTOM = 560;
OEM  = 265;

maxPayload_struct = 138.5;
maxFuel           = 250;

% Convert to kg
MTOM = MTOM * 1e3;
OEM  = OEM  * 1e3;
maxPayload_struct = maxPayload_struct * 1e3;
maxFuel = maxFuel * 1e3;

% Performance assumptions
LD  = 16;
M   = 0.82;
alt = 11000;

% Atmosphere
T0 = 288.15;
a0 = 340;
a  = a0 * sqrt(1 - 0.0065*alt/T0);
V  = M * a;

% TSFC
TSFC_hr = 0.55;
TSFC = TSFC_hr / 3600;

%% -------------------- MASS LIMITS --------------------

usefulLoad = MTOM - OEM;

% Max Zero Fuel Weight (IMPORTANT for extra slope)
MZFW = OEM + maxPayload_struct;

%% -------------------- BREGUET FUNCTION --------------------

breguet = @(fuel) (V/TSFC)*LD*log(MTOM/(MTOM - fuel)) / 1000;

%% -------------------- KEY POINTS --------------------

% A: Start (zero range, max payload)
R_A = 0;
P_A = maxPayload_struct;

% B: Max payload range (MZFW limit reached)
fuel_B = MTOM - MZFW;
R_B = breguet(fuel_B);
P_B = maxPayload_struct;

% C: tank limit
%% -------------------- POINT C --------------------

fuel_C = maxFuel;
P_C = usefulLoad - fuel_C;

Wi_C = MTOM;
Wf_C = MTOM - fuel_C;

R_C = (V/TSFC)*LD*log(Wi_C / Wf_C) / 1000;

%% -------------------- POINT D (CORRECT FIX) --------------------

fuel_D = maxFuel;
P_D = 0;

Wi_D = OEM + fuel_D;   % <-- THIS is the key fix
Wf_D = OEM;

R_D = (V/TSFC)*LD*log(Wi_D / Wf_D) / 1000;

%% -------------------- BUILD SHAPE --------------------

ranges_plot = [R_A; R_B; R_C; R_D];
payloads_plot = [P_A; P_B; P_C; P_D];

%% -------------------- PLOT --------------------

figure;
plot(ranges_plot, payloads_plot/1000, '-o','LineWidth',2)
grid on

xlabel('Range (km)')
ylabel('Payload (tonnes)')
title('Payload–Range Diagram')

xlim([0 max(ranges_plot)*1.1])
ylim([0 max(payloads_plot/1000)*1.1])

%% -------------------- DESIGN TOOL --------------------

hold on

payload_design = 130.7 * 1e3;   % <-- change this ONLY

% --- compute fuel from mass balance ---
fuel_design = MTOM - OEM - payload_design;

% --- enforce physical limits ---
if payload_design > maxPayload_struct
    error('Payload exceeds structural limit (MZFW)')
end

if fuel_design > maxFuel
    warning('Payload too low → fuel capped by tank, switching to fuel-limited region')
    fuel_design = maxFuel;
end

if fuel_design <= 0
    error('Payload too high → no fuel possible')
end

% --- compute range ---
R_design = breguet(fuel_design);

% --- plot ---
plot(R_design, payload_design/1000, 'rx', ...
    'MarkerSize',10,'LineWidth',2)

text(R_design, payload_design/1000, '  Design Point')

% --- print ---
fprintf('\n--- DESIGN RESULT ---\n')
fprintf('Payload: %.1f t\n', payload_design/1000)
fprintf('Fuel: %.1f t\n', fuel_design/1000)
fprintf('Range: %.0f km\n', R_design)
%% -------------------- OUTPUT --------------------

disp('--- KEY POINTS ---')
fprintf('Max payload range: %.0f km\n', R_B);
fprintf('Max fuel range: %.0f km\n', R_C);
fprintf('Payload at max fuel: %.1f t\n', P_C/1000);