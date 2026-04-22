%% PAYLOAD-RANGE DIAGRAM (ULTRAFAN ONLY, CONSISTENT UNITS)
% Internal units:
%   mass   -> kg
%   range  -> m
%   speed  -> m/
%   TSFC   -> kg/N/s
% Plot/output units:
%   range  -> km
%   payload-> tonnes



%clear; clc; close all;

set(0,'DefaultFigureColor','w')
set(0,'DefaultAxesFontSize',18)
set(0,'DefaultTextFontSize',18)

%% -------------------- INPUTS --------------------

% Masses [tonnes]
MTOM            = ADP.MTOM;
OEM             = ADP.OEM;
%maxPayload_struct_t = 138.5;
%maxFuel_t         = 0.4 * MTOM_t;

rho_fuel = 800; % kg/m^3

V_fuel = 613.8; % m^3 (example)

maxFuel = rho_fuel * V_fuel*0.33; % kg

%maxFuel         = 0.32 * MTOM;


%maxFuel_t = ADP.Mf_Fuel * MTOM;
maxPayload_struct = ADP.TLAR.Payload;   % if defined properly


% % Convert to SI [kg]
% MTOM              = MTOM_t * 1e3;
% OEM               = OEM_t  * 1e3;
% maxPayload_struct = maxPayload_struct_t * 1e3;
% maxFuel           = maxFuel_t * 1e3;

% % Performance assumptions
% LD       = 16;
% M_cruise = 0.72;
% alt      = 11000;     % m
g        = 9.81;      % m/s^2


M_cruise = ADP.TLAR.M_c;

LD = ADP.LD_c;

alt = ADP.TLAR.Alt_cruise;


%% -------------------- ENGINE / ATMOSPHERE --------------------

eng_UF = cast.eng.TurboFan.UF(1, alt, M_cruise);
TSFC_UF = eng_UF.TSFC(M_cruise, alt);   % kg/N/s

fprintf('\n--- CRUISE CONDITIONS ---\n')
fprintf('UltraFan TSFC: %.3e kg/N/s\n', TSFC_UF)

% Simple atmosphere
T0 = 288.15;
a0 = 340;
a  = a0 * sqrt(1 - 0.0065*alt/T0);   % m/s
V  = M_cruise * a;                   % m/s

fprintf('Cruise speed: %.1f m/s\n', V)

%% -------------------- MASS LIMITS --------------------

usefulLoad = MTOM - OEM;
MZFW       = OEM + maxPayload_struct;

%% -------------------- UNIT HELPERS --------------------

m2km = @(x) x/1000;
kg2t = @(x) x/1000;

%% -------------------- CORRECTED BREGUET --------------------
% Since TSFC is in kg/N/s, the mass-based Breguet needs g in the denominator.

range_from_fuel = @(fuel, TSFC) (V/(TSFC*g)) * LD * log(MTOM/(MTOM - fuel));  % [m]

% Closed-form inversion of Breguet:
% R = (V/(TSFC*g))*LD*ln(MTOM/(MTOM-fuel))
% => fuel = MTOM*(1 - exp(-R*TSFC*g/(V*LD)))
fuel_from_range = @(R, TSFC) MTOM * (1 - exp(-(R*TSFC*g)/(V*LD)));             % [kg]

%% -------------------- PAYLOAD-RANGE ENVELOPE POINTS --------------------

% A: zero range, max payload
R_A = 0;
P_A = maxPayload_struct;

% B: max payload range (limited by MZFW)
fuel_B = MTOM - MZFW;
R_B = range_from_fuel(fuel_B, TSFC_UF);
P_B = maxPayload_struct;

% C: max fuel point
fuel_C = maxFuel;
R_C = range_from_fuel(fuel_C, TSFC_UF);
P_C = usefulLoad - fuel_C;

% D: ferry range (zero payload, max fuel)
Wi_D = OEM + maxFuel;
Wf_D = OEM;
R_D  = (V/(TSFC_UF*g)) * LD * log(Wi_D/Wf_D);
P_D  = 0;

ranges_m    = [R_A; R_B; R_C; R_D];
payloads_kg = [P_A; P_B; P_C; P_D];

ranges_km   = m2km(ranges_m);
payloads_t  = kg2t(payloads_kg);

%% -------------------- PRINT ENVELOPE --------------------

fprintf('\n--- ENVELOPE POINTS (ULTRAFAN) ---\n')
fprintf('Max payload range (B): %.0f km\n', m2km(R_B))
fprintf('Max fuel point (C):    %.0f km at %.1f t payload\n', m2km(R_C), kg2t(P_C))
fprintf('Ferry range (D):       %.0f km\n', m2km(R_D))

%% -------------------- ROUTE LIST --------------------

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
5454
];

%% -------------------- ROUTE PAYLOAD ANALYSIS --------------------

nRoutes = length(dist_km);

payloads_routes_kg   = zeros(nRoutes,1);
fuel_required_kg     = zeros(nRoutes,1);
isWithinEnvelope     = false(nRoutes,1);

fprintf('\n--- ROUTE PAYLOAD RESULTS (ULTRAFAN) ---\n')

for i = 1:nRoutes

    R_target_m = dist_km(i) * 1000;
    fuel_req   = fuel_from_range(R_target_m, TSFC_UF);

    fuel_required_kg(i) = fuel_req;

    if R_target_m <= R_C
        % ---------- REGION A → C (fuel-limited) ----------
    
        fuel_req = fuel_from_range(R_target_m, TSFC_UF);
        fuel_req = min(fuel_req, maxFuel);
    
        payload = MTOM - OEM - fuel_req;
        payload = min(payload, maxPayload_struct);
        payload = max(payload, 0);
    
        payloads_routes_kg(i) = payload;
        fuel_required_kg(i)   = fuel_req;
        isWithinEnvelope(i)   = true;
    
    else
        % ---------- REGION C → D (payload-limited) ----------
    
        fuel_req = maxFuel;
    
        % Solve for payload instead of fuel
        payload_fun = @(payload) (V/(TSFC_UF*g))*LD*log( ...
            (OEM + payload + maxFuel) / (OEM + payload) ) - R_target_m;
    
        try
            payload = fzero(payload_fun, P_C);   % good initial guess
        catch
            payload = 0;
        end
    
        payload = max(payload, 0);
    
        payloads_routes_kg(i) = payload;
        fuel_required_kg(i)   = fuel_req;
    
        % still inside envelope IF payload > 0
        isWithinEnvelope(i) = payload > 0;
    end
end

payloads_routes_t = kg2t(payloads_routes_kg);

%% -------------------- PLOT --------------------



figure; hold on; grid on

% Envelope
plot(ranges_km, payloads_t, '-ob', ...
    'LineWidth', 2, ...
    'DisplayName', 'UltraFan Envelope')

% Key envelope points
plot(ranges_km, payloads_t, 'ko', ...
    'MarkerFaceColor','k', ...
    'HandleVisibility','off')

% Feasible routes
plot(dist_km(isWithinEnvelope), payloads_routes_t(isWithinEnvelope), 'ks', ...
    'MarkerFaceColor', 'y', ...
    'MarkerSize', 7, ...
    'DisplayName', 'Routes within envelope')

% Infeasible routes
plot(dist_km(~isWithinEnvelope), payloads_routes_t(~isWithinEnvelope), 'rx', ...
    'MarkerSize', 9, ...
    'LineWidth', 2, ...
    'DisplayName', 'Routes outside envelope')

% Labels
xlabel('Range (km)')
ylabel('Payload (tonnes)')
title('UltraFan Payload–Range with Mission Routes')
legend('Location','northeast')

% Make MATLAB stop using huge scientific notation on x-axis
ax = gca;
ax.XAxis.Exponent = 0;

xlim([0, 1.05*max([ranges_km; dist_km])])
ylim([0, 1.10*max(payloads_t)])

% Optional annotations
text(0.15*R_B, kg2t(P_A) + 4, 'Max Payload', ...
    'FontWeight','bold')

text(0.82*R_C, 0.25*kg2t(P_C), 'Max Fuel Point', ...
    'FontWeight','bold')

text(0.93*R_D, 8, 'Ferry Range', ...
    'FontWeight','bold', ...
    'HorizontalAlignment','right')

%% -------------------- SUMMARY --------------------

fprintf('\n--- SUMMARY ---\n')
fprintf('Routes inside envelope:  %d / %d\n', sum(isWithinEnvelope), nRoutes)
fprintf('Routes outside envelope: %d / %d\n', sum(~isWithinEnvelope), nRoutes)


%% ==================== ENGINE COMPARISON (SEPARATE FIGURE) ====================

% ---- Second engine ----
eng_977B = cast.eng.TurboFan.T_977B(1, alt, M_cruise);
TSFC_977B = eng_977B.TSFC(M_cruise, alt);

% ---- Breguet for 977B (same correct formulation) ----
range_from_fuel_977B = @(fuel) (V/(TSFC_977B*g)) * LD * log(MTOM/(MTOM - fuel));

% ---- Envelope points (reuse SAME masses) ----

% A
R_A_977B = 0;
P_A_977B = maxPayload_struct;

% B
fuel_B = MTOM - MZFW;
R_B_977B = range_from_fuel_977B(fuel_B);
P_B_977B = maxPayload_struct;

% C
fuel_C = maxFuel;
R_C_977B = range_from_fuel_977B(fuel_C);
P_C_977B = usefulLoad - fuel_C;

% D (ferry)
Wi_D = OEM + maxFuel;
Wf_D = OEM;
R_D_977B = (V/(TSFC_977B*g)) * LD * log(Wi_D/Wf_D);
P_D_977B = 0;

% ---- Convert ----
ranges_977B_km = [R_A_977B; R_B_977B; R_C_977B; R_D_977B] / 1000;
payloads_977B_t = [P_A_977B; P_B_977B; P_C_977B; P_D_977B] / 1000;

% ---- Existing UltraFan already computed ----
ranges_UF_km = ranges_km;
payloads_UF_t = payloads_t;

% ---- Plot new figure ----
figure; hold on; grid on

plot(ranges_UF_km, payloads_UF_t, '-ob', ...
    'LineWidth',2,'DisplayName','UltraFan')

plot(ranges_977B_km, payloads_977B_t, '-or', ...
    'LineWidth',2,'DisplayName','Trent 977B')

xlabel('Range (km)')
ylabel('Payload (tonnes)')
title('Payload–Range Comparison (UltraFan vs 977B)')
legend('Location','northeast')

xlim([0, 1.05*max([ranges_UF_km; ranges_977B_km])])
ylim([0, 1.10*max(payloads_UF_t)])

% ---- Optional: % improvement at max fuel point ----
delta_range_pct = (R_C - R_C_977B) / R_C_977B * 100;

text(0.5*(R_C + R_C_977B), (P_C/1000)+5, ...
    sprintf('+%.1f%% range', delta_range_pct), ...
    'HorizontalAlignment','center', ...
    'FontWeight','bold')


%% ==================== DESIGN OPERATION FIGURE (123t SPLIT LOGIC) ====================

% ---- Fleet requirement ----
Payload_tot = 736 * 1000;   % kg (fleet total)
N_fleetsize = 6;
Payload_operating = Payload_tot / N_fleetsize;   % kg per aircraft

% ---- compute design range ----
fuel_design = MTOM - OEM - Payload_operating;
fuel_design = min(fuel_design, maxFuel);

R_design = range_from_fuel(fuel_design, TSFC_UF);   % [m]
R_design_km = R_design / 1000;

fprintf('\n--- DESIGN POINT ---\n')
fprintf('Design payload: %.1f kg (%.1f t)\n', ...
    Payload_operating, Payload_operating/1000)
fprintf('Design range:   %.0f km\n', R_design_km)

% ---- split routes ----
dist_split_km    = [];
payload_split_kg = [];

for i = 1:nRoutes
    
    if dist_km(i) <= R_design_km
        % --- no split ---
        dist_split_km(end+1)     = dist_km(i);
        payload_split_kg(end+1)  = payloads_routes_kg(i);
        
    else
        % --- split into 2 legs ---
        leg_dist_km = dist_km(i)/2;
        leg_dist_m  = leg_dist_km * 1000;
        
        fuel_req = fuel_from_range(leg_dist_m, TSFC_UF);
        fuel_req = min(fuel_req, maxFuel);
        
        payload = MTOM - OEM - fuel_req;
        payload = min(payload, maxPayload_struct);
        payload = max(payload, 0);
        
        % store BOTH legs
        dist_split_km(end+1)     = leg_dist_km;
        payload_split_kg(end+1)  = payload;
        
        dist_split_km(end+1)     = leg_dist_km;
        payload_split_kg(end+1)  = payload;
        
        % debug print
        fprintf('Splitting %.0f km -> %.0f + %.0f km\n', ...
            dist_km(i), leg_dist_km, leg_dist_km)
    end
end

%% ==================== PLOT ====================

figure; hold on; grid on

% Envelope (already in tonnes/km)
plot(ranges_km, payloads_t, '-ob', ...
    'LineWidth',2, ...
    'DisplayName','Envelope')

% Original routes (already tonnes)
plot(dist_km, payloads_routes_t, 'ks', ...
    'MarkerFaceColor','y', ...
    'DisplayName','Original Routes')

% Split routes (convert kg → tonnes ONLY HERE)
plot(dist_split_km, payload_split_kg/1000, 'rd', ...
    'MarkerFaceColor','r', ...
    'DisplayName','Split Routes')

% Design point (convert kg → tonnes)
% plot(R_design_km, Payload_operating/1000, 'bx', ...
%     'MarkerSize',12, ...
%     'LineWidth',2, ...
%     'DisplayName','Design Point')

plot(R_design_km, Payload_operating/1000, 'x', ...
    'Color','b', ...
    'MarkerSize',14, ...
    'LineWidth',3, ...
    'DisplayName','Design Point')

% Design range line
xline(R_design_km, '--b', 'Design Range', ...
    'LabelVerticalAlignment','bottom')

xlabel('Range (km)')
ylabel('Payload (tonnes)')
title('Operational Strategy: 123t Design + Route Splitting')
legend('Location','northeast')

xlim([0, 1.05*max([ranges_km; dist_km])])
ylim([0, 1.10*max(payloads_t)])





%% ==================== OPERATIONAL MODEL (FIXED PAYLOAD) ====================



fprintf('\n--- OPERATIONAL MODEL (FIXED PAYLOAD - CONSISTENT) ---\n')

payload_oper = Payload_operating;   % kg



dist_oper_km    = [];
payload_oper_kg = [];
route_type      = [];   % 1 = direct, 2 = split

% -------------------- HELPER: RANGE FROM PAYLOAD --------------------
% Enforces MTOM and fuel limits explicitly

range_from_payload = @(payload, fuel) ...
    (V/(TSFC_UF*g))*LD*log( ...
    (OEM + payload + fuel) / (OEM + payload));

% Max fuel allowed by MTOM for given payload
fuel_MTOM_limit = @(payload) MTOM - OEM - payload;

for i = 1:nRoutes
    
    R_target_m = dist_km(i) * 1000;
    
    % -------------------- DIRECT CHECK --------------------
    
    fuel_limit_struct = maxFuel;
    fuel_limit_mass   = fuel_MTOM_limit(payload_oper);
    
    fuel_available = min(fuel_limit_struct, fuel_limit_mass);
    
    % Max achievable range at this payload
    R_max_direct = range_from_payload(payload_oper, fuel_available);
    
    if R_target_m <= R_design
        % ✅ DIRECT FLIGHT (PHYSICALLY VALID)

        if AirportPairs(i) == "AUS-MEX"
            payload_here = 0;   % <-- FORCE EMPTY FLIGHT
        else
            payload_here = payload_oper;
        end
        
        dist_oper_km(end+1)     = dist_km(i);
        payload_oper_kg(end+1)  = payload_here;
        route_type(end+1)       = 1;
        
        fprintf('Direct: %.0f km\n', dist_km(i))
        
    else
        % -------------------- SPLIT (1 STOP) --------------------
        
        leg_dist_km = dist_km(i)/2;
        leg_dist_m  = leg_dist_km * 1000;
        
        % recompute fuel limit (same payload)
        fuel_available = min(maxFuel, fuel_MTOM_limit(payload_oper));
        
        R_max_leg = range_from_payload(payload_oper, fuel_available);
        
        if leg_dist_m <= R_max_leg
            % ✅ SPLIT FEASIBLE
            
            % store BOTH legs + matching type
            dist_oper_km(end+1)    = leg_dist_km;
            payload_oper_kg(end+1) = payload_oper;
            route_type(end+1)      = 2;
            
            dist_oper_km(end+1)    = leg_dist_km;
            payload_oper_kg(end+1) = payload_oper;
            route_type(end+1)      = 2;
            
            fprintf('Split (1 stop): %.0f km -> %.0f + %.0f km\n', ...
                dist_km(i), leg_dist_km, leg_dist_km)
            
        else
            % ❌ NOT FEASIBLE EVEN WITH 1 STOP
            
            fprintf('INFEASIBLE (>1 stop needed): %.0f km\n', dist_km(i))
        end
    end
end

payload_oper_t = payload_oper_kg / 1000;   % <-- ADD THIS LINE


%% ==================== OPERATIONAL PLOT ====================

figure; hold on; grid on

% Envelope
plot(ranges_km, payloads_t, '-ob', ...
    'LineWidth',2,'DisplayName','Envelope')

% Original routes
plot(dist_km, payloads_routes_t, 'ks', ...
    'MarkerFaceColor','y', ...
    'DisplayName','Original Routes')

% -------------------- SPLIT vs DIRECT --------------------
is_direct = route_type == 1;
is_split  = route_type == 2;

% Direct (magenta)
plot(dist_oper_km(is_direct), payload_oper_t(is_direct), 'md', ...
    'MarkerFaceColor','m', ...
    'DisplayName','Operational Direct (123t)')

% Split (cyan)
plot(dist_oper_km(is_split), payload_oper_t(is_split), 'cd', ...
    'MarkerFaceColor','c', ...
    'DisplayName','Operational Split (1 stop)')

% Design point
plot(R_design_km, payload_oper/1000, 'bx', ...
    'MarkerSize',12,'LineWidth',2, ...
    'DisplayName','Design Point')

% Design range line
xline(R_design_km, '--b', 'Design Range', ...
    'LabelVerticalAlignment','bottom')

xlabel('Range (km)')
ylabel('Payload (tonnes)')
title('Operational Strategy (Fixed Payload Across Network)')
legend('Location','northeast')

xlim([0, 1.05*max([ranges_km; dist_km])])
ylim([0, 1.10*max(payloads_t)])

%% ----------- INPUTS YOU ALREADY HAVE -----------
Payload_total = 736e3;   % kg  :contentReference[oaicite:0]{index=0}
N_aircraft    = 6;

Payload_oper = Payload_total / N_aircraft;

Fleet.N_aircraft = N_aircraft;

%% ----------- DESIGN RANGE (JUST A CONSTANT) -----------
R_design = ADP.TLAR.RangeDes / 1000;   % km
% or hardcode if cleaner:
% R_design = 10186;

%% ----------- FLIGHT LOGIC ONLY -----------
k = 1;

for i = 1:length(dist_km)

    R = dist_km(i);

    if R <= R_design
        % -------- DIRECT --------
        Flights(k).Type = "direct";

    else
        % -------- SPLIT (max 1 stop) --------
        Flights(k).Type = "split";
    end

    % -------- STORE ONLY WHAT YOU CARE ABOUT --------
    Flights(k).Payload_kg = Payload_oper;
    Flights(k).NumLegs    = 1 + (R > R_design);  % 1 or 2

    % ✅ UNIQUE IDENTIFIER
    Flights(k).RouteName = AirportPairs(i);

    k = k + 1;
end

Fleet.Flights = Flights;