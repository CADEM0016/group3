%% MISSION-BASED OPERATING PAYLOAD-RANGE PROOF (UNCONVENTIONAL, ONE FIGURE)
% This script:
%   1) sizes the unconventional aircraft
%   2) builds a mission-based payload-range envelope
%   3) computes mission-based max payload on each route
%   4) proves fixed-payload operations at fleet-driven payload
%   5) plots ONE final figure only
%
% IMPORTANT:
%   In MissionAnalysis_oscar / MissionAnalysis, replace:
%       matlab.desktop.editor.getActiveFilename
%   with:
%       mfilename('fullpath')

clear; clc; close all;

set(0,'DefaultFigureColor','w')
set(0,'DefaultAxesFontSize',18)
set(0,'DefaultTextFontSize',18)

%% ==================== 1) AIRCRAFT SIZING ====================

% ---- Build baseline ADP from your sizing loop ----
ADP0 = Unconventional.ADP();
ADP0.TLAR = cast.TLAR.Unconventional();
ADP0.TLAR.M_c = 0.84;

% --------------------- unconventional-specific parameters ---------------------
ADP0.FuselageLength = 65;
ADP0.KinkPos        = 10;
ADP0.CabinRadius    = 6.3;
ADP0.CabinLength    = 50;
ADP0.CockpitLength  = 5;

ADP0.WingPos = 0.44 * ADP0.FuselageLength;
ADP0.V_HT    = 0.97;
ADP0.V_VT    = 0.072;
ADP0.HtpPos  = 0.85 * ADP0.FuselageLength;
ADP0.VtpPos  = 0.82 * ADP0.FuselageLength;

ADP0.WingArea = 750;   % m^2 guess

% ------------------------- hyperparameters -------------------------
ADP0.Span = 74;

% -------------------------- class-I estimates ---------------------------
ADP0.MTOM    = 490000;  % kg initial guess
ADP0.Mf_Fuel = 0.32;
ADP0.Mf_res  = 0.03;
ADP0.Mf_Ldg  = 0.62;
ADP0.Mf_TOC  = 0.975;

% -------------------------- size aircraft --------------------------
ADP = Unconventional.Size(ADP0);

%% ==================== 2) MASS / OPS INPUTS ====================

% Use sized aircraft where possible
MTOM = ADP.MTOM;   % kg

% OEM fallback: use ADP.OEM if available, otherwise set manually
if isprop(ADP,'OEM') && ~isempty(ADP.OEM) && isfinite(ADP.OEM) && ADP.OEM > 0
    OEM = ADP.OEM;
else
    OEM = 265e3;  % fallback
    warning('ADP.OEM not found or invalid. Using fallback OEM = 265,000 kg.')
end

% Structural payload cap
maxPayload_struct = 138.5e3;  % kg

% Max fuel from sized aircraft
if isprop(ADP,'Mf_Fuel') && ~isempty(ADP.Mf_Fuel) && isfinite(ADP.Mf_Fuel)
    maxFuel = ADP.Mf_Fuel * ADP.MTOM;
else
    maxFuel = 0.4 * MTOM;
    warning('ADP.Mf_Fuel not found. Using fallback maxFuel = 0.4*MTOM.')
end

% Fleet / ops
Payload_tot = 736e3;     % kg fleet total
N_fleetsize = 6;
payload_oper = Payload_tot / N_fleetsize;   % kg per aircraft

% Unit helpers
m2km = @(x) x/1000;
kg2t = @(x) x/1000;

fprintf('\n--- SIZED AIRCRAFT ---\n')
fprintf('MTOM: %.1f t\n', MTOM/1000)
fprintf('OEM:  %.1f t\n', OEM/1000)
fprintf('Max fuel: %.1f t\n', maxFuel/1000)
fprintf('Max structural payload: %.1f t\n', maxPayload_struct/1000)
fprintf('Operational payload: %.1f t\n', payload_oper/1000)

%% ==================== 3) ROUTE LIST ====================

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

nRoutes = numel(dist_km);

%% ==================== 4) SELECT MISSION FUNCTION ====================
% Keep ONE of these active depending on your actual function name.

mission_fun = @(ADP,tripRange,M_TO) Unconventional.MissionAnalysis_oscar(ADP,tripRange,M_TO);

% If your function is plain MissionAnalysis.m instead, use this instead:
% mission_fun = @(ADP,tripRange,M_TO) MissionAnalysis(ADP,tripRange,M_TO);

%% ==================== 5) SILENCE MISSION DEBUG FIGURES/PRINTS ====================

oldFigVis = get(0,'DefaultFigureVisible');
set(0,'DefaultFigureVisible','off');
cleanupObj = onCleanup(@() set(0,'DefaultFigureVisible', oldFigVis)); %#ok<NASGU>

fprintf('\n--- BUILDING MISSION-BASED ENVELOPE ---\n')

%% ==================== 6) MISSION-BASED ENVELOPE ====================

payload_grid_kg = linspace(maxPayload_struct, 0, 31)';   % high -> low payload
range_env_m     = nan(size(payload_grid_kg));

for i = 1:length(payload_grid_kg)
    payload_i = payload_grid_kg(i);

    range_env_m(i) = range_for_payload_mission( ...
        payload_i, ADP, MTOM, OEM, maxFuel, mission_fun);
end

ranges_km  = m2km(range_env_m);
payloads_t = kg2t(payload_grid_kg);

%% ==================== 7) DESIGN POINT (MISSION-BASED) ====================

R_design_m  = range_for_payload_mission(payload_oper, ADP, MTOM, OEM, maxFuel, mission_fun);
R_design_km = m2km(R_design_m);

fprintf('\n--- DESIGN POINT (MISSION-BASED) ---\n')
fprintf('Design payload: %.1f kg (%.1f t)\n', payload_oper, payload_oper/1000)
fprintf('Design range:   %.0f km\n', R_design_km)

%% ==================== 8) ROUTE PAYLOAD ANALYSIS (MISSION-BASED) ====================

payloads_routes_kg = zeros(nRoutes,1);
isWithinEnvelope   = false(nRoutes,1);

fprintf('\n--- ROUTE PAYLOAD RESULTS (MISSION-BASED) ---\n')

for i = 1:nRoutes
    R_target_m = dist_km(i) * 1000;

    [payload_sol, feasible] = payload_for_range_mission( ...
        R_target_m, ADP, MTOM, OEM, maxPayload_struct, maxFuel, mission_fun);

    payloads_routes_kg(i) = payload_sol;
    isWithinEnvelope(i)   = feasible;

    if feasible
        fprintf('%s : feasible, max payload = %.1f t\n', ...
            AirportPairs(i), payload_sol/1000)
    else
        fprintf('%s : infeasible even at zero payload\n', AirportPairs(i))
    end
end

payloads_routes_t = kg2t(payloads_routes_kg);

fprintf('\n--- SUMMARY ---\n')
fprintf('Routes inside envelope:  %d / %d\n', sum(isWithinEnvelope), nRoutes)
fprintf('Routes outside envelope: %d / %d\n', sum(~isWithinEnvelope), nRoutes)

%% ==================== 9) OPERATIONAL MODEL (FIXED PAYLOAD, MISSION-BASED) ====================

dist_oper_km    = [];
payload_oper_kg = [];
route_type      = [];   % 1 = direct, 2 = split
route_labels    = strings(0,1);

fprintf('\n--- OPERATIONAL MODEL (FIXED PAYLOAD, MISSION-BASED) ---\n')

for i = 1:nRoutes
    R_target_m = dist_km(i) * 1000;

    if R_target_m <= R_design_m
        % Direct
        dist_oper_km(end+1,1)    = dist_km(i);
        payload_oper_kg(end+1,1) = payload_oper;
        route_type(end+1,1)      = 1;
        route_labels(end+1,1)    = AirportPairs(i);

        fprintf('Direct: %s (%.0f km)\n', AirportPairs(i), dist_km(i))

    else
        % Try one stop = 2 equal legs
        leg_dist_km = dist_km(i) / 2;
        leg_dist_m  = leg_dist_km * 1000;

        if leg_dist_m <= R_design_m
            dist_oper_km(end+1,1)    = leg_dist_km;
            payload_oper_kg(end+1,1) = payload_oper;
            route_type(end+1,1)      = 2;
            route_labels(end+1,1)    = AirportPairs(i);

            dist_oper_km(end+1,1)    = leg_dist_km;
            payload_oper_kg(end+1,1) = payload_oper;
            route_type(end+1,1)      = 2;
            route_labels(end+1,1)    = AirportPairs(i);

            fprintf('Split (1 stop): %s -> %.0f + %.0f km\n', ...
                AirportPairs(i), leg_dist_km, leg_dist_km)
        else
            fprintf('INFEASIBLE (>1 stop needed): %s (%.0f km)\n', ...
                AirportPairs(i), dist_km(i))
        end
    end
end

%% ==================== 10) FINAL PLOT (ONE FIGURE ONLY) ====================

set(0,'DefaultFigureVisible','on');

figure; hold on; grid on

% Mission-based envelope
plot(ranges_km, payloads_t, '-ob', ...
    'LineWidth', 2, ...
    'DisplayName', 'Mission-Based Envelope')

% Route max-payload markers
plot(dist_km(isWithinEnvelope), payloads_routes_t(isWithinEnvelope), 'ks', ...
    'MarkerFaceColor', 'y', ...
    'MarkerSize', 7, ...
    'DisplayName', 'Mission-Based Route Payload')

plot(dist_km(~isWithinEnvelope), payloads_routes_t(~isWithinEnvelope), 'rx', ...
    'MarkerSize', 9, ...
    'LineWidth', 2, ...
    'DisplayName', 'Route Outside Envelope')

% Operational direct vs split
is_direct = route_type == 1;
is_split  = route_type == 2;

plot(dist_oper_km(is_direct), payload_oper_kg(is_direct)/1000, 'md', ...
    'MarkerFaceColor', 'm', ...
    'DisplayName', 'Operational Direct (122.7t)')

plot(dist_oper_km(is_split), payload_oper_kg(is_split)/1000, 'cd', ...
    'MarkerFaceColor', 'c', ...
    'DisplayName', 'Operational Split (1 stop)')

% Design point
plot(R_design_km, payload_oper/1000, 'bp', ...
    'MarkerSize', 12, ...
    'LineWidth', 2, ...
    'DisplayName', 'Design Point')

% Design range line
xline(R_design_km, '--b', 'Design Range', ...
    'LabelVerticalAlignment', 'bottom')

xlabel('Range (km)')
ylabel('Payload (tonnes)')
title('Mission-Based Payload–Range Envelope and Operational Proof')
legend('Location','northeast')

xlim([0, 1.05*max([ranges_km; dist_km])])
ylim([0, 1.10*max(payloads_t)])

%% ==================== LOCAL FUNCTIONS ====================

function R_max = range_for_payload_mission(payload, ADP, MTOM, OEM, maxFuel, mission_fun)

    fuel_available = min(maxFuel, MTOM - OEM - payload);

    if fuel_available <= 0
        R_max = 0;
        return
    end

    fun = @(R) fuel_required_for_range_mission(R, payload, ADP, OEM, mission_fun) ...
             - fuel_available;

    R_lo = 1;       % m
    f_lo = fun(R_lo);

    if ~isfinite(f_lo)
        R_max = NaN;
        return
    end

    if f_lo > 0
        R_max = 0;
        return
    end

    % Expand upper bracket until root is bracketed
    R_hi = 1e6;   % 1000 km
    f_hi = fun(R_hi);

    while isfinite(f_hi) && f_hi < 0 && R_hi < 3e7
        R_hi = 1.5 * R_hi;
        f_hi = fun(R_hi);
    end

    if ~isfinite(f_hi)
        R_max = NaN;
        return
    end

    if f_hi < 0
        R_max = R_hi;
        return
    end

    R_max = fzero(fun, [R_lo, R_hi]);
end

function [payload_sol, feasible] = payload_for_range_mission(R_target, ADP, MTOM, OEM, maxPayload_struct, maxFuel, mission_fun)

    fuel_available = @(p) min(maxFuel, MTOM - OEM - p);
    fun = @(p) fuel_required_for_range_mission(R_target, p, ADP, OEM, mission_fun) ...
             - fuel_available(p);

    f0 = fun(0);
    f1 = fun(maxPayload_struct);

    if ~isfinite(f0) || (f0 > 0)
        payload_sol = 0;
        feasible = false;
        return
    end

    if isfinite(f1) && (f1 <= 0)
        payload_sol = maxPayload_struct;
        feasible = true;
        return
    end

    payload_sol = fzero(fun, [0, maxPayload_struct]);
    payload_sol = max(payload_sol, 0);
    feasible = true;
end

function fuel_req = fuel_required_for_range_mission(R, payload, ADP, OEM, mission_fun)
    % Fixed-point iteration:
    % fuel depends on M_TO, but M_TO depends on fuel.

    fuel_guess = 1e5;   % kg initial guess

    for k = 1:8
        M_TO = OEM + payload + fuel_guess;

        [BlockFuel, ok] = call_mission_analysis_silently(mission_fun, ADP, R, M_TO);

        if ~ok || ~isfinite(BlockFuel) || BlockFuel < 0
            fuel_req = Inf;
            return
        end

        if abs(BlockFuel - fuel_guess) < max(1.0, 1e-3*fuel_guess)
            fuel_guess = BlockFuel;
            break
        end

        fuel_guess = 0.5 * fuel_guess + 0.5 * BlockFuel;   % damping
    end

    fuel_req = fuel_guess;
end

function [BlockFuel, ok] = call_mission_analysis_silently(mission_fun, ADP, tripRange, M_TO)

    BlockFuel = Inf;
    ok = false;

    try
        evalc('[BlockFuel,~,~,~,~,~] = mission_fun(ADP, tripRange, M_TO);');
        ok = true;
    catch
        ok = false;
    end
end

% 
% 
% 
% Index exceeds the number of array elements. Index must not exceed 12.
% 
% Error in MissionAnalysis (line 92)
% CL_c = CL_c(idx);
% 
% Error in @(ADP,tripRange,M_TO)MissionAnalysis(ADP,tripRange,M_TO) (line 80)
% mission_fun = @(ADP,tripRange,M_TO) MissionAnalysis(ADP,tripRange,M_TO);
% 
% Error in SimplifiedPayloadRange4>call_mission_analysis_silently (line 314)
%         evalc('[BlockFuel,~,~,~,~,~] = mission_fun(ADP, tripRange, M_TO);');
% 
% Error in SimplifiedPayloadRange4>fuel_required_for_range_mission (line 294)
%         [BlockFuel, ok] = call_mission_analysis_silently(mission_fun, ADP, R, M_TO);
% 
% Error in range_for_payload_mission>@(R)fuel_required_for_range_mission(R,payload,ADP,OEM,mission_fun)-fuel_available (line 243)
%     fun = @(R) fuel_required_for_range_mission(R, payload, ADP, OEM, mission_fun) ...
% 
% Error in SimplifiedPayloadRange4>range_for_payload_mission (line 247)
%     f_lo = fun(R_lo);
% 
% Error in SimplifiedPayloadRange4 (line 91)
%     range_env_m(i) = range_for_payload_mission( ...
 

