%% Payload-Range Diagram (Unconventional Aircraft)
% Full fixed version
% - Enforces a maximum fuel-capacity limit
% - Produces the standard A-B-C-D payload-range envelope
% - Uses existing mission analysis without modifying it

clear
clc
close all

%% =========================
% 1. Build & freeze aircraft
% =========================
scripts.ExampleUnconventional   % must create ADP

MTOM  = ADP.MTOM;           % [kg]
OEM   = ADP.OEM;            % [kg]
Pmax  = ADP.TLAR.Payload;   % [kg]
Rdes  = ADP.TLAR.RangeDes;  % [m]

fprintf('\n--- Aircraft Summary ---\n');
fprintf('MTOM: %.1f t\n', MTOM/1e3);
fprintf('OEM : %.1f t\n', OEM/1e3);
fprintf('Max payload: %.1f t\n', Pmax/1e3);
fprintf('Design range: %.0f km\n', Rdes/1e3);

%% =========================
% 2. Estimate design-mission fuel
% =========================
% This is the fuel actually required at the sized design point
Fuel_design = fuel_used(Rdes, ADP, Pmax, OEM);

%% =========================
% 3. Define maximum fuel capacity
% =========================
% IMPORTANT:
% Replace this with your real tank-capacity-based fuel mass when available.
% For now, use a reasonable assumption above design mission fuel.
fuelCapacityFactor = 1.20;     % <-- adjust if needed
MaxFuelMass = fuelCapacityFactor * Fuel_design;

% Structural sanity cap
MaxFuelMass = min(MaxFuelMass, MTOM - OEM);

fprintf('Design mission fuel: %.1f t\n', Fuel_design/1e3);
fprintf('Assumed max fuel capacity: %.1f t\n', MaxFuelMass/1e3);

%% =========================
% 4. Key payload-range points
% =========================
% Point A: zero range, maximum payload
RA = 0;
PA = Pmax;

% Point B: maximum payload range
% At max payload, fuel is limited by both MTOM and tank capacity
Fuel_B = min(MaxFuelMass, MTOM - OEM - Pmax);
Fuel_B = max(Fuel_B, 0);
RB = solve_range_for_fuel(Fuel_B, ADP, Pmax, OEM);
PB = Pmax;

% Point C: full tanks, reduced payload, still at MTOM
PC = max(MTOM - OEM - MaxFuelMass, 0);
Fuel_C = min(MaxFuelMass, MTOM - OEM - PC);
Fuel_C = max(Fuel_C, 0);
RC = solve_range_for_fuel(Fuel_C, ADP, PC, OEM);

% Point D: ferry range (zero payload, full tanks)
PD = 0;
Fuel_D = min(MaxFuelMass, MTOM - OEM);
Fuel_D = max(Fuel_D, 0);
RD = solve_range_for_fuel(Fuel_D, ADP, PD, OEM);

%% =========================
% 5. Build plotting arrays
% =========================
range_km   = [RA, RB, RC, RD] / 1000;
payload_t  = [PA, PB, PC, PD] / 1000;

%% =========================
% 6. Plot payload-range diagram
% =========================
figure('Color','w');
plot(range_km, payload_t, 'b-', 'LineWidth', 2); hold on;

plot(RB/1000, PB/1000, 'ro', ...
    'MarkerSize', 8, ...
    'MarkerFaceColor', 'none', ...
    'DisplayName', 'Max Payload');

plot(RD/1000, PD/1000, 'ks', ...
    'MarkerSize', 8, ...
    'MarkerFaceColor', 'none', ...
    'DisplayName', 'Ferry Range');

xlabel('Range [km]');
ylabel('Payload [tonnes]');
title('Payload–Range Diagram (Unconventional Aircraft)');
grid on;
box on;
set(gca, 'FontSize', 12);

legend('Payload–Range Envelope', 'Max Payload', 'Ferry Range', ...
    'Location', 'northeast');

xlim([0, 1.05*max(range_km)]);
ylim([0, 1.10*max(payload_t)]);

%% =========================
% 7. Print results
% =========================
fprintf('\n--- Payload-Range Results ---\n');
fprintf('Point A: zero-range payload = %.1f t\n', PA/1e3);
fprintf('Point B: max payload range = %.0f km\n', RB/1e3);
fprintf('Point C: payload at full tanks = %.1f t\n', PC/1e3);
fprintf('Point C: range at full tanks = %.0f km\n', RC/1e3);
fprintf('Point D: ferry range = %.0f km\n', RD/1e3);

%% =========================
% 8. Helper functions
% =========================
function fuel = fuel_used(range, ADP, payload, OEM)
% Computes required fuel for given range and payload
%
% range   [m]
% payload [kg]
% OEM     [kg]
% fuel    [kg]

    % Better initial mass guess than using full MTOM every time
    mass_guess = OEM + payload + 0.5 * max(ADP.MTOM - OEM - payload, 0);

    fuel = 0;

    % Iterate to resolve mass-fuel coupling
    for k = 1:10
        [~, TripFuel, ResFuel, ~, ~] = ...
            Unconventional.MissionAnalysis_oscar(ADP, range, mass_guess);

        fuel_new = TripFuel + ResFuel;
        mass_new = OEM + payload + fuel_new;

        % convergence check
        if abs(mass_new - mass_guess) < 1e-4 * max(mass_guess, 1)
            fuel = fuel_new;
            return;
        end

        fuel = fuel_new;
        mass_guess = mass_new;
    end
end

function R_sol = solve_range_for_fuel(Fuel_target, ADP, payload, OEM)
% Solves for range such that required fuel equals Fuel_target

    if Fuel_target <= 0
        R_sol = 0;
        return;
    end

    fuel_error = @(R) fuel_used(R, ADP, payload, OEM) - Fuel_target;

    % Lower bracket
    R_lo = 0;

    % Upper bracket starts near design range
    R_hi = max(ADP.TLAR.RangeDes, 1000);

    % Expand until target is bracketed
    maxExpand = 40;
    count = 0;

    while fuel_error(R_hi) < 0 && count < maxExpand
        R_hi = 1.5 * R_hi;
        count = count + 1;
    end

    % If still not bracketed, return NaN instead of crashing
    if fuel_error(R_lo) * fuel_error(R_hi) > 0
        warning('Could not bracket range solution for payload %.1f t.', payload/1e3);
        R_sol = NaN;
        return;
    end

    R_sol = fzero(fuel_error, [R_lo, R_hi]);
    R_sol = max(R_sol, 0);
end