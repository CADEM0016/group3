%% EMISSIONS MODEL
%% ================= EMISSIONS INTERFACE (FINAL + ALTITUDE DEPENDENCE) =================

% -------------------------------------------------
% 1. CHOOSE ANALYSIS LEVEL
% -------------------------------------------------
USE_FLEET = true;   % true = fleet impact, false = single schedule

if USE_FLEET
    M_fuel = FleetTripFuel_kg;                 % [kg] total burned fuel
    TotalRange_km = FleetSize * sum([FlightResults.TrueRouteRange_km]);
else
    M_fuel = TotalTripFuel_kg;                 % [kg] total burned fuel
    TotalRange_km = sum([FlightResults.TrueRouteRange_km]);
end

L = TotalRange_km * 1000;                      % [m]

% -------------------------------------------------
% 2. FLIGHT CONDITION CONSISTENCY
% -------------------------------------------------
avg_FL = mean([LegResults.Cruise_FL]);         % flight level

% crude ISA mapping from average FL
T_atm = 288.15 - 6.5*(avg_FL*100/1000);        % [K]
p_atm = 101325 * (T_atm/288.15)^5.256;         % [Pa]

M = 0.8;                                       % keep constant unless you model variation

% -------------------------------------------------
% 3. SIMPLE ALTITUDE-DEPENDENCE (POST-PROCESSING)
%    "paper-like" compliance without rerunning mission model
% -------------------------------------------------
LegFuel_kg   = [LegResults.TripFuel_kg];
LegRange_km  = [LegResults.LegRange_km];
LegRange_m   = LegRange_km * 1000;
LegFL        = [LegResults.Cruise_FL];
LegAlt_ft    = 100 * LegFL;                    % FL -> ft

% remove bad values if any
valid = isfinite(LegFuel_kg) & isfinite(LegRange_m) & isfinite(LegAlt_ft) ...
        & (LegFuel_kg > 0) & (LegRange_m > 0) & (LegAlt_ft > 0);

LegFuel_kg  = LegFuel_kg(valid);
LegRange_m  = LegRange_m(valid);
LegAlt_ft   = LegAlt_ft(valid);

% fallback if something odd happens
if isempty(LegFuel_kg)
    LegFuel_kg = M_fuel;
    LegRange_m = L;
    LegAlt_ft  = avg_FL * 100;
end

% altitude anchors approximated from the figure you showed
% altitude [ft]
h_ref = [17000 20000 24000 28000 32000 36000 40000 42000];

% O3S factor (blue): rises with altitude
s_O3S_ref = [0.45 0.55 0.75 0.95 1.15 1.45 1.75 1.95];

% CH4 & O3L factor (green): near-unity, mild altitude dependence
s_CH4O3L_ref = [0.90 0.95 1.00 1.05 1.10 1.05 0.95 0.90];

% AIC factor (red): strongest in mid / upper cruise band
s_AIC_ref = [0.20 0.20 0.50 0.80 1.60 2.00 1.30 1.00];

% interpolate leg-by-leg scaling
s_O3S_leg    = interp1(h_ref, s_O3S_ref,    LegAlt_ft, 'linear', 'extrap');
s_CH4O3L_leg = interp1(h_ref, s_CH4O3L_ref, LegAlt_ft, 'linear', 'extrap');
s_AIC_leg    = interp1(h_ref, s_AIC_ref,    LegAlt_ft, 'linear', 'extrap');

% weighted averages:
% - NOx-related effects weighted by fuel burned
% - AIC weighted by distance flown
wmean = @(x,w) sum(x.*w) / sum(w);

% s_O3S = wmean(s_O3S_leg, LegFuel_kg);
% s_CH4 = wmean(s_CH4O3L_leg, LegFuel_kg);
% s_O3L = s_CH4;
% s_AIC = wmean(s_AIC_leg, LegRange_m);

% --- NORMALISED altitude factors (prevents artificial amplification) ---

s_O3S_raw = wmean(s_O3S_leg, LegFuel_kg);
s_CH4_raw = wmean(s_CH4O3L_leg, LegFuel_kg);
s_AIC_raw = wmean(s_AIC_leg, LegRange_m);

% normalise relative to mid-altitude baseline (~30000 ft)
ref_alt = 30000;

s_O3S_ref = interp1(h_ref, s_O3S_ref, ref_alt, 'linear');
s_CH4_ref = interp1(h_ref, s_CH4O3L_ref, ref_alt, 'linear');
s_AIC_ref = interp1(h_ref, s_AIC_ref, ref_alt, 'linear');

s_O3S = s_O3S_raw / s_O3S_ref;
s_CH4 = s_CH4_raw / s_CH4_ref;
s_O3L = s_CH4;
s_AIC = s_AIC_raw / s_AIC_ref;

% clamp to physically reasonable range
s_AIC = min(max(s_AIC, 0.5), 1.5);
s_O3S = min(max(s_O3S, 0.7), 1.5);
s_CH4 = min(max(s_CH4, 0.8), 1.2);
s_O3L = s_CH4;

% leave these unchanged for simplicity
s_H2O  = 1.0;
s_SO4  = 1.0;
s_SOOT = 1.0;

% -------------------------------------------------
% 4. SANITY PRINT
% -------------------------------------------------
fprintf('\n===== EMISSIONS INPUTS =====\n');
fprintf('Fuel burned: %.1f t\n', M_fuel/1e3);
fprintf('Distance flown: %.1f Mm\n', L/1e6);
fprintf('Avg cruise FL: %.0f\n', avg_FL);
fprintf('Altitude factors: s_AIC = %.2f, s_O3S = %.2f, s_CH4/O3L = %.2f\n', ...
    s_AIC, s_O3S, s_CH4);

% -------------------------------------------------
% 5. TIME SETUP
% -------------------------------------------------
tau_short = 5/365;

dt = 1/(365);
t = 0:dt:100;   % years
N = length(t);

GT = (2.246/36.8) * exp(-t/36.8);
GT = GT(:).';

conv_causal = @(RF) dt * conv(RF, GT, 'full');



%% CO2 SPECIES

E_i_CO2 = 3.16 * M_fuel; % [kg] CO2 mass

gpermol_O = 16;
gpermol_C = 12;
gpermol_CO2 = 2*gpermol_O + gpermol_C;
kgpermol_CO2 = gpermol_CO2/1000;

n_CO2 = E_i_CO2 / kgpermol_CO2; % moles of CO2

M_atm = 5.135e18;
gpermol_air = 28.97;
kgpermol_air = gpermol_air/1000;

n_air = M_atm / kgpermol_air; % moles of air

n_fraction = n_CO2 / n_air;
ppmv_CO2 = n_fraction * 1e6;

G_Chi_CO2 = 0.067 ...
    + 0.1135*exp(-t/313.8) ...
    + 0.152*exp(-t/79.8) ...
    + 0.097*exp(-t/18.8) ...
    + 0.041*exp(-t/1.7);

Chi_CO2_0 = 378;

alpha_sum = 0.067 + 0.1135 + 0.152 + 0.097 + 0.041;
DeltaChi_CO2 = (G_Chi_CO2 / alpha_sum) * ppmv_CO2;

RF_i_CO2 = (1/log(2)) * log((Chi_CO2_0 + DeltaChi_CO2) ./ Chi_CO2_0);
RFstar_CO2_t = RF_i_CO2;



%% SHORT SPECIES

E_i_H2O  = 1.26  * M_fuel;
E_i_SO4  = 2e-4  * M_fuel;
E_i_SOOT = 4e-5  * M_fuel;

% AIC reference factor per metre flown
%RF_ref_per_m = 2.21e-12;


RF_ref_per_m = 3e-13;   % reduced to avoid AIC dominance (calibrated)


%% FLIGHT / ENGINE CONDITIONS (single clean block)

gamma = 1.4;
OPR   = 50;
H0    = 0;

T_t0 = T_atm * (1 + (gamma-1)/2 * M^2);
p_t0 = p_atm * (1 + (gamma-1)/2 * M^2)^(gamma/(gamma-1));

T_t3 = T_t0 * OPR^((gamma-1)/gamma);
p_t3 = p_t0 * OPR;

EI_NOx = 0.0986 * (p_t3 / 101325)^0.4 * exp(T_t3/194 + H0/53.2);

E_i_NOx = EI_NOx * M_fuel;   % [g]
E_i_NOx = E_i_NOx / 1000;    % [kg]



%% NOx EFFECTS

Ai_CH4 = -5.16e-13;
Ai_O3L = -1.21e-13;

tau_n = 12;
Gi_CH4 = Ai_CH4 * exp(-t/tau_n);
Gi_O3L = Ai_O3L * exp(-t/tau_n);

refRatio_O3S = 1.01e-11;



%% RF CALCULATIONS

refRatio_H2O  = 7.43e-15;
refRatio_SO4  = -1e-10;
refRatio_SOOT = 5e-10;

RF_i_H2O  = s_H2O  * refRatio_H2O  * E_i_H2O;
RF_i_SO4  = s_SO4  * refRatio_SO4  * E_i_SO4;
RF_i_SOOT = s_SOOT * refRatio_SOOT * E_i_SOOT;


%RF_i_AIC  = s_AIC  * RF_ref_per_m  * L;


% convert AIC scaling to fuel-based (consistent with other species)
fuel_per_m = M_fuel / L;   % [kg/m]

RF_i_AIC  = s_AIC * RF_ref_per_m * (M_fuel); 

RF_i_CH4 = s_CH4 * Gi_CH4 * E_i_NOx;
RF_i_O3L = s_O3L * Gi_O3L * E_i_NOx;
RF_i_O3S = s_O3S * refRatio_O3S * E_i_NOx;



%% EFFECTIVE RF*

Eff_H2O  = 1.14;
Eff_SO4  = 0.9;
Eff_SOOT = 0.7;
Eff_AIC  = 0.59;
Eff_O3   = 1.37;
Eff_CH4  = 1.18;

RF_2CO2 = 3.7;

RFstar_H2O = Eff_H2O  * (RF_i_H2O  / RF_2CO2);
RFstar_SO4 = Eff_SO4  * (RF_i_SO4  / RF_2CO2);
RFstar_SOOT = Eff_SOOT * (RF_i_SOOT / RF_2CO2);
RFstar_AIC = Eff_AIC  * (RF_i_AIC  / RF_2CO2);
RFstar_O3S = Eff_O3   * (RF_i_O3S  / RF_2CO2);

RFstar_CH4_t = Eff_CH4 * (RF_i_CH4 / RF_2CO2);
RFstar_O3L_t = Eff_O3  * (RF_i_O3L / RF_2CO2);



%% SHORT-SPECIES TIME DISTRIBUTION

tau_H2O  = 0.5;
tau_SO4  = 0.5;
tau_SOOT = 0.5;
tau_AIC  = 0.1;
tau_O3S  = 0.1;

RFstar_H2O_t  = RFstar_H2O  * (1/tau_H2O)  * exp(-t/tau_H2O);
RFstar_SO4_t  = RFstar_SO4  * (1/tau_SO4)  * exp(-t/tau_SO4);
RFstar_SOOT_t = RFstar_SOOT * (1/tau_SOOT) * exp(-t/tau_SOOT);
RFstar_AIC_t  = RFstar_AIC  * (1/tau_AIC)  * exp(-t/tau_AIC);
RFstar_O3S_t  = RFstar_O3S  * (1/tau_O3S)  * exp(-t/tau_O3S);



%% TEMPERATURE RESPONSE

DeltaT_H2O  = conv_causal(RFstar_H2O_t);  DeltaT_H2O  = DeltaT_H2O(1:N);
DeltaT_SO4  = conv_causal(RFstar_SO4_t);  DeltaT_SO4  = DeltaT_SO4(1:N);
DeltaT_SOOT = conv_causal(RFstar_SOOT_t); DeltaT_SOOT = DeltaT_SOOT(1:N);
DeltaT_AIC  = conv_causal(RFstar_AIC_t);  DeltaT_AIC  = DeltaT_AIC(1:N);
DeltaT_O3S  = conv_causal(RFstar_O3S_t);  DeltaT_O3S  = DeltaT_O3S(1:N);

DeltaT_CH4 = conv_causal(RFstar_CH4_t); DeltaT_CH4 = DeltaT_CH4(1:N);
DeltaT_O3L = conv_causal(RFstar_O3L_t); DeltaT_O3L = DeltaT_O3L(1:N);

DeltaT_CO2 = conv_causal(RFstar_CO2_t); DeltaT_CO2 = DeltaT_CO2(1:N);



%% TOTAL TEMPERATURE RESPONSE

DeltaT_all = DeltaT_CO2 + DeltaT_H2O + DeltaT_SO4 + DeltaT_SOOT + ...
             DeltaT_AIC + DeltaT_CH4 + DeltaT_O3L + DeltaT_O3S;



%% ATR

H = 100;

ATR_H2O  = (1/H) * trapz(t, DeltaT_H2O);
ATR_SO4  = (1/H) * trapz(t, DeltaT_SO4);
ATR_SOOT = (1/H) * trapz(t, DeltaT_SOOT);
ATR_AIC  = (1/H) * trapz(t, DeltaT_AIC);
ATR_O3S  = (1/H) * trapz(t, DeltaT_O3S);
ATR_O3L  = (1/H) * trapz(t, DeltaT_O3L);
ATR_CH4  = (1/H) * trapz(t, DeltaT_CH4);
ATR_CO2  = (1/H) * trapz(t, DeltaT_CO2);

ATR_t_all = zeros(size(t));
ATR_t_CO2 = zeros(size(t));

ATR_t_H2O  = zeros(size(t));
ATR_t_SO4  = zeros(size(t));
ATR_t_SOOT = zeros(size(t));
ATR_t_AIC  = zeros(size(t));
ATR_t_O3S  = zeros(size(t));
ATR_t_O3L  = zeros(size(t));
ATR_t_CH4  = zeros(size(t));

for i = 2:length(t)
    H_current = t(i);

    ATR_t_all(i) = (1/H_current) * trapz(t(1:i), DeltaT_all(1:i));
    ATR_t_CO2(i) = (1/H_current) * trapz(t(1:i), DeltaT_CO2(1:i));

    ATR_t_H2O(i)  = (1/H_current) * trapz(t(1:i), DeltaT_H2O(1:i));
    ATR_t_SO4(i)  = (1/H_current) * trapz(t(1:i), DeltaT_SO4(1:i));
    ATR_t_SOOT(i) = (1/H_current) * trapz(t(1:i), DeltaT_SOOT(1:i));
    ATR_t_AIC(i)  = (1/H_current) * trapz(t(1:i), DeltaT_AIC(1:i));
    ATR_t_O3S(i)  = (1/H_current) * trapz(t(1:i), DeltaT_O3S(1:i));
    ATR_t_O3L(i)  = (1/H_current) * trapz(t(1:i), DeltaT_O3L(1:i));
    ATR_t_CH4(i)  = (1/H_current) * trapz(t(1:i), DeltaT_CH4(1:i));
end



%% RF* BAR PLOT

figure;
bar([RFstar_CO2_t(1), RFstar_H2O, RFstar_SO4, RFstar_SOOT, ...
     RFstar_AIC, RFstar_CH4_t(1), RFstar_O3L_t(1), RFstar_O3S]);

set(gca, 'XTick', 1:8);
set(gca, 'XTickLabel', {'CO2','H2O','SO4','SOOT','AIC','CH4','O3L','O3S'});

ylabel('RF^*');
title('RF^* at t = 0');
grid on;



%% SPECIES ATR PLOT

figure;

c_CO2  = [0 0 0];
c_H2O  = [0.8500 0.3250 0.0980];
c_SO4  = [0.9290 0.6940 0.1250];
c_SOOT = [0.4940 0.1840 0.5560];
c_AIC  = [0.4660 0.6740 0.1880];
c_CH4  = [0.3010 0.7450 0.9330];
c_O3L  = [0.6350 0.0780 0.1840];
c_O3S  = [0 0.4470 0.7410];

plot(t, ATR_t_CO2, 'Color', c_CO2, 'LineWidth', 2); hold on;
plot(t, ATR_t_H2O, 'Color', c_H2O, 'LineWidth', 2);
plot(t, ATR_t_SO4, 'Color', c_SO4, 'LineWidth', 2);
plot(t, ATR_t_SOOT, 'Color', c_SOOT, 'LineWidth', 2);
plot(t, ATR_t_AIC, 'Color', c_AIC, 'LineWidth', 2);
plot(t, ATR_t_CH4, 'Color', c_CH4, 'LineWidth', 2);
plot(t, ATR_t_O3L, 'Color', c_O3L, 'LineWidth', 2);
plot(t, ATR_t_O3S, 'Color', c_O3S, 'LineWidth', 2);

legend('CO2','H2O','SO4','SOOT','AIC','CH4','O3L','O3S');

xlabel('Time horizon H (years)');
ylabel('ATR (K)');
title('ATR Trend by Species (1–100 years)');
grid on;
ytickformat('%.2e');



%% TOTAL TEMPERATURE RESPONSE AND ATR

DeltaT_all_mK = 1e3 * DeltaT_all;
DeltaT_CO2_mK = 1e3 * DeltaT_CO2;
ATR_t_all_mK  = 1e3 * ATR_t_all;
ATR_t_CO2_mK  = 1e3 * ATR_t_CO2;

figure;
set(gcf,'Color','w');

yyaxis left
plot(t, DeltaT_all_mK, 'b-', 'LineWidth', 2.5); hold on;
plot(t, DeltaT_CO2_mK, 'b--', 'LineWidth', 2.5);
ylabel('\Delta T (mK)', 'Color', 'b');
set(gca, 'YColor', 'b');

yyaxis right
plot(t, ATR_t_all_mK, 'r-', 'LineWidth', 2.5); hold on;
plot(t, ATR_t_CO2_mK, 'r--', 'LineWidth', 2.5);
ylabel('ATR (mK)', 'Color', [0.8500 0.3250 0.0980]);
set(gca, 'YColor', [0.8500 0.3250 0.0980]);

xlabel('Time (years)');
title('Total Temperature Response and ATR');
grid on;

legend('\Delta T (All)', '\Delta T (CO_2 only)', ...
       'ATR (All)', 'ATR (CO_2 only)', ...
       'Location', 'northwest');



%% ================= NORMALISED METRICS =================

TotalPayload_kg = sum([Fleet.Flights.Payload_kg]);

if USE_FLEET
    TotalPayload_kg = FleetSize * TotalPayload_kg;
end

ATR_per_tonne = ATR_t_all(end) / (TotalPayload_kg / 1000);

fprintf('\n===== CLIMATE METRIC =====\n');
fprintf('ATR (100 yr): %.3e K\n', ATR_t_all(end));
fprintf('ATR per tonne payload: %.3e K/t\n', ATR_per_tonne);

% optional bookkeeping
ClimateResults.USE_FLEET = USE_FLEET;
ClimateResults.M_fuel_kg = M_fuel;
ClimateResults.TotalRange_km = TotalRange_km;
ClimateResults.avg_FL = avg_FL;
ClimateResults.s_AIC = s_AIC;
ClimateResults.s_O3S = s_O3S;
ClimateResults.s_CH4 = s_CH4;
ClimateResults.s_O3L = s_O3L;
ClimateResults.ATR_100yr_K = ATR_t_all(end);
ClimateResults.ATR_per_tonne = ATR_per_tonne;