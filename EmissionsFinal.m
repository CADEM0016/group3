%% EMISSIONS MODEL



%% ================= CLIMATE PROFILE FROM EXISTING MISSION CALL =================
if ~exist('tripRange','var') || isempty(tripRange)
    tripRange = ADP.TLAR.RangeDes;
end

if ~exist('M_TO','var') || isempty(M_TO)
    M_TO = ADP.MTOM;
end

[h_profile_ft, R_profile_m, climateMeta] = buildClimateProfileFromMissionCall(ADP, tripRange, M_TO);

% Build explicit segment-wise mission climate profile
seg = buildClimateSegments(h_profile_ft, R_profile_m);

% Segment forcing factors
s_AIC_seg = get_s_AIC(seg.h_mid_ft);
s_O3S_seg = get_s_O3S(seg.h_mid_ft);
s_CH4_seg = get_s_CH4_O3L(seg.h_mid_ft);
s_O3L_seg = s_CH4_seg;

% Diagnostic averages only
s_AIC = sum(s_AIC_seg .* seg.dR_m) / sum(seg.dR_m);
s_O3S = sum(s_O3S_seg .* seg.dR_m) / sum(seg.dR_m);
s_CH4 = sum(s_CH4_seg .* seg.dR_m) / sum(seg.dR_m);
s_O3L = s_CH4;

fprintf('\n===== CLIMATE PROFILE / FORCING FACTORS =====\n');
fprintf('Cruise FL used       : %.0f\n', climateMeta.cruise_FL);
fprintf('Cruise altitude      : %.0f ft\n', climateMeta.h_cruise_ft);
fprintf('Climb range          : %.1f km\n', climateMeta.climb_range_m/1000);
fprintf('Cruise range         : %.1f km\n', climateMeta.cruise_range_m/1000);
fprintf('Descent range        : %.1f km\n', climateMeta.descent_range_m/1000);
fprintf('s_AIC avg            : %.3f\n', s_AIC);
fprintf('s_O3S avg            : %.3f\n', s_O3S);
fprintf('s_CH4_O3L avg        : %.3f\n', s_CH4);

plotForcingFactorGraph(h_profile_ft, R_profile_m);

%% ================= EMISSIONS INTERFACE (FINAL) =================

% -------------------------------------------------
% 1. CHOOSE ANALYSIS LEVEL
% -------------------------------------------------
USE_FLEET = true;   % true = fleet impact, false = single schedule

if USE_FLEET
    M_fuel = FleetTripFuel_kg;        % [kg]
    TotalRange_km = FleetSize * sum([FlightResults.TrueRouteRange_km]);
else
    M_fuel = TotalTripFuel_kg;        % [kg]
    TotalRange_km = sum([FlightResults.TrueRouteRange_km]);
end

L_m = TotalRange_km * 1000;             % [m]
L_nm = TotalRange_km / 1.852;         % [nmi]

% -------------------------------------------------
% 2. FLIGHT CONDITION CONSISTENCY (IMPORTANT FIX)
% -------------------------------------------------
% Use actual mission outputs instead of hardcoding

avg_FL = mean([LegResults.Cruise_FL]);   % flight level

% crude ISA mapping (good enough for conceptual model)
T_atm = 288.15 - 6.5*(avg_FL*100/1000);   % [K]
p_atm = 101325 * (T_atm/288.15)^5.256;    % [Pa]

M = 0.8;   % keep constant unless you model variation

% -------------------------------------------------
% 3. SANITY PRINT
% -------------------------------------------------
fprintf('\n===== EMISSIONS INPUTS =====\n');
fprintf('Fuel burned: %.1f t\n', M_fuel/1e3);
fprintf('Distance flown: %.1f Mm\n', L_m/1e6);
fprintf('Avg cruise FL: %.0f\n', avg_FL);






%M_fuel = 15*10^3;
%M_fuel = TotalBurn;
% tau_short = 0.011107635;
tau_short = 5/365;

dt = 1/(365);
t = 0:dt:100;   % years
N = length(t);

GT = (2.246/36.8) * exp(-t/36.8);
GT = GT(:).';

conv_causal = @(RF) dt * conv(RF, GT, 'full');



%% CO2 SPECIES

E_i_CO2 = 3.16*M_fuel; % CO2 Mass

gpermol_O = 16;
gpermol_C = 12;
gpermol_CO2 = 2*gpermol_O + gpermol_C;
kgpermol_CO2 = gpermol_CO2/1000;

n_CO2 = E_i_CO2/kgpermol_CO2; % moles of CO2

M_atm = 5.135e18;
gpermol_air = 28.97;
kgpermol_air = gpermol_air/1000;

n_air = M_atm/kgpermol_air; % moles of air

n_fraction = n_CO2/n_air;
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

E_i_H2O = 1.26*M_fuel;
E_i_SO4 = 2e-4*M_fuel;
E_i_SOOT = 4e-5*M_fuel;

%L = TotalRange; % mission distance [m]

% RF_ref_per_L = 2.21e-12;
% RF_ref_per_L_km = RF_ref_per_L/(1.852e3);

%s_AIC = 1;

RF_ref_per_L = 2.21e-12;   % (W/m^2)/nmi



%% FLIGHT / ENGINE CONDITIONS

gamma = 1.4;
M = 0.8;
T_atm = 220.9;
p_atm = 25940;

OPR = 50;
H0 = 0;

T_t0 = T_atm * (1 + (gamma-1)/2 * M^2);
p_t0 = p_atm * (1 + (gamma-1)/2 * M^2)^(gamma/(gamma-1));

T_t3 = T_t0 * OPR^((gamma-1)/gamma);
p_t3 = p_t0 * OPR;

EI_NOx = 0.0986 * (p_t3 / 101325)^0.4 * exp(T_t3/194 + H0/53.2);
E_i_NOx = EI_NOx * M_fuel;   % g
E_i_NOx = E_i_NOx / 1000;    % kg

%% FLIGHT / ENGINE CONDITIONS
gamma = 1.4;
M = 0.8;
T_atm = 220.9;
p_atm = 25940;
OPR_XWB = 50;
H0 = 0;
T_t0 = T_atm * (1 + (gamma-1)/2 * M^2);
p_t0 = p_atm * (1 + (gamma-1)/2 * M^2)^(gamma/(gamma-1));
T_t3_XWB = T_t0 * OPR_XWB^((gamma-1)/gamma);
p_t3_XWB = p_t0 * OPR_XWB;
EI_NOx = 0.0986 * (p_t3_XWB / 101325)^0.4 * exp(T_t3_XWB/194 + H0/53.2);
E_i_NOx = EI_NOx * M_fuel;   % g
E_i_NOx = E_i_NOx / 1000;    % kg
LHV_AF = 43*10^6; %LVH = Lower heating value & net calorific value
%https://www.engineeringtoolbox.com/fuels-higher-calorific-values-d_169.html
Est_Fuel2Air = (1/60); % fuel to air ratio from raymer multiplied by 1.5 for some reason!!!!
eta_combu = 1; %combustion efficiency - assuemd get proper justifications
C_p = 1004; %classic thermal value recomment it later
T4_XWB = T_t3_XWB+((Est_Fuel2Air*eta_combu*LHV_AF)/C_p); %final necessary temperature value
EmpiricalCorrectionFactor = 1745/T4_XWB; %data from the slovak paper
%% FLIGHT / ENGINE CONDITIONS
OPR = 60;
H0 = 0;
T_t0 = T_atm * (1 + (gamma-1)/2 * M^2);
p_t0 = p_atm * (1 + (gamma-1)/2 * M^2)^(gamma/(gamma-1));
T_t3 = T_t0 * OPR^((gamma-1)/gamma);
p_t3 = p_t0 * OPR;
EI_NOx = 0.0986 * (p_t3 / 101325)^0.4 * exp(T_t3/194 + H0/53.2);
E_i_NOx = EI_NOx * M_fuel;   % g
E_i_NOx = E_i_NOx / 1000;    % kg
LHV_AF = 43*10^6; %LVH = Lower heating value & net calorific value
%https://www.engineeringtoolbox.com/fuels-higher-calorific-values-d_169.html
Fuel2Air = (1/60); % fuel to air ratio from raymer multiplied by 1.5 for some reason!!!!
eta_combu = 1; %combustion efficiency - assuemd get proper justifications
C_p = 1004; %classic thermal value recomment it later
T4 = T_t3+((Fuel2Air*eta_combu*LHV_AF)/C_p); %final necessary temperature value
T4_corrected_UF = T4*EmpiricalCorrectionFactor;

%% NOx EFFECTS

Ai_CH4 = -5.16e-13;
Ai_O3L = -1.21e-13;

tau_n = 12;
Gi_CH4 = Ai_CH4 * exp(-t/tau_n);
Gi_O3L = Ai_O3L * exp(-t/tau_n);

% s_CH4 = 1;
% s_O3L = 1;
% s_O3S = 1;


%s_CH4 = FF.s_CH4_O3L;
%s_O3L = FF.s_CH4_O3L;
%s_O3S = FF.s_O3S;

% Segment-wise forcing factors already computed above:
% s_AIC_seg, s_O3S_seg, s_CH4_seg, s_O3L_seg

refRatio_O3S = 1.01e-11;



%% RF CALCULATIONS

refRatio_H2O = 7.43e-15;
refRatio_SO4 = -1e-10;
refRatio_SOOT = 5e-10;

RF_i_H2O = refRatio_H2O * E_i_H2O;
RF_i_SO4 = refRatio_SO4 * E_i_SO4;
RF_i_SOOT = refRatio_SOOT * E_i_SOOT;

% =========================================================
% SEGMENT-WISE AIC APPLICATION
% =========================================================
% Exact per-segment application over flown distance
RF_i_AIC_seg = RF_ref_per_L .* s_AIC_seg .* seg.dR_nm;
RF_i_AIC = sum(RF_i_AIC_seg);

% =========================================================
% SEGMENT-WISE NOx APPLICATION
% =========================================================
% IMPORTANT:
% This is explicit per-segment forcing-factor application.
% Because your mission model does NOT expose segment fuel burn / segment NOx,
% total NOx is apportioned by flown segment distance as a non-invasive proxy.
% If you later expose segment fuel burn, replace E_i_NOx_seg with that.

E_i_NOx_seg = E_i_NOx .* seg.dR_m ./ sum(seg.dR_m);

RF_i_O3S_seg = refRatio_O3S .* s_O3S_seg .* E_i_NOx_seg;
RF_i_O3S = sum(RF_i_O3S_seg);

RF_i_CH4_weighted = sum(s_CH4_seg .* E_i_NOx_seg);
RF_i_O3L_weighted = sum(s_O3L_seg .* E_i_NOx_seg);

RF_i_CH4 = RF_i_CH4_weighted .* Gi_CH4;
RF_i_O3L = RF_i_O3L_weighted .* Gi_O3L;



%% EFFECTIVE RF*

Eff_H2O = 1.14;
Eff_SO4 = 0.9;
Eff_SOOT = 0.7;
Eff_AIC = 0.59;
Eff_O3 = 1.37;
Eff_CH4 = 1.18;

RF_2CO2 = 3.7;

RFstar_H2O = Eff_H2O * (RF_i_H2O / RF_2CO2);
RFstar_SO4 = Eff_SO4 * (RF_i_SO4 / RF_2CO2);
RFstar_SOOT = Eff_SOOT * (RF_i_SOOT / RF_2CO2);
RFstar_AIC = Eff_AIC * (RF_i_AIC / RF_2CO2);
RFstar_O3S = Eff_O3 * (RF_i_O3S / RF_2CO2);

RFstar_CH4_t = Eff_CH4 * (RF_i_CH4 / RF_2CO2);
RFstar_O3L_t = Eff_O3 * (RF_i_O3L / RF_2CO2);





%% SHORT-SPECIES TIME DISTRIBUTION (dt-INVARIANT)

% kernel = (t/tau_short) .* exp(-t/tau_short);
% kernel = kernel / sum(kernel*dt);
% 
% RFstar_H2O_t = RFstar_H2O * kernel;
% RFstar_SO4_t = RFstar_SO4 * kernel;
% RFstar_SOOT_t = RFstar_SOOT * kernel;
% RFstar_AIC_t = RFstar_AIC * kernel;
% RFstar_O3S_t = RFstar_O3S * kernel;

tau_H2O  = 0.5;   % ~1 week
tau_SO4  = 0.5;   % ~3–5 days
tau_SOOT = 0.5;   % ~10 days
tau_AIC  = 0.1;  % ~1 day
tau_O3S  = 0.1;    % ~2 months


RFstar_H2O_t  = RFstar_H2O  * (1/tau_H2O)  * exp(-t/tau_H2O);
RFstar_SO4_t  = RFstar_SO4  * (1/tau_SO4)  * exp(-t/tau_SO4);
RFstar_SOOT_t = RFstar_SOOT * (1/tau_SOOT) * exp(-t/tau_SOOT);
RFstar_AIC_t  = RFstar_AIC  * (1/tau_AIC)  * exp(-t/tau_AIC);
RFstar_O3S_t  = RFstar_O3S  * (1/tau_O3S)  * exp(-t/tau_O3S);


%% TEMPERATURE RESPONSE

DeltaT_H2O = conv_causal(RFstar_H2O_t); DeltaT_H2O = DeltaT_H2O(1:N);
DeltaT_SO4 = conv_causal(RFstar_SO4_t); DeltaT_SO4 = DeltaT_SO4(1:N);
DeltaT_SOOT = conv_causal(RFstar_SOOT_t); DeltaT_SOOT = DeltaT_SOOT(1:N);
DeltaT_AIC = conv_causal(RFstar_AIC_t); DeltaT_AIC = DeltaT_AIC(1:N);
DeltaT_O3S = conv_causal(RFstar_O3S_t); DeltaT_O3S = DeltaT_O3S(1:N);

DeltaT_CH4 = conv_causal(RFstar_CH4_t); DeltaT_CH4 = DeltaT_CH4(1:N);
DeltaT_O3L = conv_causal(RFstar_O3L_t); DeltaT_O3L = DeltaT_O3L(1:N);

DeltaT_CO2 = conv_causal(RFstar_CO2_t); DeltaT_CO2 = DeltaT_CO2(1:N);



%% TOTAL TEMPERATURE RESPONSE

DeltaT_all = DeltaT_CO2 + DeltaT_H2O + DeltaT_SO4 + DeltaT_SOOT + ...
             DeltaT_AIC + DeltaT_CH4 + DeltaT_O3L + DeltaT_O3S;



%% ATR

H = 100;

ATR_H2O = (1/H) * trapz(t, DeltaT_H2O);
ATR_SO4 = (1/H) * trapz(t, DeltaT_SO4);
ATR_SOOT = (1/H) * trapz(t, DeltaT_SOOT);
ATR_AIC = (1/H) * trapz(t, DeltaT_AIC);
ATR_O3S = (1/H) * trapz(t, DeltaT_O3S);
ATR_O3L = (1/H) * trapz(t, DeltaT_O3L);
ATR_CH4 = (1/H) * trapz(t, DeltaT_CH4);
ATR_CO2 = (1/H) * trapz(t, DeltaT_CO2);

ATR_t_all = zeros(size(t));
ATR_t_CO2 = zeros(size(t));

ATR_t_H2O = zeros(size(t));
ATR_t_SO4 = zeros(size(t));
ATR_t_SOOT = zeros(size(t));
ATR_t_AIC = zeros(size(t));
ATR_t_O3S = zeros(size(t));
ATR_t_O3L = zeros(size(t));
ATR_t_CH4 = zeros(size(t));

for i = 2:length(t)
    H_current = t(i);

    ATR_t_all(i) = (1/H_current) * trapz(t(1:i), DeltaT_all(1:i));
    ATR_t_CO2(i) = (1/H_current) * trapz(t(1:i), DeltaT_CO2(1:i));

    ATR_t_H2O(i) = (1/H_current) * trapz(t(1:i), DeltaT_H2O(1:i));
    ATR_t_SO4(i) = (1/H_current) * trapz(t(1:i), DeltaT_SO4(1:i));
    ATR_t_SOOT(i) = (1/H_current) * trapz(t(1:i), DeltaT_SOOT(1:i));
    ATR_t_AIC(i) = (1/H_current) * trapz(t(1:i), DeltaT_AIC(1:i));
    ATR_t_O3S(i) = (1/H_current) * trapz(t(1:i), DeltaT_O3S(1:i));
    ATR_t_O3L(i) = (1/H_current) * trapz(t(1:i), DeltaT_O3L(1:i));
    ATR_t_CH4(i) = (1/H_current) * trapz(t(1:i), DeltaT_CH4(1:i));
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
plot(t, ATR_t_all, 'k', 'LineWidth', 3);

legend('CO2','H2O','SO4','SOOT','AIC','CH4','O3L','O3S','TOTAL');

xlabel('Time horizon H (years)');
ylabel('ATR (K)');
title('ATR Trend by Species (1–100 years)');
grid on;
ytickformat('%.2e')



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






function FF = computeWeightedForcingFactors(h_profile_ft, R_profile_m)

    if numel(h_profile_ft) ~= numel(R_profile_m)
        error('h_profile_ft and R_profile_m must have the same length.')
    end

    if numel(h_profile_ft) < 2
        error('Need at least 2 profile points.')
    end

    dR_m = diff(R_profile_m);
    h_mid_ft = 0.5 * (h_profile_ft(1:end-1) + h_profile_ft(2:end));

    keep = isfinite(dR_m) & isfinite(h_mid_ft) & (dR_m > 0);
    dR_m = dR_m(keep);
    h_mid_ft = h_mid_ft(keep);

    if isempty(dR_m)
        error('No positive segment lengths found.')
    end

    s_AIC_profile = get_s_AIC(h_mid_ft);
    s_O3S_profile = get_s_O3S(h_mid_ft);
    s_CH4_profile = get_s_CH4_O3L(h_mid_ft);

    FF.h_mid_ft = h_mid_ft;
    FF.dR_m = dR_m;

    FF.s_AIC_profile = s_AIC_profile;
    FF.s_O3S_profile = s_O3S_profile;
    FF.s_CH4_profile = s_CH4_profile;

    FF.s_AIC = sum(s_AIC_profile .* dR_m) / sum(dR_m);
    FF.s_O3S = sum(s_O3S_profile .* dR_m) / sum(dR_m);
    FF.s_CH4_O3L = sum(s_CH4_profile .* dR_m) / sum(dR_m);
end


function plotForcingFactorGraph(h_profile_ft, R_profile_m)

    h_plot = linspace(16000, 42000, 300);

    s_AIC = get_s_AIC(h_plot);
    s_O3S = get_s_O3S(h_plot);
    s_CH4 = get_s_CH4_O3L(h_plot);

    figure;
    plot(s_O3S, h_plot, 'b', 'LineWidth', 2); hold on;
    plot(s_CH4, h_plot, 'g', 'LineWidth', 2);
    plot(s_AIC, h_plot, 'r', 'LineWidth', 2);

    if nargin == 2 && ~isempty(h_profile_ft) && ~isempty(R_profile_m)
        FF = computeWeightedForcingFactors(h_profile_ft, R_profile_m);

        xline(FF.s_O3S, '--b', 'LineWidth', 1.3);
        xline(FF.s_CH4_O3L, '--g', 'LineWidth', 1.3);
        xline(FF.s_AIC, '--r', 'LineWidth', 1.3);

        legend('O_{3S}', 'CH_4 & O_{3L}', 'AIC', ...
               'O_{3S} avg', 'CH_4 & O_{3L} avg', 'AIC avg', ...
               'Location', 'southeast');
    else
        legend('O_{3S}', 'CH_4 & O_{3L}', 'AIC', 'Location', 'southeast');
    end

    xlabel('Forcing factor s');
    ylabel('Altitude [ft]');
    title('Altitude-dependent forcing factors');
    grid on;
    xlim([0 2.5]);
    ylim([16000 42000]);
end


function s = get_s_AIC(h)

    h_data = [ ...
        17000 18000 20000 22000 24000 26000 28000 ...
        30000 32000 34000 36000 38000 40000 42000];

    s_data = [ ...
        0.02 0.05 0.05 0.10 0.25 0.50 0.80 ...
        1.10 1.60 2.10 1.80 1.50 1.20 0.90];

    s = interp1(h_data, s_data, h, 'pchip');
    s(h < min(h_data)) = s_data(1);
    s(h > max(h_data)) = s_data(end);
    s(s < 0) = 0;
end


function s = get_s_O3S(h)

    h_data = [ ...
        17000 19000 21000 23000 25000 27000 29000 ...
        31000 33000 35000 37000 39000 41000];

    s_data = [ ...
        0.45 0.50 0.55 0.60 0.70 0.80 0.95 ...
        1.10 1.25 1.40 1.60 1.80 2.00];

    s = interp1(h_data, s_data, h, 'pchip');
    s(h < min(h_data)) = s_data(1);
    s(h > max(h_data)) = s_data(end);
    s(s < 0) = 0;
end


function s = get_s_CH4_O3L(h)

    h_data = [ ...
        17000 19000 21000 23000 25000 27000 29000 ...
        31000 33000 35000 37000 39000 41000];

    s_data = [ ...
        0.90 0.90 0.90 0.92 0.95 0.98 1.00 ...
        1.05 1.10 1.15 1.20 1.20 1.20];

    s = interp1(h_data, s_data, h, 'pchip');
    s(h < min(h_data)) = s_data(1);
    s(h > max(h_data)) = s_data(end);
    s(s < 0) = 0;
end


function seg = buildClimateSegments(h_profile_ft, R_profile_m)

    if numel(h_profile_ft) ~= numel(R_profile_m)
        error('h_profile_ft and R_profile_m must have the same length.')
    end

    if numel(h_profile_ft) < 2
        error('Need at least 2 profile points.')
    end

    dR_m = diff(R_profile_m(:).');
    dR_nm = dR_m / 1852;

    h_mid_ft = 0.5 * (h_profile_ft(1:end-1) + h_profile_ft(2:end));
    dh_ft = diff(h_profile_ft(:).');

    keep = isfinite(dR_m) & isfinite(dR_nm) & isfinite(h_mid_ft) & isfinite(dh_ft) & (dR_m > 0);

    seg.dR_m = dR_m(keep);
    seg.dR_nm = dR_nm(keep);
    seg.h_mid_ft = h_mid_ft(keep);
    seg.dh_ft = dh_ft(keep);

    % phase_id: 1 = climb, 2 = cruise, 3 = descent
    seg.phase_id = 2 * ones(size(seg.h_mid_ft));
    seg.phase_id(seg.dh_ft > 1e-6) = 1;
    seg.phase_id(seg.dh_ft < -1e-6) = 3;
end

function [h_profile_ft, R_profile_m, meta] = buildClimateProfileFromMissionCall(ADP, tripRange, M_TO)
%BUILDCLIMATEPROFILEFROMMISSIONCALL
% Calls your existing mission analysis unchanged, then reconstructs an
% airborne altitude-vs-range profile for climate forcing-factor weighting.
%
% This is a climate wrapper, not an exact extraction of internal mission
% arrays. It follows the same broad segment structure:
%   climb: 0-1500, 1500-10000, 10000-20000, 20000-cruise
%   cruise: constant cruise altitude
%   descent: cruise-20000, 20000-10000, 10000-0
%
% Inputs:
%   ADP       aircraft/design struct
%   tripRange total mission range [m]
%   M_TO      takeoff mass [kg]
%
% Outputs:
%   h_profile_ft  airborne altitude profile [ft]
%   R_profile_m   cumulative airborne range profile [m]
%   meta          diagnostics struct

    arguments
        ADP
        tripRange (1,1) double
        M_TO (1,1) double = ADP.MTOM
    end

    ft2m      = 0.3048;
    knots2m_s = 0.5144;

    % -------------------------------------------------
    % 1. Call your existing mission analysis unchanged
    % -------------------------------------------------
    [~,~,~,~,~,cruise_FL] = Unconventional.MissionAnalysis_PhysicsFinal(ADP, tripRange, M_TO);

    h_cruise_ft = 100 * cruise_FL;
    h_cruise_m  = h_cruise_ft * ft2m;

    % -------------------------------------------------
    % 2. Rebuild a climate-grade profile from same logic
    % -------------------------------------------------

    % Same broad assumptions as your mission code
    dh = 500 * ft2m;

    % climb rates copied from your structure
    dh_total_1500_20000 = (20000 - 1500) * ft2m;
    t_target = 30 * 60;
    ROC_base = dh_total_1500_20000 / t_target;
    ROC_used_12_23 = ROC_base * 3.2;
    ROC_used_34    = ROC_base * 1.0;

    % climb segment 01
    t01 = 50;
    V01 = 0.9 * 250 * knots2m_s;

    % climb segment 12
    V12 = 0.95 * 250 * knots2m_s;

    % segment 23 Mach
    M23 = ADP.TLAR.M_c * 0.8;

    % segment 34 Mach
    M34 = ADP.TLAR.M_c * 0.95;

    % descent settings from your code
    dt_des = 20;
    Mdes_hi = ADP.TLAR.M_c * 0.9;
    Vdes_lo = 0.9 * 250 * knots2m_s;

    h_profile_m = [];
    R_profile_m = [];

    % helper to append
    appendPoint(0, 0);

    % -----------------------------
    % CLIMB 0 -> 1500 ft
    % -----------------------------
    [Rseg, hseg] = buildSegmentByTime( ...
        0, 1500*ft2m, dh, t01, V01);
    appendSegment(Rseg, hseg);

    % -----------------------------
    % CLIMB 1500 -> 10000 ft
    % -----------------------------
    [Rseg, hseg] = buildSegmentByROC( ...
        1500*ft2m, 10000*ft2m, dh, ROC_used_12_23, V12);
    appendSegment(Rseg, hseg);

    % -----------------------------
    % CLIMB 10000 -> 20000 ft
    % -----------------------------
    [~, a23, ~, ~] = cast.atmos(0.5*(10000+20000)*ft2m);
    V23 = M23 * a23;
    [Rseg, hseg] = buildSegmentByROC( ...
        10000*ft2m, 20000*ft2m, dh, ROC_used_12_23, V23);
    appendSegment(Rseg, hseg);

    % -----------------------------
    % CLIMB 20000 ft -> cruise
    % -----------------------------
    if h_cruise_m > 20000*ft2m
        [~, a34, ~, ~] = cast.atmos(0.5*(20000 + h_cruise_ft)*ft2m);
        V34 = M34 * a34;
        [Rseg, hseg] = buildSegmentByROC( ...
            20000*ft2m, h_cruise_m, dh, ROC_used_34, V34);
        appendSegment(Rseg, hseg);
    end

    climb_range = R_profile_m(end);

    % -----------------------------
    % DESCENT cruise -> 0
    % -----------------------------
    [Rdes, hdes] = buildDescentProfile(h_cruise_m, dh, dt_des, Mdes_hi, Vdes_lo, ADP);
    descent_range = Rdes(end);

    % -----------------------------
    % CRUISE range = total - climb - descent
    % -----------------------------
    cruise_range = max(tripRange - climb_range - descent_range, 0);

    if cruise_range > 0
        appendPoint(R_profile_m(end) + cruise_range, h_cruise_m);
    end

    % append descent shifted
    Rdes_shifted = R_profile_m(end) + Rdes;
    appendSegment(Rdes_shifted, hdes);

    % final outputs
    h_profile_ft = h_profile_m / ft2m;

    meta.cruise_FL        = cruise_FL;
    meta.h_cruise_ft      = h_cruise_ft;
    meta.climb_range_m    = climb_range;
    meta.cruise_range_m   = cruise_range;
    meta.descent_range_m  = descent_range;
    meta.total_airborne_m = R_profile_m(end);

    % =============================
    % nested helpers
    % =============================
    function appendPoint(Rnew, hnew)
        R_profile_m(end+1) = Rnew;
        h_profile_m(end+1) = hnew;
    end

    function appendSegment(Rseg, hseg)
        if isempty(Rseg)
            return
        end
        % avoid duplicate first point
        if ~isempty(R_profile_m)
            Rseg = Rseg(:).';
            hseg = hseg(:).';
            if abs(Rseg(1) - R_profile_m(end)) < 1e-9 && abs(hseg(1) - h_profile_m(end)) < 1e-9
                Rseg(1) = [];
                hseg(1) = [];
            end
        end
        R_profile_m = [R_profile_m, Rseg];
        h_profile_m = [h_profile_m, hseg];
    end

    function [Rseg, hseg] = buildSegmentByTime(h0, h1, dh_local, t_total, V)
        if h1 <= h0
            Rseg = [];
            hseg = [];
            return
        end

        h_nodes = h0:dh_local:h1;
        if h_nodes(end) ~= h1
            h_nodes = [h_nodes h1];
        end

        Rseg = 0;
        hseg = h0;
        Rcum = 0;

        for k = 1:(length(h_nodes)-1)
            h_low  = h_nodes(k);
            h_high = h_nodes(k+1);
            h_mid  = 0.5*(h_low + h_high);

            dh_step = h_high - h_low;
            dt_step = t_total * dh_step / (h1 - h0);
            vy      = dh_step / dt_step;
            vx      = sqrt(max(V^2 - vy^2, 0));

            Rcum = Rcum + vx * dt_step;

            Rseg(end+1) = Rcum; %#ok<AGROW>
            hseg(end+1) = h_mid; %#ok<AGROW>
        end

        Rseg(end+1) = Rcum;
        hseg(end+1) = h1;
    end

    function [Rseg, hseg] = buildSegmentByROC(h0, h1, dh_local, ROC, V)
        if h1 <= h0
            Rseg = [];
            hseg = [];
            return
        end

        h_nodes = h0:dh_local:h1;
        if h_nodes(end) ~= h1
            h_nodes = [h_nodes h1];
        end

        Rseg = 0;
        hseg = h0;
        Rcum = 0;

        for k = 1:(length(h_nodes)-1)
            h_low  = h_nodes(k);
            h_high = h_nodes(k+1);
            h_mid  = 0.5*(h_low + h_high);

            dh_step = h_high - h_low;
            dt_step = dh_step / ROC;
            vy      = ROC;
            vx      = sqrt(max(V^2 - vy^2, 0));

            Rcum = Rcum + vx * dt_step;

            Rseg(end+1) = Rcum; %#ok<AGROW>
            hseg(end+1) = h_mid; %#ok<AGROW>
        end

        Rseg(end+1) = Rcum;
        hseg(end+1) = h1;
    end

    function [Rseg, hseg] = buildDescentProfile(h_start, dh_local, dt_local, M_high, V_low, ADP_local)
        if h_start <= 0
            Rseg = 0;
            hseg = 0;
            return
        end

        h_nodes = h_start:-dh_local:0;
        if h_nodes(end) ~= 0
            h_nodes = [h_nodes 0];
        end

        Rseg = 0;
        hseg = h_start;
        Rcum = 0;

        for k = 1:(length(h_nodes)-1)
            h_high = h_nodes(k);
            h_low  = h_nodes(k+1);
            h_mid  = 0.5*(h_high + h_low);

            [~, a_mid, ~, ~] = cast.atmos(h_mid);

            if h_mid > 20000*ft2m
                V = M_high * a_mid;
            else
                V = V_low;
            end

            vy = abs(h_low - h_high) / dt_local;
            vx = sqrt(max(V^2 - vy^2, 0));

            Rcum = Rcum + vx * dt_local;

            Rseg(end+1) = Rcum; %#ok<AGROW>
            hseg(end+1) = h_mid; %#ok<AGROW>
        end

        Rseg(end+1) = Rcum;
        hseg(end+1) = 0;
    end
end
