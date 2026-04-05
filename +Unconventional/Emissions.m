%% EMISSIONS MODEL

%M_fuel = 15*10^3;
M_fuel = TotalBurn;
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

L = TotalRange; % mission distance [m]

RF_ref_per_L = 2.21e-12;
RF_ref_per_L_km = RF_ref_per_L/(1.852e3);

s_AIC = 1;



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



%% NOx EFFECTS

Ai_CH4 = -5.16e-13;
Ai_O3L = -1.21e-13;

tau_n = 12;
Gi_CH4 = Ai_CH4 * exp(-t/tau_n);
Gi_O3L = Ai_O3L * exp(-t/tau_n);

s_CH4 = 1;
s_O3L = 1;
s_O3S = 1;

refRatio_O3S = 1.01e-11;



%% RF CALCULATIONS

refRatio_H2O = 7.43e-15;
refRatio_SO4 = -1e-10;
refRatio_SOOT = 5e-10;

RF_i_H2O = refRatio_H2O * E_i_H2O;
RF_i_SO4 = refRatio_SO4 * E_i_SO4;
RF_i_SOOT = refRatio_SOOT * E_i_SOOT;
RF_i_AIC = s_AIC * RF_ref_per_L_km * L;

RF_i_CH4 = s_CH4 * Gi_CH4 * E_i_NOx;
RF_i_O3L = s_O3L * Gi_O3L * E_i_NOx;
RF_i_O3S = s_O3S * refRatio_O3S * E_i_NOx;



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

c_CO2  = [1 1 1];
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

