%% Emissions model

% This is currenlty not connected toany other files yet and needs to be
% updated !!!!


%-------------------------------------------------------------------------
%ATR Setup 
%-------------------------------------------------------------------------


clear all

FuelBurn = 301217.6137;

%split per kg

% ---- check change with SAF
%AF is aviation fuel
AF_splt_CO2 = 3.16;
AF_splt_H2O = 1.26;
AF_splt_SO4 = 2*10^-4;
AF_splt_Soot = 4*10^-5;
%AF_splt_NOX
%AF_splt_AIC
%AF_splt_CH4

%mass of emissions

M_CO2 = FuelBurn*AF_splt_CO2;
M_H2O = FuelBurn*AF_splt_H2O;
M_SO4 = FuelBurn*AF_splt_SO4;
M_Soot = FuelBurn*AF_splt_Soot;

ppmv_CO2 = M_CO2/(7.8*10^12);

Tot_Range_nm = 5879+3252.7; %Total distance travelled from london to melbourne with full path
% https://www.airmilescalculator.com/distance/lhr-to-sin/

FF = 1.75; %forcing factor at current altitude change this later

altitude_ft = 36000; %altitude in feet for current iteration
Mach_c = 0.85; % cruise mach - change with connection

OPR = 38; %Pressure ratio ---- 977 model
% https://www.aircraft-commerce.com/wp-content/uploads/aircraft-commerce-docs1/Aircraft%20guides/RR%20TRENT/ISSUE83_TRENT_SPECS.pdf

T_0 = 216; %stagnation temperature at 36000 ft from standar atmospheric models
%ensure that they connect with atmosphere models in codes

gamma = 1.4;

T02 = T_0 * (1 + ((gamma-1)/2)*(Mach_c^2)); % Calculate the total temperature at the outlet

eta_compr = 0.9; %compressive efficieny - assumed get proper justification

T3 = T02*(1+(((OPR^((gamma-1)/gamma))-1)/eta_compr)); %Temperature at another stage recomment it

LHV_AF = 43*10^6; %LVH = Lower heating value & net calorific value
%https://www.engineeringtoolbox.com/fuels-higher-calorific-values-d_169.html

Fuel2Air = (1/60)*1.5; % fuel to air ratio from raymer multiplied by 1.5 for some reason!!!!

eta_combu = 0.9; %combustion efficiency - assuemd get proper justifications

C_p = 1004; %classic thermal value recomment it later

T4 = T3+((Fuel2Air*eta_combu*LHV_AF)/C_p); %final necessary temperature value

Pinf = 22600; %isa table check again

P02 = Pinf*(1+(((gamma-1)/2)*(Mach_c^2)))^(gamma/(gamma-1));

P03 = P02*OPR;

RH = 50; %taken as 50 for now but kinda arbitrary so double check!!!!



EINOX = (0.0986*((P03/101325)^0.4)*(exp((T3/194)+(RH/53.2)))); %double check what this is again
SNOX = 1.45; %double check what this is again


%-------------------------------------------------------------------------
%ATR Calcs 
%-------------------------------------------------------------------------

t = 0:1:100; % years

%CO2 Calcs

GXCO2 = 0.067 ...
    + 0.1135*exp(-t/313.8) ...
    + 0.152*exp(-t/79.8) ...
    + 0.097*exp(-t/18.8) ...
    + 0.041*exp(-t/1.7);
delta_XCO2 = GXCO2*ppmv_CO2*10^-6;
XCO2_0 = 380*10^-6;
RF_CO2 = (1/log(2))*log((delta_XCO2+XCO2_0)/XCO2_0);
Eff_RF_CO2 = 1*RF_CO2;
GT = (2.245/36.8)*exp(-t/36.8);
deltaT_CO2 = GT.*Eff_RF_CO2;

%NOX Calcs

GCH4 = (-5.16*10^-13)*exp(-t/12);
RF_CH4 = GCH4*EINOX;
Eff_RF_CH4 = RF_CH4*1.18;
deltaT_CH4 = GT.*Eff_RF_CH4;


GO3L = (-5.16*10^-13)*exp(-t/12);
RF_O3L = GO3L*EINOX;
Eff_RF_O3L = RF_O3L*1.37;
deltaT_O3L = GT.*Eff_RF_O3L;

RF_NOXO3S = SNOX*(1.01*10^-11)*EINOX;
Eff_RF_NOXO3S = 1.37*RF_NOXO3S;
deltaT_NOXO3S = GT(1).*Eff_RF_NOXO3S;

%H2O Calcs

RF_H2O = M_H2O*7.43*10^-15;
Eff_RF_H2O = 1.14*RF_H2O;
deltaT_H2O = GT(1).*Eff_RF_H2O;

%SO4 Calcs

RF_SO4 = M_SO4*-1*10^-10;
Eff_RF_SO4 = RF_SO4*0.9;
deltaT_SO4 = GT(1).*Eff_RF_SO4;

%Soot Calcs

RF_Soot = M_Soot*5*10^-10;
Eff_RF_Soot = RF_Soot*0.7;
deltaT_Soot = GT(1).*Eff_RF_Soot;

%AIC Calcs

RF_AIC = FF*(2.21*10^-12)*Tot_Range_nm;
Eff_RF_AIC = RF_AIC*0.59;
deltaT_AIC = GT(1).*Eff_RF_AIC;


Total_deltaT = deltaT_AIC + deltaT_Soot + deltaT_SO4 +deltaT_H2O + deltaT_NOXO3S + deltaT_O3L + deltaT_CH4 + deltaT_CO2;



%plots ---- combine with temporaray o3

%figure
%plot(t,Total_deltaT)
%hold on
%plot(t,deltaT_CH4)
%plot(t,deltaT_O3L)

figure
plot(t,deltaT_CO2,'LineWidth',2)
hold on
plot(t,deltaT_CH4,'LineWidth',2)
plot(t,deltaT_O3L,'LineWidth',2)
plot(t,deltaT_NOXO3S,'LineWidth',2)
plot(t,deltaT_H2O,'LineWidth',2)
plot(t,deltaT_SO4,'LineWidth',2)
plot(t,deltaT_Soot,'LineWidth',2)
plot(t,deltaT_AIC,'LineWidth',2)
plot(t,Total_deltaT,'k','LineWidth',3)

grid on
xlabel('Time (years)')
ylabel('\DeltaT (K)')
title('Temperature Response by Emission Species')

legend( ...
'CO2', ...
'CH4 (NOx)', ...
'O3 Long', ...
'O3 Short', ...
'H2O', ...
'Sulfates', ...
'Soot', ...
'Contrails', ...
'Total','Location','best')

% notes for other subjects obj.RangeA = 10888000;%/SI.Nmile;% m (from nautical miles) - london singapore ------- find study of things
%5deg change with landing gear

%change the tsfc file