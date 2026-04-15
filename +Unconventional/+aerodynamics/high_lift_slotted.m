function [CLmax] = high_lift_slotted(obj,re_c,aerofoil,flap_deflection,M)
%HIGH_LIFT Summary of this function goes here
% ESDU 91014 - Multi-Slotted Flap Analysis

% File imports
% - Provides imported data for ESDU graphs
cwd = fileparts(mfilename('fullpath'));
figFolder = fullfile(cwd,'+datafigs');

fig4_83040_path = fullfile(figFolder,'83040_figure4.csv');
fig4_83040 = Unconventional.aerodynamics.datafigs.loadcsvmatrix(fig4_83040_path);
fig4_83040 = fig4_83040(~any(isnan(fig4_83040),2), :);
fig4_83040 = sortrows(fig4_83040, 1);
[x4, ~, ic4] = unique(fig4_83040(:,1));
y4 = accumarray(ic4, fig4_83040(:,2), [], @mean);
fig4_83040 = [x4, y4];

fig5_83040_path = fullfile(figFolder,'83040_figure5.csv');
fig5_83040 = Unconventional.aerodynamics.datafigs.loadcsvmatrix(fig5_83040_path);
fig5_83040 = fig5_83040(~any(isnan(fig5_83040),2), :);
fig5_83040 = sortrows(fig5_83040, 1);
[x5, ~, ic5] = unique(fig5_83040(:,1));
y5 = accumarray(ic5, fig5_83040(:,2), [], @mean);
fig5_83040 = [x5, y5];

fig1_91028_path = fullfile(figFolder,'91028_figure1.csv');
fig1_91028 = Unconventional.aerodynamics.datafigs.loadcsvmatrix(fig1_91028_path);
fig1_91028 = fig1_91028(~any(isnan(fig1_91028),2), :);
fig1_91028 = sortrows(fig1_91028, 1);
[x91028, ~, ic91028] = unique(fig1_91028(:,1));
y91028 = accumarray(ic91028, fig1_91028(:,2), [], @mean);
fig1_91028 = [x91028, y91028];

fig1_91014 = fullfile(figFolder,'91014_figure1.csv');
fig1_91014 = Unconventional.aerodynamics.datafigs.loadcsvmatrix(fig1_91014);

fig2_91014 = fullfile(figFolder,'91014_figure3.csv');
fig2_91014 = Unconventional.aerodynamics.datafigs.loadcsvmatrix(fig2_91014);

fig3_91014 = fullfile(figFolder,'91014_figure3.csv');
fig3_91014 = Unconventional.aerodynamics.datafigs.loadcsvmatrix(fig3_91014);

% Aerofoil parameters
t_c = aerofoil.t_c% thickness to chord
rho1_c = aerofoil.rho1_c; % in example 0.01087  Leading edge radius to chord ratio
rho1_t = rho1_c / t_c; % leading edge to thickness ratio

phi_t = aerofoil.phi_t; % Trailing edge flap effectiveness

% Wing parameter
lambda = obj.TaperRatio; % Wing taper Ratio
AspectRatio = obj.AspectRatio; % Wing Aspect Ratio
delta_quarter = obj.WingQuaterChord;
c_t_c_prime = obj.FlapChordRatio; % flap chord ratio

% Section 1 
delta_0 = atan( tan(obj.WingQuaterChord) + (1/AspectRatio)*((1 - lambda)/(1 + lambda)) );
delta_1 = atan( tan(delta_quarter) - (3/AspectRatio)*((1 - lambda)/(1 + lambda)) );
delta_h = atan( tan(delta_quarter) + (4/AspectRatio)*( (1/4 - 0.70) * ((1 - lambda)/(1 + lambda)) ) );

% Section 2
A_tan_half = AspectRatio * tan(delta_quarter) * ((1 - lambda) / (1 + lambda));
kappa = (1 + 2*lambda) / (3*(1 + lambda));

% Section 3
 % FROM GRAPH 83040 fig 4 or fig 5
betaA = sqrt(1 - M^2) * AspectRatio;

if abs(betaA - 8) < abs(betaA - 12)
    % betaA is closer to 8
    eta_bar = interp1(fig4_83040(:,1), fig4_83040(:,2), A_tan_half, 'linear', 'extrap');
else
    % betaA is closer to 12
    eta_bar = interp1(fig5_83040(:,1), fig5_83040(:,2), A_tan_half, 'linear', 'extrap');
end


% Section 4
eta_p = Unconventional.aerodynamics.datafigs.getCurve(fig1_91014, eta_bar, lambda); % FROM GRAPH 91014 fig 1

% Section 5
mu_p = Unconventional.aerodynamics.datafigs.getCurve(fig2_91014, eta_bar, lambda); % FROM GRAPH 91014 fig 2

% Section 6
cp_cbar = (3/2) * ((1 + lambda) / (1 + lambda + lambda^2)) * (1 - eta_p + lambda*eta_p);
R_cp = re_c * cp_cbar
R_cp_cos2_Lambda0 = R_cp * cosd(delta_0)^2


% Section 7
b  = (flap_deflection + phi_t) * sec(delta_h) % Done to account for sweep

delta_t_sec_delta_h = deg2rad(flap_deflection) * sec(delta_h)

J_p = interp1(fig1_91028(:,1), fig1_91028(:,2), delta_t_sec_delta_h, 'linear', 'extrap')


Delta_CL0t = 2 * J_p * delta_t_sec_delta_h * ( pi - acos(2 *c_t_c_prime- 1) + sqrt(1 - (2 *c_t_c_prime- 1)^2) )


K_G = 1.225 + 4.525 * rho1_t;
K_t = 1.0; % Note this would be 0.8 for plain flaps -- sort of a guess ?!
T = 0.442;
Delta_CL0t;
Delta_CLmt = K_G * K_t * T * Delta_CL0t
Delta_CLmt_final = (1/c_t_c_prime) * Delta_CLmt;

% Section 8 - FROM GRAPH 91014 fig 3
phi_0 = Unconventional.aerodynamics.datafigs.getCurve(fig3_91014, (A_tan_half - 8*lambda), eta_p); % Not 100% sure if eta_p is correct here !!
Phi_i = 0;
F_R = 0.153 * log10(R_cp_cos2_Lambda0);
K_Lt = cosd(delta_quarter)^2.5;

K_f = 1.1 % for slotted
Delta_CLmaxt = K_f * K_Lt * cosd(delta_h) * F_R * (Delta_CLmt_final / mu_p) * (phi_0 - Phi_i);


CLmax = Delta_CLmaxt + (aerofoil.clmax*obj.oswald*cosd(obj.WingSweep)); % Rough approximation for no flap CLmax

end