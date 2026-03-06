function [rho, a, T, P] = Atmosphere(alt_m)
% =========================================================================
% Atmosphere.m  —  +Structures package
% ISA Standard Atmosphere model.  PRIVATE COPY — no cast.atmos dependency.
%
% INPUT:
%   alt_m   — geometric altitude [m], scalar or array
%
% OUTPUTS:
%   rho     — air density [kg/m^3]
%   a       — speed of sound [m/s]
%   T       — temperature [K]
%   P       — pressure [Pa]
%
% Valid range: 0 – 20,000 m (covers all relevant flight phases)
% =========================================================================

% ISA constants
T0     = 288.15;    % Sea-level temperature [K]
P0     = 101325;    % Sea-level pressure [Pa]
L      = 0.0065;    % Lapse rate [K/m]
H_trop = 11000;     % Tropopause altitude [m]
T_trop = 216.65;    % Tropopause temperature [K]
P_trop = 22632.1;   % Tropopause pressure [Pa]
R      = 287.058;   % Gas constant [J/kg/K]
gamma  = 1.4;
g      = 9.80665;

% Pre-allocate
T   = zeros(size(alt_m));
P   = zeros(size(alt_m));
rho = zeros(size(alt_m));
a   = zeros(size(alt_m));

for i = 1:numel(alt_m)
    h = alt_m(i);
    if h <= H_trop
        % Troposphere: temperature decreases linearly
        T(i) = T0 - L * h;
        P(i) = P0 * (T(i)/T0)^(g/(R*L));
    else
        % Lower stratosphere: isothermal
        T(i) = T_trop;
        P(i) = P_trop * exp(-g*(h - H_trop)/(R*T_trop));
    end
    rho(i) = P(i) / (R * T(i));
    a(i)   = sqrt(gamma * R * T(i));
end

end