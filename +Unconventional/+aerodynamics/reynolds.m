function [reynolds] = reynolds(v,refL,alt)
% Finds the reynolds number for a given 
% rho - Density (m^3/kg)
% v - velocity (m/s^2)
% refL - Reference length (m)
% alt - Altitude (m)
% NOTE: Standard ISA 15+ used

temp_off = 15; % Temp offset (°C)

[rho,a,T,P,nu,z,sigma] = cast.atmos(alt,temp_off);

reynolds = (v*refL)/nu;
end