function [massObj,GeomObj] = auxPowerUnit(obj)
%AUXPOWERUNIT Summary of this function goes here
%   Detailed explanation goes here


% rectangle
Xs = [-0.5,0.5;0.5,0.5;0.5,-0.5;-0.5,-0.5];
% https://aerospace.honeywell.com/us/en/products-and-services/products/power-and-propulsion/auxiliary-power-units/331-series-auxiliary-power-units
% Rough APU dimensions 
apuLength = 1.8; % (m)
apuWidth  = 1.1; % (m)

% Scale rectangle
Xs = Xs .* [apuLength apuWidth];

% Offset to cargo centroid
offsetAPU = [(obj.FuselageLength - apuLength) 0];
GeomObj = cast.GeomObj(Name="APU",Xs=offsetAPU+Xs);

% ------------------------- Create Mass Objects --------------------------
mass_APU_lbs = 2.2*c.APU_uninst;
mass_APU_kg = mass_APU_lbs * 1/SI.lbs;

massObj(end+1) = cast.MassObj(Name="APU", m=mass_APU_kg, X=obj.FuselageLength); % Assumed at the end of the tail
end