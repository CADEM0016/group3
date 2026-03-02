function [massObj,GeomObj] = cargoHandling(obj)
%CARGOHANDLING Summary of this function goes here
%   Detailed explanation goes here

% ---------------- CARGO HANDLING GEOMETRY ----------------

% rectangle
Xs = [-0.5,0.5;0.5,0.5;0.5,-0.5;-0.5,-0.5];
% Cargo dimensions
cargoLength = obj.x_cargo_end - obj.x_cargo_start;
cargoWidth  = 2*(obj.CabinRadius - obj.SidewallThickness);

% Scale rectangle
Xs = Xs .* [cargoLength cargoWidth];

% Offset to cargo centroid
offsetCargo = [obj.x_cargo_cg 0];

% Create geometry
GeomObj = cast.GeomObj(Name="CargoHandlingSystem",Xs=Xs + offsetCargo);


% --------------------------- Create Mass Objects----------------------------
mass_handling_lbs = 2.4*obj.CabinLength*obj.CabinRadius*obj.Decks; % Can be improved rough guess
mass_handling_kg = mass_handling_lbs * 1/SI.lbs;
massObj(end) = cast.MassObj(Name="Cargo handling",m=mass_handling_kg,X=Xs);
end