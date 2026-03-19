function [x_dist,z_dist] = leading_offset(sweep_angle,dihedral_angle,wing_location)
%LEADING_OFFSET 
% For finding the offset of a given section of the wing
% sweep_angle: Sweep of the leading edge in degrees
% dihedral_angle: "Vertical" offset of the wing in degrees
% wing_location: location along the wing that we want to know the x,z
% coordinates

% X Location (m)
x_dist = wing_location * tand(sweep_angle);
% Z Location (m)
z_dist = wing_location * tand(dihedral_angle);

end