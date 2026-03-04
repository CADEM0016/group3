function loads = WingLoads(ac)
%WINGLOADS Computes wing shear and bending moment distribution
%
%   loads = Structures.WingLoads(ac)

g = 9.81;

% Use current MTOM from sizing loop
W = ac.MTOM * g;

% Design load factor (CS-25 maneuver)
n = 2.5;

% Wing geometry
b = ac.geom.b;        % span
S = ac.geom.S;        % wing area

% Elliptical lift distribution
y = linspace(0,b/2,200);
L0 = (2*n*W)/(pi*b/2);

L = L0 * sqrt(1 - (2*y/b).^2);

% Shear force
V = flip(cumtrapz(flip(y), flip(L)));

% Bending moment
M = flip(cumtrapz(flip(y), flip(V)));

loads.y = y;
loads.L = L;
loads.V = V;
loads.M = M;
loads.M_root = M(1);
loads.V_root = V(1);

end