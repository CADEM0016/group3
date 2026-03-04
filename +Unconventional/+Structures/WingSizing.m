function sizing = WingSizing(ac, loads)
%WINGSIZING Sizes spar caps and skin using bending stress

% Material properties (Aluminium baseline)
rho = 2800;                % kg/m^3
sigma_allow = 300e6;       % Pa
E = 70e9;                  % Pa

b = ac.geom.b;
c_root = ac.geom.c_root;
t_c = ac.geom.tc;

h = t_c * c_root;          % wing box height

M_root = loads.M_root;

% Required second moment of area
I_req = M_root * (h/2) / sigma_allow;

% Assume box width 40% chord
box_width = 0.4 * c_root;

% Solve for skin thickness approximation
t_skin = I_req / (box_width * (h/2)^2);

% Buckling check (upper skin)
k = 4;
b_panel = 0.6;  % assumed rib spacing (m)

sigma_cr = (k*pi^2*E)/(12*(1-0.33^2)) * (t_skin/b_panel)^2;

if sigma_cr < sigma_allow
    t_skin = t_skin * 1.2;  % increase thickness if buckling critical
end

% Approximate structural area
A_struct = 2*box_width*t_skin;

% Structural volume
V_struct = A_struct * (b/2);

mass_half = rho * V_struct;

sizing.mass = 2 * mass_half;   % full wing
sizing.t_skin = t_skin;

end