function sizing = WingSizing(ac, loads)
%WINGSIZING Sizes wing box for bending, buckling, hinge and deflection
%   sizing = Structures.WingSizing(ac, loads)
%
%   Inputs:
%       ac    - aircraft struct (from sizing loop)
%       loads - output from Structures.WingLoads(ac)
%
%   Outputs:
%       sizing.mass        - total wing structural mass (kg)
%       sizing.t_skin      - required skin thickness (m)
%       sizing.delta_tip   - estimated tip deflection (m)
%       sizing.constraints - MDO constraint vector [g1 g2 g3 g4]
%                            all must be <= 0 for feasible design

if nargin < 2
    error('Structures:WingSizing:noInput', ...
        'WingSizing requires both "ac" and "loads" as inputs.');
end

%% --- SAFE MATERIAL INPUTS ---
rho_mat     = getfield_safe(ac, 'struct', 'material', 'rho',         2800);   % kg/m^3
sigma_allow = getfield_safe(ac, 'struct', 'material', 'sigma_allow', 300e6);  % Pa
E           = getfield_safe(ac, 'struct', 'material', 'E',           70e9);   % Pa

%% --- SAFE STRUCTURAL PARAMETERS ---
SF          = getfield_safe(ac, 'struct', [],        'SF',           1.5);
rib_spacing = getfield_safe(ac, 'struct', [],        'rib_spacing',  0.6);    % m

%% --- SAFE GEOMETRY ---
b      = getfield_safe(ac, 'geom', [], 'b',      60);    % span (m)
c_root = getfield_safe(ac, 'geom', [], 'c_root',  8);    % root chord (m)
tc     = getfield_safe(ac, 'geom', [], 'tc',      0.12); % thickness-to-chord

%% --- WING BOX GEOMETRY ---
h         = tc * c_root;        % wing box height (m)
box_width = 0.4 * c_root;      % 40% chord box width assumption

%% --- ROOT BENDING SIZING ---
M_root = loads.M_root;

% Required second moment of area from bending stress constraint
I_req  = (M_root * (h/2)) / (sigma_allow / SF);

% Required skin thickness from simplified box beam
t_skin = I_req / (box_width * (h/2)^2);

%% --- BUCKLING CHECK (upper compression skin) ---
k  = 4;
nu = 0.33;

sigma_cr = (k * pi^2 * E) / (12 * (1 - nu^2)) * (t_skin / rib_spacing)^2;

if sigma_cr < (sigma_allow / SF)
    t_skin = t_skin * 1.2;   % increase thickness if buckling governs
    % Recompute I with updated skin thickness
    I_req  = box_width * (h/2)^2 * t_skin;
end

%% --- HINGE SIZING ---
M_fold = loads.M_fold;

% Hinge reinforcement mass (engineering factor on hinge bending demand)
k_reinf = getfield_safe(ac, 'struct', 'fold', 'k_reinf', 1.3);

if M_fold > 0
    I_hinge_req = (M_fold * (h/2)) / (sigma_allow / SF);
    hinge_mass  = k_reinf * rho_mat * I_hinge_req / h;
else
    hinge_mass = 0;
end

%% --- TIP DEFLECTION ESTIMATE (cantilever beam approximation) ---
delta_tip = (M_root * (b/2)^2) / (2 * E * I_req);
delta_max = 0.1 * b;   % 10% span limit

%% --- STRUCTURAL MASS ESTIMATION ---
A_struct   = 2 * box_width * t_skin;   % simplified cross-section area
V_half     = A_struct * (b/2);
mass_wing  = 2 * rho_mat * V_half + hinge_mass;   % full wing + hinge

%% --- MDO CONSTRAINT VECTOR (all <= 0 means feasible) ---
sigma_actual = M_root * (h/2) / I_req;

g1 = sigma_actual / (sigma_allow / SF) - 1;    % bending stress
g2 = (sigma_allow / SF) / sigma_cr - 1;        % buckling
g3 = delta_tip / delta_max - 1;                % tip deflection
g4 = M_fold / max(sigma_allow * I_req / (h/2), 1e-6) - 1;  % hinge bending

%% --- OUTPUT STRUCT ---
sizing.mass        = mass_wing;
sizing.t_skin      = t_skin;
sizing.I_req       = I_req;
sizing.delta_tip   = delta_tip;
sizing.hinge_mass  = hinge_mass;
sizing.constraints = [g1, g2, g3, g4];

end


%% ===== HELPER: SAFE FIELD ACCESS (supports up to 3 levels) =====
function val = getfield_safe(s, sub1, sub2, field, default)
% getfield_safe  Safely extract nested struct field with fallback default
%   Usage:
%       getfield_safe(s, 'sub1', 'sub2', 'field', default)  -> s.sub1.sub2.field
%       getfield_safe(s, 'sub1', [],     'field', default)  -> s.sub1.field
%       getfield_safe(s, [],     [],     'field', default)  -> s.field
    try
        if isempty(sub1)
            val = s.(field);
        elseif isempty(sub2)
            val = s.(sub1).(field);
        else
            val = s.(sub1).(sub2).(field);
        end
    catch
        val = default;
    end
end