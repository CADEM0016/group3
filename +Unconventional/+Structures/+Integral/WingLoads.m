function loads = WingLoads(ac)
%WINGLOADS Computes lift, shear, bending and hinge loads
%   loads = Structures.WingLoads(ac)
%
%   Compatible with existing ac struct.
%   Safe defaults included to avoid runtime errors if fields are missing.

if nargin < 1
    error('Structures:WingLoads:noInput', ...
        'WingLoads requires the aircraft struct "ac" as input.');
end

g = 9.81;

%% --- SAFE INPUT EXTRACTION ---
% Geometry
b    = getfield_safe(ac, 'geom', 'b',   60);
S    = getfield_safe(ac, 'geom', 'S',   350);
MTOM = getfield_safe(ac, [],     'MTOM', 500000);

% Fold location — nested struct handled safely
if isstruct(ac) && isfield(ac,'struct') && ...
   isstruct(ac.struct) && isfield(ac.struct,'fold') && ...
   isstruct(ac.struct.fold) && isfield(ac.struct.fold,'y_location')
    y_fold = ac.struct.fold.y_location;
else
    y_fold = 0.75 * b / 2;   % default: fold at 75% semi-span
end

% Flight conditions
rho = getfield_safe(ac, 'flight', 'rho_cruise', 0.38);
V   = getfield_safe(ac, 'flight', 'V_cruise',   230);
Ude = getfield_safe(ac, 'flight', 'Ude',         15);

% Structural limit
n_limit = getfield_safe(ac, 'struct', 'n_limit', 2.5);

a = 5.7;   % lift curve slope (per rad)
W = MTOM * g;

%% --- GUST + MANOEUVRE LOAD FACTOR ---
WS      = W / S;
delta_n = (rho * V * a * Ude) / (2 * WS);
n_gust  = 1 + delta_n;
n       = max(n_limit, n_gust);

%% --- SPANWISE GRID ---
y = linspace(0, b/2, 300);

%% --- ELLIPTICAL LIFT DISTRIBUTION ---
L0 = (2 * n * W) / (pi * b/2);
L  = L0 .* sqrt(max(0, 1 - (2*y/b).^2));   % max(0,...) prevents -ve under sqrt

%% --- SHEAR FORCE (integrate tip to root) ---
Vspan = flip(cumtrapz(flip(y), flip(L)));

%% --- BENDING MOMENT (integrate tip to root) ---
Mspan = flip(cumtrapz(flip(y), flip(Vspan)));

%% --- HINGE LOADS (outboard of fold station only) ---
idx   = y >= y_fold;
y_out = y(idx);
L_out = L(idx);

if numel(y_out) < 2
    % Fold station is beyond tip — no outboard segment
    M_fold = 0;
    V_fold = 0;
else
    M_fold = trapz(y_out, L_out .* (y_out - y_fold));
    V_fold = trapz(y_out, L_out);
end

%% --- OUTPUT STRUCT ---
loads.y        = y;
loads.L        = L;
loads.V        = Vspan;
loads.M        = Mspan;
loads.M_root   = Mspan(1);
loads.V_root   = Vspan(1);
loads.M_fold   = M_fold;
loads.V_fold   = V_fold;
loads.n_design = n;

end


%% ===== HELPER: SAFE FIELD ACCESS (two-level) =====
function val = getfield_safe(s, subfield, field, default)
% getfield_safe  Safely extract s.subfield.field or s.field
%   If subfield is [], looks directly in s.field
    try
        if isempty(subfield)
            if isstruct(s) && isfield(s, field)
                val = s.(field);
            else
                val = default;
            end
        else
            if isstruct(s) && isfield(s, subfield) && ...
               isstruct(s.(subfield)) && isfield(s.(subfield), field)
                val = s.(subfield).(field);
            else
                val = default;
            end
        end
    catch
        val = default;
    end
end