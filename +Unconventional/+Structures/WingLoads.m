function loads = WingLoads(ac)
%WINGLOADS Computes lift, shear, bending and hinge loads
% Compatible with existing ac struct
% Safe defaults included to avoid runtime errors

g = 9.81;

%% --- SAFE INPUT EXTRACTION ---

b = getfield_safe(ac.geom,'b',60);
S = getfield_safe(ac.geom,'S',350);
MTOM = getfield_safe(ac,'MTOM',500000);
y_fold = getfield_safe(ac.struct.fold,'y_location',0.75*b/2);

rho = getfield_safe(ac.flight,'rho_cruise',0.38);
V = getfield_safe(ac.flight,'V_cruise',230);
Ude = getfield_safe(ac.flight,'Ude',15);
a = 5.7; % lift curve slope per rad

W = MTOM * g;

%% --- GUST + MANOEUVRE LOAD FACTOR ---

WS = W/S;
delta_n = (rho * V * a * Ude) / (2 * WS);
n_gust = 1 + delta_n;

n_limit = getfield_safe(ac.struct,'n_limit',2.5);
n = max(n_limit,n_gust);

%% --- SPANWISE GRID ---

y = linspace(0,b/2,300);

%% --- ELLIPTICAL LIFT DISTRIBUTION ---

L0 = (2*n*W)/(pi*b/2);
L = L0 .* sqrt(1 - (2*y/b).^2);

%% --- SHEAR ---

Vspan = flip(cumtrapz(flip(y),flip(L)));

%% --- BENDING ---

Mspan = flip(cumtrapz(flip(y),flip(Vspan)));

%% --- HINGE LOADS (OUTBOARD SEGMENT ONLY) ---

idx = y >= y_fold;
y_out = y(idx);
L_out = L(idx);

M_fold = trapz(y_out, L_out .* (y_out - y_fold));
V_fold = trapz(y_out, L_out);

%% --- OUTPUT STRUCT ---

loads.y = y;
loads.L = L;
loads.V = Vspan;
loads.M = Mspan;

loads.M_root = Mspan(1);
loads.V_root = Vspan(1);

loads.M_fold = M_fold;
loads.V_fold = V_fold;
loads.n_design = n;

end


%% ===== SAFE FIELD FUNCTION =====
function val = getfield_safe(s,field,default)
if isstruct(s) && isfield(s,field)
    val = s.(field);
else
    val = default;
end
end