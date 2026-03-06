function W = WingboxSizing(p, G, S, material)
% =========================================================================
% WingboxSizing.m  —  +Structures package
% Size wingbox cross-section at every spanwise station from SMT loads.
%
% Three sizing criteria applied at each station:
%   (a) Spar caps  ← bending moment M  (Megson §12 idealised wingbox)
%   (b) Skin       ← torque T          (Bredt-Batho thin-wall torsion)
%   (c) Spar webs  ← shear force Q     (thin-wall shear flow)
%
% Additional check:
%   (d) Minimum gauge enforced (manufacturing constraint)
%
% INPUT:
%   p        — AircraftParams struct
%   G        — WingGeometry struct
%   S        — SMT struct
%   material — string: 'Al' (default) | 'CF'
%
% OUTPUT:  W — struct (all arrays in TIP → ROOT order, matching G.y)
%   W.t_skin    [m]    skin thickness (torsion-driven)
%   W.A_cap     [m^2]  spar cap area, each cap (bending-driven)
%   W.t_web     [m]    spar web thickness (shear-driven)
%   W.t_skin_mm [mm]   convenience copy in mm
%   W.A_cap_cm2 [cm^2] convenience copy in cm^2
%   W.t_web_mm  [mm]   convenience copy in mm
%   W.material  string
%   W.sig_all   [Pa]   allowable stress used
%   W.tau_all   [Pa]   allowable shear stress used
%
% NO external dependencies.
% =========================================================================

arguments
    p        struct
    G        struct
    S        struct
    material string = 'Al'
end

% -------------------------------------------------------------------------
%  SELECT MATERIAL PROPERTIES
% -------------------------------------------------------------------------
switch upper(material)
    case 'AL'
        mat       = p.Al;
        mat_name  = 'Aluminium 7075-T6';
    case 'CF'
        mat       = p.CF;
        mat_name  = 'CFRP (quasi-isotropic)';
    otherwise
        error('Use Al or CF as material input');
end

sig_all  = mat.sig_all;
tau_all  = mat.tau_all;
t_min    = mat.t_min;
tw_min   = mat.t_web_min;
A_min    = mat.A_cap_min;

N = G.N;

% Pre-allocate
t_skin = zeros(1, N);
A_cap  = zeros(1, N);
t_web  = zeros(1, N);

% -------------------------------------------------------------------------
%  SIZE EACH STATION
% -------------------------------------------------------------------------
for i = 1:N

    % Local geometry
    h  = G.h_wb(i);        % wingbox height [m]
    w  = G.w_wb(i);        % wingbox width  [m]
    Ae = G.A_enc(i);       % enclosed area  [m^2]

    % Applied loads (use absolute values for sizing)
    Mi = abs(S.M(i));
    Qi = abs(S.Q(i));
    Ti = abs(S.T(i));

    % -----------------------------------------------------------------
    %  (a) SPAR CAP AREA — from bending moment
    %
    %  Idealised wingbox model (Megson Chapter 12):
    %    All bending carried by concentrated spar cap areas.
    %    σ = M * z / I
    %    For symmetric box: I ≈ 2 * A_cap * (h/2)^2  (caps at ±h/2)
    %    σ_max = M * (h/2) / I = M / (2 * A_cap * h/2) = M / (A_cap * h)
    %    → A_cap = M / (σ_all * h)
    % -----------------------------------------------------------------
    if h > 1e-6 && Mi > 0
        A_cap(i) = Mi / (2.0 * sig_all * h);
    end
    A_cap(i) = max(A_cap(i), A_min);    % minimum gauge

    % -----------------------------------------------------------------
    %  (b) SKIN THICKNESS — from torque (Bredt-Batho)
    %
    %  Closed thin-walled section (Cooper slide 23, Megson §17):
    %    q = T / (2 * A_enclosed)      [shear flow, N/m]
    %    τ = q / t                     [shear stress]
    %    t = T / (2 * A_enc * τ_all)
    % -----------------------------------------------------------------
    if Ae > 1e-8 && Ti > 0
        t_skin(i) = Ti / (2 * Ae * tau_all);
    end
    t_skin(i) = max(t_skin(i), t_min);  % minimum gauge

    % -----------------------------------------------------------------
    %  (c) SPAR WEB THICKNESS — from shear force
    %
    %  Two spar webs carry vertical shear equally (symmetric loading).
    %  Simplified uniform shear stress:
    %    τ = Q / (2 * h * t_web)
    %    → t_web = Q / (2 * h * τ_all)
    %
    %  Note: this is a lower bound. Refined rib spacing / shear lag
    %        effects handled in Sprint 3 refinement.
    % -----------------------------------------------------------------
    if h > 1e-6 && Qi > 0
        t_web(i) = Qi / (2 * h * tau_all);
    end
    t_web(i) = max(t_web(i), tw_min);   % minimum gauge

end

% -------------------------------------------------------------------------
%  CONVENIENCE UNIT COPIES
% -------------------------------------------------------------------------
t_skin_mm = t_skin * 1e3;
A_cap_cm2 = A_cap  * 1e4;
t_web_mm  = t_web  * 1e3;

% -------------------------------------------------------------------------
%  PACK OUTPUT
% -------------------------------------------------------------------------
W.t_skin    = t_skin;
W.A_cap     = A_cap;
W.t_web     = t_web;
W.t_skin_mm = t_skin_mm;
W.A_cap_cm2 = A_cap_cm2;
W.t_web_mm  = t_web_mm;
W.material  = material;
W.mat_name  = mat_name;
W.sig_all   = sig_all;
W.tau_all   = tau_all;
W.rho_mat   = mat.rho;
W.E_mat     = mat.E;
W.G_mat     = mat.G;

% -------------------------------------------------------------------------
%  PRINT SUMMARY
% -------------------------------------------------------------------------
fprintf('\n--- Wingbox Sizing: %s (%s) ---\n', S.loadcase, mat_name);
fprintf('  Allowable stress:   %.0f MPa\n',  sig_all/1e6);
fprintf('  Allowable shear:    %.0f MPa\n',  tau_all/1e6);
fprintf('  Root:\n');
fprintf('    Skin thickness:   %.2f mm\n',   t_skin_mm(end));
fprintf('    Spar cap area:    %.1f cm^2\n', A_cap_cm2(end));
fprintf('    Spar web thick:   %.2f mm\n',   t_web_mm(end));
fprintf('  Hinge (y=%.1fm):\n', p.y_hinge);
ih = G.i_hinge;
fprintf('    Skin thickness:   %.2f mm\n',   t_skin_mm(ih));
fprintf('    Spar cap area:    %.1f cm^2\n', A_cap_cm2(ih));
fprintf('    Spar web thick:   %.2f mm\n',   t_web_mm(ih));
fprintf('  Tip:\n');
fprintf('    Skin thickness:   %.2f mm  (min gauge)\n', t_skin_mm(1));
fprintf('    Spar cap area:    %.1f cm^2  (min gauge)\n', A_cap_cm2(1));

end