function D = StiffnessDistribution(p, G, W)
% =========================================================================
% Compute bending stiffness EI(y) and torsional stiffness GJ(y).
%
%   EI — Euler-Bernoulli bending stiffness
%   GJ — Bredt-Batho torsional stiffness
%
% INPUT:
%   p   — AircraftParams struct
%   G   — WingGeometry struct
%   W   — WingboxSizing struct
%
% OUTPUT:  D — struct
%   D.EI       [Nm^2]  bending stiffness at each station (tip→root)
%   D.GJ       [Nm^2]  torsional stiffness at each station
%   D.EI_root  [Nm^2]  root value
%   D.GJ_root  [Nm^2]  root value
%   D.EI_hinge [Nm^2]  value at fold hinge
%   D.GJ_hinge [Nm^2]  value at fold hinge
%   D.EI_tip   [Nm^2]  tip value
%   D.GJ_tip   [Nm^2]  tip value
%
% =========================================================================

N  = G.N;
ih = G.i_hinge;

EI = zeros(1, N);
GJ = zeros(1, N);

E = W.E_mat;
Gm = W.G_mat;

for i = 1:N
    h  = G.h_wb(i);
    w  = G.w_wb(i);
    Ae = G.A_enc(i);

    t_s = W.t_skin(i);
    A_c = W.A_cap(i);

    % -----------------------------------------------------------------
    %  BENDING STIFFNESS  EI
    % -----------------------------------------------------------------
    if h > 1e-6
        I_caps = 4 * A_c * (h/2)^2;
        I_skin = 2 * (t_s * w) * (h/2)^2;
        I_total = I_caps + I_skin;
        EI(i) = E * I_total;
    end

    % -----------------------------------------------------------------
    %  TORSIONAL STIFFNESS  GJ
    % -----------------------------------------------------------------
    if Ae > 1e-8 && t_s > 1e-8 && (w + h) > 1e-6
        perimeter = 2 * (w + h);
        J_box     = 4 * Ae^2 / (perimeter / t_s);
        GJ(i)     = Gm * J_box;
    end
end

% -------------------------------------------------------------------------
%  PACK OUTPUT
% -------------------------------------------------------------------------
D.EI       = EI;
D.GJ       = GJ;
D.EI_root  = EI(N);      % station N = root (tip→root order)
D.GJ_root  = GJ(N);
D.EI_hinge = EI(ih);
D.GJ_hinge = GJ(ih);
D.EI_tip   = EI(1);      % station 1 = tip
D.GJ_tip   = GJ(1);

% -------------------------------------------------------------------------
%  PRINT SUMMARY
% -------------------------------------------------------------------------
fprintf('\n--- Stiffness Distributions (%s) ---\n', W.mat_name);
fprintf('  EI at root:    %.4e Nm^2\n', D.EI_root);
fprintf('  EI at hinge:   %.4e Nm^2  (y=%.1fm)\n', D.EI_hinge, p.y_hinge);
fprintf('  EI at tip:     %.4e Nm^2\n', D.EI_tip);
fprintf('  EI_tip/EI_root ratio:  %.4f\n', D.EI_tip/D.EI_root);
fprintf('  GJ at root:    %.4e Nm^2\n', D.GJ_root);
fprintf('  GJ at hinge:   %.4e Nm^2\n', D.GJ_hinge);
fprintf('  GJ at tip:     %.4e Nm^2\n', D.GJ_tip);
fprintf('  GJ_tip/GJ_root ratio:  %.4f\n', D.GJ_tip/D.GJ_root);

end