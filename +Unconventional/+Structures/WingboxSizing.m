function W = WingboxSizing(loc, G, S, material)

arguments
    loc      struct
    G        struct
    S        struct
    material string = 'Al'
end

switch upper(material)
    case 'AL',  mat = loc.Al;  mat_name = 'Aluminium 7075-T6';
    case 'CF',  mat = loc.CF;  mat_name = 'CFRP (quasi-isotropic)';
    otherwise,  error('WingboxSizing: material must be Al or CF');
end

t_skin = zeros(1, G.N);
A_cap  = zeros(1, G.N);
t_web  = zeros(1, G.N);

% Minimum skin t from buckling (constant spanwise — depends only on rib spacing)
t_buckle = loc.rib_spacing * sqrt(12*(1 - mat.nu^2)*mat.sig_all / (loc.k_buckle*pi^2*mat.E));

for i = 1:G.N

    h  = G.h_wb(i);
    w  = G.w_wb(i);
    Ae = G.A_enc(i);
    Mi = abs(S.M(i));
    Qi = abs(S.Q(i));
    Ti = abs(S.T(i));

    % Spar caps from bending  (idealised 4-cap box: I = 4A(h/2)²; skins carry 40%)
    if h > 1e-6 && Mi > 0
        A_cap(i) = 0.60 * Mi / (2 * mat.sig_all * h);
    end
    A_cap(i) = max(A_cap(i), mat.A_cap_min);

    % Skin from Bredt-Batho torsion  (tau = T/2At)  then buckling check
    if Ae > 1e-8 && Ti > 0
        t_skin(i) = Ti / (2 * Ae * mat.tau_all);
    end
    t_skin(i) = max([t_skin(i), t_buckle, mat.t_min]);

    % Web from Jourawski shear  (tau_max = 1.5 × tau_avg for rectangular web)
    if h > 1e-6 && Qi > 0
        t_web(i) = 1.5 * Qi / (2 * h * mat.tau_all);
    end
    t_web(i) = max(t_web(i), mat.t_web_min);

end

W.t_skin    = t_skin;
W.A_cap     = A_cap;
W.t_web     = t_web;
W.t_skin_mm = t_skin * 1e3;
W.A_cap_cm2 = A_cap  * 1e4;
W.t_web_mm  = t_web  * 1e3;
W.material  = material;
W.mat_name  = mat_name;
W.sig_all   = mat.sig_all;
W.tau_all   = mat.tau_all;
W.rho_mat   = mat.rho;
W.E_mat     = mat.E;
W.G_mat     = mat.G;
W.nu        = mat.nu;

ih = G.i_hinge;

fprintf('\n--- Wingbox Sizing: %s (%s) ---\n', S.loadcase, mat_name);
fprintf('  sig_all / tau_all      %.0f / %.0f MPa\n', mat.sig_all/1e6, mat.tau_all/1e6);
fprintf('  Buckling t (%.0f mm ribs)  %.2f mm\n',     loc.rib_spacing*1e3, t_buckle*1e3);
fprintf('  Root    t_skin=%.2f mm  A_cap=%.1f cm²  t_web=%.2f mm\n', ...
    W.t_skin_mm(end), W.A_cap_cm2(end), W.t_web_mm(end));
fprintf('  Hinge   t_skin=%.2f mm  A_cap=%.1f cm²  t_web=%.2f mm\n', ...
    W.t_skin_mm(ih), W.A_cap_cm2(ih), W.t_web_mm(ih));
fprintf('  Tip     t_skin=%.2f mm  A_cap=%.1f cm²  (min gauge)\n', ...
    W.t_skin_mm(1), W.A_cap_cm2(1));

end
