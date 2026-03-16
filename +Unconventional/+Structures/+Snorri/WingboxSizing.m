function W = WingboxSizing(p, G, S, material)

arguments
    p        struct
    G        struct
    S        struct
    material string = 'Al'
end

switch upper(material)
    case 'AL'
        mat      = p.Al;
        mat_name = 'Aluminium 7075-T6';
    case 'CF'
        mat      = p.CF;
        mat_name = 'CFRP (quasi-isotropic)';
    otherwise
        error('WingboxSizing: material must be Al or CF');
end

sig_all  = mat.sig_all;
tau_all  = mat.tau_all;
E_mat    = mat.E;
nu       = mat.nu;
t_min    = mat.t_min;
tw_min   = mat.t_web_min;
A_min    = mat.A_cap_min;

N           = G.N;
rib_spacing = p.rib_spacing;
k_buckle    = p.k_buckle;

t_skin = zeros(1, N);
A_cap  = zeros(1, N);
t_web  = zeros(1, N);

for i = 1:N

    h  = G.h_wb(i);
    w  = G.w_wb(i);
    Ae = G.A_enc(i);

    Mi = abs(S.M(i));
    Qi = abs(S.Q(i));
    Ti = abs(S.T(i));

    % =================================================================
    %  (a) SPAR CAP AREA  — bending moment  (Snorri Ch5 / Megson §12)
    %
    %  Snorri: sigma = M*y/I
    %  Idealised wingbox: I = 4*A_cap*(h/2)^2  (four caps at ±h/2)
    %  sigma_max = M*(h/2) / [4*A_cap*(h/2)^2] = M / [2*A_cap*h]
    %  Setting sigma_max = sig_all:
    %    A_cap = M / (2 * sig_all * h)
    %  Skin carries ~40% of bending so caps carry 60%:
    %    A_cap = 0.60 * M / (2 * sig_all * h)
    % =================================================================
    if h > 1e-6 && Mi > 0
        A_cap(i) = 0.60 * Mi / (2.0 * sig_all * h);
    end
    A_cap(i) = max(A_cap(i), A_min);

    % =================================================================
    %  (b) SKIN THICKNESS  — torque  (Snorri Ch5 / Bredt-Batho)
    %
    %  Snorri: tau = T / (2*A*t)   where A = enclosed wingbox area
    %  Solving for t:  t_skin = T / (2 * A_enc * tau_all)
    % =================================================================
    if Ae > 1e-8 && Ti > 0
        t_skin(i) = Ti / (2 * Ae * tau_all);
    end

    % =================================================================
    %  (b2) SKIN BUCKLING CHECK  (Snorri Ch5)
    %
    %  sigma_cr = k*pi^2*E / [12*(1-nu^2)] * (t/b_rib)^2
    %  If sigma_cr < sig_all, skin buckles before yielding -> increase t
    %  Solving t_buckle from sigma_cr = sig_all:
    %    t_buckle = b_rib * sqrt(12*(1-nu^2)*sig_all / (k*pi^2*E))
    % =================================================================
    t_buckle = rib_spacing * sqrt(12*(1 - nu^2)*sig_all / (k_buckle*pi^2*E_mat));
    t_skin(i) = max(t_skin(i), t_buckle);
    t_skin(i) = max(t_skin(i), t_min);

    % =================================================================
    %  (c) SPAR WEB THICKNESS  — shear force  (Snorri Ch5)
    %
    %  Snorri: tau = V / A_web  (average shear)
    %  Two webs: A_web = 2*h*t_web
    %    t_web = Q / (2 * h * tau_all)
    %
    %  Jourawski check: peak shear stress at web mid-height exceeds average
    %  For rectangular web: tau_max = 1.5 * tau_avg  (parabolic distribution)
    %  So size for tau_max = tau_all:
    %    t_web = 1.5 * Q / (2 * h * tau_all)
    % =================================================================
    if h > 1e-6 && Qi > 0
        t_web(i) = 1.5 * Qi / (2 * h * tau_all);
    end
    t_web(i) = max(t_web(i), tw_min);

end

t_skin_mm = t_skin * 1e3;
A_cap_cm2 = A_cap  * 1e4;
t_web_mm  = t_web  * 1e3;

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
W.E_mat     = E_mat;
W.G_mat     = mat.G;
W.nu        = nu;

ih = G.i_hinge;

fprintf('\n--- Wingbox Sizing: %s (%s) ---\n', S.loadcase, mat_name);
fprintf('  Allowable stress:   %.0f MPa\n',  sig_all/1e6);
fprintf('  Allowable shear:    %.0f MPa\n',  tau_all/1e6);
fprintf('  Buckling skin t:    %.2f mm  (rib spacing %.0f mm)\n', ...
    1e3*rib_spacing*sqrt(12*(1-nu^2)*sig_all/(k_buckle*pi^2*E_mat)), rib_spacing*1e3);
fprintf('  Root:\n');
fprintf('    Skin thickness:   %.2f mm\n',   t_skin_mm(end));
fprintf('    Spar cap area:    %.1f cm^2\n', A_cap_cm2(end));
fprintf('    Spar web thick:   %.2f mm\n',   t_web_mm(end));
fprintf('  Hinge (y=%.1fm):\n', p.y_hinge);
fprintf('    Skin thickness:   %.2f mm\n',   t_skin_mm(ih));
fprintf('    Spar cap area:    %.1f cm^2\n', A_cap_cm2(ih));
fprintf('    Spar web thick:   %.2f mm\n',   t_web_mm(ih));
fprintf('  Tip (min gauge):\n');
fprintf('    Skin thickness:   %.2f mm\n',   t_skin_mm(1));
fprintf('    Spar cap area:    %.1f cm^2\n', A_cap_cm2(1));

end
