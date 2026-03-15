function D = StiffnessDistribution(p, G, W)

N  = G.N;
ih = G.i_hinge;

EI    = zeros(1, N);
GJ    = zeros(1, N);
theta = zeros(1, N);   % cumulative angle of twist from tip inward

E  = W.E_mat;
Gm = W.G_mat;

for i = 1:N

    h   = G.h_wb(i);
    w   = G.w_wb(i);
    Ae  = G.A_enc(i);
    t_s = W.t_skin(i);
    A_c = W.A_cap(i);

    % =================================================================
    %  BENDING STIFFNESS  EI  (Snorri Ch5 / Euler-Bernoulli)
    %
    %  Snorri: I = sum(A_i * y_i^2)
    %  Four spar caps at distance h/2 from neutral axis:
    %    I_caps = 4 * A_cap * (h/2)^2
    %  Top and bottom skin panels at h/2 from neutral axis:
    %    I_skin = 2 * (t_skin * w) * (h/2)^2
    %  EI = E * (I_caps + I_skin)
    % =================================================================
    if h > 1e-6
        I_caps  = 4 * A_c * (h/2)^2;
        I_skin  = 2 * (t_s * w) * (h/2)^2;
        EI(i)   = E * (I_caps + I_skin);
    end

    % =================================================================
    %  TORSIONAL STIFFNESS  GJ  (Snorri Ch5 / Bredt-Batho)
    %
    %  Snorri: T = GJ * (d_theta/dy)
    %  For closed thin-wall section:
    %    J = 4*A_enc^2 / contour_integral(ds/t)
    %  Uniform skin approximation: contour = 2*(w+h)/t_skin
    %    J = 4*A_enc^2 * t_skin / (2*(w+h))
    %  GJ = G * J
    % =================================================================
    if Ae > 1e-8 && t_s > 1e-8 && (w+h) > 1e-6
        J_box = 4 * Ae^2 * t_s / (2*(w + h));
        GJ(i) = Gm * J_box;
    end

end

% ---- Pack output
D.EI        = EI;
D.GJ        = GJ;
D.EI_root   = EI(N);
D.GJ_root   = GJ(N);
D.EI_hinge  = EI(ih);
D.GJ_hinge  = GJ(ih);
D.EI_tip    = EI(1);
D.GJ_tip    = GJ(1);

fprintf('\n--- Stiffness Distributions (%s) ---\n', W.mat_name);
fprintf('  EI at root:          %.4e Nm^2\n', D.EI_root);
fprintf('  EI at hinge (%.1fm): %.4e Nm^2\n', p.y_hinge, D.EI_hinge);
fprintf('  EI at tip:           %.4e Nm^2\n', D.EI_tip);
fprintf('  EI_tip / EI_root:    %.5f\n',      D.EI_tip / D.EI_root);
fprintf('  GJ at root:          %.4e Nm^2\n', D.GJ_root);
fprintf('  GJ at hinge (%.1fm): %.4e Nm^2\n', p.y_hinge, D.GJ_hinge);
fprintf('  GJ at tip:           %.4e Nm^2\n', D.GJ_tip);
fprintf('  GJ_tip / GJ_root:    %.5f\n',      D.GJ_tip / D.GJ_root);

end
