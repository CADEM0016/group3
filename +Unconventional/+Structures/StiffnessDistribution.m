function D = StiffnessDistribution(G, W)

N  = G.N;
ih = G.i_hinge;

EI = zeros(1, N);
GJ = zeros(1, N);

for i = 1:N

    h   = G.h_wb(i);
    w   = G.w_wb(i);
    Ae  = G.A_enc(i);
    t_s = W.t_skin(i);
    A_c = W.A_cap(i);

    % EI: parallel axis - four caps + top/bottom skins at h/2
    if h > 1e-6
        I     = 4*A_c*(h/2)^2 + 2*(t_s*w)*(h/2)^2;
        EI(i) = W.E_mat * I;
    end

    % GJ: Bredt-Batho closed section, perimeter = 2(w+h)
    if Ae > 1e-8 && t_s > 1e-8 && (w+h) > 1e-6
        J     = 4 * Ae^2 * t_s / (2*(w+h));
        GJ(i) = W.G_mat * J;
    end

end

D.EI       = EI;
D.GJ       = GJ;
D.EI_root  = EI(N);
D.GJ_root  = GJ(N);
D.EI_hinge = EI(ih);
D.GJ_hinge = GJ(ih);
D.EI_tip   = EI(1);
D.GJ_tip   = GJ(1);

fprintf('\n--- Stiffness: %s ---\n', W.mat_name);
fprintf('  EI  root / hinge / tip   %.3e / %.3e / %.3e  Nm²\n', D.EI_root, D.EI_hinge, D.EI_tip);
fprintf('  GJ  root / hinge / tip   %.3e / %.3e / %.3e  Nm²\n', D.GJ_root, D.GJ_hinge, D.GJ_tip);
fprintf('  GJ / EI at root          %.3f\n', D.GJ_root / D.EI_root);

end
