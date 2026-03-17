function S = SMT(loc, G, L)

N  = G.N;
dy = G.dy;
ih = G.i_hinge;

Q = zeros(1, N);
M = zeros(1, N);
T = zeros(1, N);

% Trapezoidal integration tip→root.  Station 1 is the free end: Q=M=T=0.
for i = 1 : N-1

    Q(i+1) = Q(i) + 0.5*(L.net_dist(i) + L.net_dist(i+1)) * dy;

    % Engine point load applied as a jump when the step crosses y_engine
    if G.y(i+1) <= G.y_engine && G.y(i) > G.y_engine
        Q(i+1) = Q(i+1) + L.sign_relief * L.P_engine;
    end

    M(i+1) = M(i) + 0.5*(Q(i) + Q(i+1)) * dy;

    % Only aerodynamic lift torques the box; inertia loads act through elastic axis
    T(i+1) = T(i) + L.sign_lift * L.lift_dist(i) * G.e_ac_fa(i) * dy;

end

% Snorri hinge BM estimate for cross-check  (M = L_tip × b_outer/2)
M_hinge_snorri = L.L_tip * (G.s - loc.y_hinge) / 2;

S.Q              = Q;
S.M              = M;
S.T              = T;
S.Q_root         = Q(N);
S.M_root         = M(N);
S.T_root         = T(N);
S.Q_hinge        = Q(ih);
S.M_hinge        = M(ih);
S.T_hinge        = T(ih);
S.M_hinge_snorri = M_hinge_snorri;
S.loadcase       = L.loadcase;
S.n_limit        = L.n_limit;
S.n_ult          = L.n_ult;

fprintf('\n--- SMT: %s ---\n', L.loadcase);
fprintf('  Root  Q / M / T      %.3f MN  /  %.3f MNm  /  %.3f MNm\n', ...
    abs(S.Q_root)/1e6, abs(S.M_root)/1e6, abs(S.T_root)/1e6);
fprintf('  Root M (Snorri Lb/8) %.3f MNm\n', L.M_root_snorri/1e6);
fprintf('  Hinge Q / M (SMT)    %.3f MN   /  %.3f MNm\n', ...
    abs(S.Q_hinge)/1e6, abs(S.M_hinge)/1e6);
fprintf('  Hinge M (Snorri)     %.3f MNm\n', M_hinge_snorri/1e6);
fprintf('  Hinge M / Root M     %.1f%%\n',   abs(S.M_hinge / S.M_root)*100);

end
