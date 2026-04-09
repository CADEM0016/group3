function S = SMT(loc, G, L)

N  = G.N;
dy = G.dy;
ih = G.i_hinge;

Q = zeros(1, N);
M = zeros(1, N);
T = zeros(1, N);

% Trapezoidal integration tip→root; BC: Q=M=T=0 at station 1 (tip)
for i = 1 : N-1

    Q(i+1) = Q(i) + 0.5*(L.net_dist(i) + L.net_dist(i+1)) * dy;

    % Inner engine point load (station 1 of 2 per semi-wing)
    if G.y(i+1) <= G.y_engine1 && G.y(i) > G.y_engine1
        Q(i+1) = Q(i+1) + L.sign_relief * L.P_engine1;
    end

    % Outer engine point load (station 2 of 2 per semi-wing)
    if G.y(i+1) <= G.y_engine2 && G.y(i) > G.y_engine2
        Q(i+1) = Q(i+1) + L.sign_relief * L.P_engine2;
    end

    M(i+1) = M(i) + 0.5*(Q(i) + Q(i+1)) * dy;

    T(i+1) = T(i) + L.sign_lift * L.lift_dist(i) * G.e_ac_fa(i) * dy;

end

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