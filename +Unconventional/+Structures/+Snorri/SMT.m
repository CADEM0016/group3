function S = SMT(p, G, L)

N  = G.N;
dy = G.dy;

Q = zeros(1, N);
M = zeros(1, N);
T = zeros(1, N);

% =========================================================================
%  SPANWISE INTEGRATION  tip -> root  (free-end BCs: Q=M=T=0 at tip)
%
%  Snorri Ch6 reference equations (uniform distribution, for comparison):
%    V(y) = (L/2)*(1 - 2y/b)
%    M(y) = (L/2)*(b*y/2 - y^2/2)
%  This code integrates the full elliptical + inertia-relief distribution
%  using second-order trapezoidal quadrature, which is more accurate.
% =========================================================================
for i = 1 : N-1

    % Shear force: integrate net distributed load inboard
    Q(i+1) = Q(i) + 0.5*(L.net_dist(i) + L.net_dist(i+1)) * dy;

    % Engine point load: applied as discrete jump when crossing y_engine
    y_curr = G.y(i+1);
    y_prev = G.y(i);
    if y_curr <= p.y_engine && y_prev > p.y_engine
        Q(i+1) = Q(i+1) + L.sign_relief * L.P_engine;
    end

    % Bending moment: integrate shear inboard
    M(i+1) = M(i) + 0.5*(Q(i) + Q(i+1)) * dy;

    % Torque: aerodynamic lift acts at AC; structural loads act at FA
    % Only lift contributes torque (inertia loads act through elastic axis)
    % dT = sign_lift * l(y) * e_ac_fa(y) * dy
    T(i+1) = T(i) + L.sign_lift * L.lift_dist(i) * G.e_ac_fa(i) * dy;

end

% =========================================================================
%  HINGE LOADS  (Snorri folding tip: M_hinge = L_tip * d)
%  L_tip is the outboard panel lift from LoadDistribution
%  d = distance from hinge to tip panel centroid ~ b_outer/2
%  The SMT integration gives the exact value; Snorri gives the estimate.
% =========================================================================
b_outer      = G.s - p.y_hinge;
d_tip_moment = b_outer / 2;
M_hinge_snorri = L.L_tip * d_tip_moment;

% ---- Extract key station values
Q_root  = Q(N);
M_root  = M(N);
T_root  = T(N);

ih       = G.i_hinge;
Q_hinge  = Q(ih);
M_hinge  = M(ih);
T_hinge  = T(ih);

% ---- Pack output
S.Q              = Q;
S.M              = M;
S.T              = T;
S.Q_root         = Q_root;
S.M_root         = M_root;
S.T_root         = T_root;
S.Q_hinge        = Q_hinge;
S.M_hinge        = M_hinge;
S.T_hinge        = T_hinge;
S.M_hinge_snorri = M_hinge_snorri;
S.loadcase       = L.loadcase;
S.n_limit        = L.n_limit;
S.n_ult          = L.n_ult;

fprintf('\n--- SMT Integration: %s ---\n', L.loadcase);
fprintf('  Root shear force:        %8.3f MN\n',  abs(Q_root)/1e6);
fprintf('  Root bending moment:     %8.3f MNm\n', abs(M_root)/1e6);
fprintf('  Root bending (Snorri):   %8.3f MNm  (L*b/8 check)\n', L.M_root_snorri/1e6);
fprintf('  Root torque:             %8.3f MNm\n', abs(T_root)/1e6);
fprintf('  Hinge shear force:       %8.3f MN\n',  abs(Q_hinge)/1e6);
fprintf('  Hinge bending (SMT):     %8.3f MNm\n', abs(M_hinge)/1e6);
fprintf('  Hinge bending (Snorri):  %8.3f MNm  (L_tip*d)\n', M_hinge_snorri/1e6);
fprintf('  Hinge torque:            %8.3f MNm\n', abs(T_hinge)/1e6);
fprintf('  BM hinge / BM root:      %8.1f%%\n',   abs(M_hinge/M_root)*100);

end
