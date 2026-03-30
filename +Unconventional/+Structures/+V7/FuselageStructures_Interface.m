%% FuselageStructures_Interface — Wing Structures outputs for Role 3

% ── Root loads — 2.5g  (governing upbend case) ───────────────────────
Q_root     = WS.Q_root;           % N    root shear force
M_root     = WS.M_root;           % Nm   root bending moment
T_root     = WS.T_root;           % Nm   root torque

% ── Root loads — neg1g  (downbend check) ─────────────────────────────
Q_root_n1g = WS.Q_root_n1g;       % N
M_root_n1g = WS.M_root_n1g;       % Nm

% ── Hinge loads (fold station) ────────────────────────────────────────
Q_hinge    = WS.Q_hinge;          % N
M_hinge    = WS.M_hinge;          % Nm
T_hinge    = WS.T_hinge;          % Nm

% ── Root stiffness for junction design ───────────────────────────────
EI_root    = WS.EI_root;          % Nm²
GJ_root    = WS.GJ_root;          % Nm²

% ── Geometry ─────────────────────────────────────────────────────────
c_root     = WS.c_root;           % m   root chord
Span       = WS.Span;             % m   full flight span

fprintf('\n--- Wing Structures → Fuselage Structures handoff ---\n');
fprintf('  Root loads (2.5g — governs):\n');
fprintf('    Q_root   %8.3f MN\n',  abs(Q_root)/1e6);
fprintf('    M_root   %8.3f MNm\n', abs(M_root)/1e6);
fprintf('    T_root   %8.3f MNm\n', abs(T_root)/1e6);
fprintf('  Root loads (neg-1g — downbend):\n');
fprintf('    Q_root   %8.3f MN\n',  abs(Q_root_n1g)/1e6);
fprintf('    M_root   %8.3f MNm\n', abs(M_root_n1g)/1e6);
fprintf('  Hinge loads (2.5g):\n');
fprintf('    Q_hinge  %8.3f MN\n',  abs(Q_hinge)/1e6);
fprintf('    M_hinge  %8.3f MNm\n', abs(M_hinge)/1e6);
fprintf('    T_hinge  %8.3f MNm\n', abs(T_hinge)/1e6);
fprintf('  Root stiffness:\n');
fprintf('    EI_root  %.3e Nm²\n',  EI_root);
fprintf('    GJ_root  %.3e Nm²\n',  GJ_root);
