function S = SMT(p, G, L)
% =========================================================================
% SMT.m  —  +Structures package
% Spanwise integration: Shear Force, Bending Moment, Torque.
%
% Integration direction: TIP → ROOT (free-end boundary conditions).
%   At wing tip:  Q(tip) = 0,  M(tip) = 0,  T(tip) = 0
%   This is the correct B.C. for a cantilevered beam loaded from outside.
%   (Cooper GDP lecture slide 17: "start at tip where BCs are known")
%
% Method: trapezoidal integration (second-order accurate).
%
% INPUT:
%   p   — AircraftParams struct
%   G   — WingGeometry struct
%   L   — LoadDistribution struct
%
% OUTPUT:  S — struct (all arrays in TIP → ROOT order, matching G.y)
%   S.Q        [N]    shear force
%   S.M        [Nm]   bending moment
%   S.T        [Nm]   torque
%   S.Q_root   [N]    root shear force
%   S.M_root   [Nm]   root bending moment
%   S.T_root   [Nm]   root torque
%   S.Q_hinge  [N]    shear force at fold hinge
%   S.M_hinge  [Nm]   bending moment at fold hinge
%   S.T_hinge  [Nm]   torque at fold hinge
%
% NO external dependencies.
% =========================================================================

N  = G.N;
dy = G.dy;

% -------------------------------------------------------------------------
%  PRE-ALLOCATE
% -------------------------------------------------------------------------
Q = zeros(1, N);    % Shear Force  [N]
M = zeros(1, N);    % Bending Moment [Nm]
T = zeros(1, N);    % Torque [Nm]

% -------------------------------------------------------------------------
%  INTEGRATION  (trapezoidal, tip → root)
%  Station 1 = tip (free end, Q=M=T=0)
%  Station N = root
%
%  At each step from i to i+1 (moving inboard):
%    dQ = 0.5*(net_dist(i) + net_dist(i+1)) * dy   [trapezoidal]
%    dM = 0.5*(Q(i) + Q(i+1)) * dy
%    dT = lift(i) * e_ac_fa(i) * dy                [torque from lift offset]
% -------------------------------------------------------------------------
for i = 1 : N-1

    % --- Shear Force
    % Accumulate net distributed load from tip inward
    Q(i+1) = Q(i) + 0.5*(L.net_dist(i) + L.net_dist(i+1)) * dy;

    % --- Engine point load
    % Apply when integration crosses the engine spanwise station.
    % Engine hangs below wing → acts DOWNWARD.
    % In +ve manoeuvre (lift up): engine load OPPOSES lift → relieves root BM
    % In -ve manoeuvre (lift down): engine load ADDS to downward net
    % Sign: in our convention net_dist already accounts for sign of loads;
    %       engine is a point mass, treat separately for clarity.
    y_curr = G.y(i+1);
    y_prev = G.y(i);
    if y_curr <= p.y_engine && y_prev > p.y_engine
        % Crossing the engine station: add point load
        % Convention matches net_dist sign:
        %   +ve manoeuvre: engine relief acts same direction as inertia relief
        %   i.e., sign_relief * P_engine
        Q(i+1) = Q(i+1) + L.sign_relief * L.P_engine;
    end

    % --- Bending Moment
    % Integrate shear from tip inward (moment arm grows inward)
    M(i+1) = M(i) + 0.5*(Q(i) + Q(i+1)) * dy;

    % --- Torque
    % dT = aerodynamic pitching contribution at this station
    % Torque arm = offset between aerodynamic centre and flexural axis
    % l(y) * (x_ac - x_fa) * dy
    % Note: using LIFT only for torque (not net load) — structural inertia
    %       acts through the elastic axis and produces no torque about it
    torque_contrib = L.sign_lift * L.lift_dist(i) * G.e_ac_fa(i) * dy;
    T(i+1) = T(i) + torque_contrib;

end

% -------------------------------------------------------------------------
%  EXTRACT KEY VALUES
% -------------------------------------------------------------------------
% Root values (station N, last in tip→root array)
Q_root  = Q(N);
M_root  = M(N);
T_root  = T(N);

% Hinge values (at fold hinge station)
ih = G.i_hinge;
Q_hinge = Q(ih);
M_hinge = M(ih);
T_hinge = T(ih);

% -------------------------------------------------------------------------
%  SIGN CORRECTION FOR OUTPUT
%  For positive manoeuvre the root BM should be positive (hogging).
%  If integration gives negative values due to sign convention, flip.
%  Store both raw and absolute for easy plotting.
% -------------------------------------------------------------------------

% -------------------------------------------------------------------------
%  PACK OUTPUT
% -------------------------------------------------------------------------
S.Q        = Q;
S.M        = M;
S.T        = T;
S.Q_root   = Q_root;
S.M_root   = M_root;
S.T_root   = T_root;
S.Q_hinge  = Q_hinge;
S.M_hinge  = M_hinge;
S.T_hinge  = T_hinge;
S.loadcase = L.loadcase;
S.n_limit  = L.n_limit;
S.n_ult    = L.n_ult;

% -------------------------------------------------------------------------
%  PRINT SUMMARY
% -------------------------------------------------------------------------
fprintf('\n--- SMT Integration: %s ---\n', L.loadcase);
fprintf('  Root shear force:        %8.3f MN\n',  abs(Q_root)/1e6);
fprintf('  Root bending moment:     %8.3f MNm\n', abs(M_root)/1e6);
fprintf('  Root torque:             %8.3f MNm\n', abs(T_root)/1e6);
fprintf('  Hinge shear force:       %8.3f MN\n',  abs(Q_hinge)/1e6);
fprintf('  Hinge bending moment:    %8.3f MNm\n', abs(M_hinge)/1e6);
fprintf('  Hinge torque:            %8.3f MNm\n', abs(T_hinge)/1e6);
fprintf('  BM hinge / BM root:      %8.1f%%\n',   abs(M_hinge/M_root)*100);

end