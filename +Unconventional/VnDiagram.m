%% V-n DIAGRAM (Flight Envelope)
%  Pulls required parameters from the Unconventional ADP model and
%  plots the V-n (velocity – load factor) diagram in the style of
%  the FAR/CS-25 structural flight envelope.
%
%  Required on the MATLAB path:
%    +Unconventional  package (ADP, ConstraintAnalysis, BuildGeometry, etc.)
%    cast / SI        utility packages used by the model
%
%  Usage:
%    VnDiagram          % builds its own ADP from default values
%    VnDiagram(myADP)   % pass an already-sized ADP object

function VnDiagram(ADP)

%% ── 0.  Build / receive ADP ────────────────────────────────────────────
if nargin < 1
    fprintf('No ADP supplied – constructing from SimplifiedPayloadRange4 defaults.\n');
    ADP = Unconventional.ADP();
    ADP.TLAR           = cast.TLAR.Unconventional();
    ADP.TLAR.M_c       = 0.84;
    ADP.FuselageLength = 65;
    ADP.KinkPos        = 10;
    ADP.CabinRadius    = 6.3;
    ADP.CabinLength    = 50;
    ADP.CockpitLength  = 5;
    ADP.WingPos        = 0.44 * ADP.FuselageLength;
    ADP.V_HT           = 0.97;
    ADP.V_VT           = 0.072;
    ADP.HtpPos         = 0.85 * ADP.FuselageLength;
    ADP.VtpPos         = 0.82 * ADP.FuselageLength;
    ADP.WingArea       = 750;
    ADP.Span           = 74;
    ADP.MTOM           = 490000;
    ADP.Mf_Fuel        = 0.32;
    ADP.Mf_res         = 0.03;
    ADP.Mf_Ldg         = 0.62;
    ADP.Mf_TOC         = 0.975;
    % Run constraint analysis to populate WingLoading & Thrust
    Unconventional.ConstraintAnalysis(ADP);
end

%% ── 1.  Extract parameters from ADP ────────────────────────────────────

% ---- Mass / loading ----
MTOM        = ADP.MTOM;                   % [kg]  Maximum Take-Off Mass
W           = MTOM * 9.81;                % [N]   MTOW weight
S           = ADP.WingArea;               % [m²]  Wing reference area
WS          = W / S;                      % [N/m²] Wing loading

% ---- Aerodynamic ----
CLmax_pos   = ADP.Cl_max + ADP.Delta_Cl_to; % positive CLmax (with flaps up ~ manoeuvre)
CLmax_pos   = ADP.Cl_max;                 % clean (manoeuvre envelope)
CLmax_neg   = -0.8 * CLmax_pos;           % negative CLmax (typically ~−0.6 to −1.0)
AR          = ADP.Span^2 / S;            % Aspect ratio (computed property)
e_oswald    = ADP.e;                      % Oswald efficiency
CD0         = ADP.CD0;

% ---- Load factor limits (CS-25 / FAR Part 25 transport category) ----
n_pos_limit  =  2.5;                      % positive limit load factor (CS-25.337)
n_neg_limit  = -1.0;                      % negative limit load factor
n_pos_ult    =  n_pos_limit * 1.5;        % ultimate (×1.5)
n_neg_ult    =  n_neg_limit * 1.5;

% ---- Atmosphere at sea level ----
rho_SL      = 1.225;                      % [kg/m³]

% ---- Speed limits ----
% Cruise Mach & altitude from TLAR
M_c  = ADP.TLAR.M_c;                     % design cruise Mach
h_c  = ADP.TLAR.Alt_cruise;              % cruise altitude [m]

% Speed of sound at sea level (ISA)
T_SL  = 288.15;  gamma = 1.4; R_air = 287.05;
a_SL  = sqrt(gamma * R_air * T_SL);      % [m/s]

% Design cruising speed Vc (EAS) — CS-25.335
% Vc_min = 33*sqrt(WS/rho_SL) [kt] but we derive from Mach & altitude
[rho_c, a_c, ~, ~] = atmos_isa(h_c);
V_cruise_TAS = M_c * a_c;                % TAS at cruise altitude [m/s]
sigma_c      = rho_c / rho_SL;
Vc_EAS       = V_cruise_TAS * sqrt(sigma_c); % EAS [m/s]

% Design dive speed Vd = 1.25*Vc (CS-25.335(b))
Vd_EAS = 1.25 * Vc_EAS;

% Convert to knots for x-axis (EAS)
ms2kt  = 1 / 0.51444;
Vc_kt  = Vc_EAS  * ms2kt;
Vd_kt  = Vd_EAS  * ms2kt;

% Stall speed at 1g (EAS) — VS = sqrt(2*WS / (rho_SL*CLmax))
VS_pos_ms = sqrt(2 * WS / (rho_SL * CLmax_pos));   % [m/s EAS]
VS_neg_ms = sqrt(2 * WS / (rho_SL * abs(CLmax_neg)));

VS_pos_kt = VS_pos_ms * ms2kt;
VS_neg_kt = VS_neg_ms * ms2kt;

% Manoeuvre speed Va = VS * sqrt(n_pos_limit)
Va_kt = VS_pos_kt * sqrt(n_pos_limit);

%% ── 2.  Build envelope curves ──────────────────────────────────────────

V_vec = linspace(0, Vd_kt * 1.05, 800);  % speed sweep [kt EAS]
V_ms  = V_vec / ms2kt;                   % [m/s EAS]

% Stall boundary: n = rho_SL * V² * CLmax / (2 * WS)
n_stall_pos =  0.5 * rho_SL .* V_ms.^2 .* CLmax_pos  ./ WS;
n_stall_neg = -0.5 * rho_SL .* V_ms.^2 .* abs(CLmax_neg) ./ WS;

% Clamp to structural limits
n_stall_pos = min(n_stall_pos,  n_pos_limit);
n_stall_neg = max(n_stall_neg,  n_neg_limit);

%% ── 3.  Build filled regions ────────────────────────────────────────────
% We trace the flight envelope boundary (the "clean" manoeuvre envelope):
%
%  Positive boundary:
%    O(0,0) → along stall curve → A(Va,n+) → B(Vc,n+) → D(Vd,n+) → reduce...
%
%  Full closed polygon for coloured fill:

% --- Positive stall arc (O → A) ---
idx_pos = n_stall_pos >= 0 & n_stall_pos <= n_pos_limit + 0.01;
V_pos_stall = V_vec(idx_pos);
n_pos_stall = n_stall_pos(idx_pos);

% --- Negative stall arc (O → C) ---
idx_neg = n_stall_neg <= 0 & n_stall_neg >= n_neg_limit - 0.01;
V_neg_stall = V_vec(idx_neg);
n_neg_stall = n_stall_neg(idx_neg);

% Key points
A_V = Va_kt;        A_n =  n_pos_limit;
B_V = Vc_kt;        B_n =  n_pos_limit;
D_V = Vd_kt;        D_n =  n_pos_limit;   % some codes slope D down slightly
E_V = Vd_kt;        E_n =  0;
C_V = Vc_kt;        C_n =  n_neg_limit;
F_V = VS_neg_kt;    F_n =  n_neg_limit;   % approx end of neg stall arc

%% ── 4.  Plot ────────────────────────────────────────────────────────────
figure('Name','V-n Diagram','Color','w','Position',[100 100 950 650]);
hold on; grid on; box on;

%--- Normal flight envelope (green fill) ---
Venv_pos  = [V_pos_stall, B_V, D_V];
nenv_pos  = [n_pos_stall, B_n, D_n];
Venv_neg  = [V_neg_stall, C_V, E_V];
nenv_neg  = [n_neg_stall, C_n, 0   ];

% Full closed polygon (positive top, then negative bottom reversed)
Vfill = [Venv_pos, fliplr(Venv_neg)];
nfill = [nenv_pos, fliplr(nenv_neg)];
fill(Vfill, nfill, [0.2 0.7 0.2], 'FaceAlpha', 0.45, 'EdgeColor','none');

%--- Caution region (yellow): Vc→Vd, positive side ---
Vcaution = [Vc_kt, Vd_kt, Vd_kt, Vc_kt];
ncaution = [0,     0,     n_pos_limit, n_pos_limit];
fill(Vcaution, ncaution, [1.0 0.9 0.2], 'FaceAlpha', 0.55, 'EdgeColor','none');

%--- Structural damage likely (orange): beyond Vc on positive side ---
% already covered by caution; overlay for negative side Vc→Vd
Vcaut_neg = [Vc_kt, Vd_kt, Vd_kt, Vc_kt];
ncaut_neg = [n_neg_limit, n_neg_limit, 0, 0];
fill(Vcaut_neg, ncaut_neg, [1.0 0.65 0.1], 'FaceAlpha', 0.50, 'EdgeColor','none');

%--- Structural failure region (red): beyond Vd ---
Vfail = [Vd_kt, Vd_kt*1.06, Vd_kt*1.06, Vd_kt];
nfail_top = [n_pos_limit, n_pos_limit, n_neg_limit, n_neg_limit];
fill(Vfail, nfail_top, [0.85 0.15 0.15], 'FaceAlpha', 0.55, 'EdgeColor','none');

%--- Structural damage above n_pos_limit ---
Vdmg = [0, Vd_kt, Vd_kt, 0];
ndmg = [n_pos_limit, n_pos_limit, n_pos_limit+1.5, n_pos_limit+1.5];
fill(Vdmg, ndmg, [1.0 0.55 0.0], 'FaceAlpha', 0.45, 'EdgeColor','none');

%--- Structural damage below n_neg_limit ---
Vdmg2 = [0, Vd_kt, Vd_kt, 0];
ndmg2 = [n_neg_limit, n_neg_limit, n_neg_limit-0.8, n_neg_limit-0.8];
fill(Vdmg2, ndmg2, [1.0 0.55 0.0], 'FaceAlpha', 0.45, 'EdgeColor','none');

%--- Structural failure above ultimate ---
Vult = [0, Vd_kt, Vd_kt, 0];
nult = [n_pos_ult, n_pos_ult, n_pos_ult+1, n_pos_ult+1];
fill(Vult, nult, [0.85 0.15 0.15], 'FaceAlpha', 0.55, 'EdgeColor','none');

%--- Draw boundary curves ---
% Positive stall curve
plot(V_pos_stall, n_pos_stall, 'k-', 'LineWidth', 2.0);
% Negative stall curve
plot(V_neg_stall, n_neg_stall, 'k-', 'LineWidth', 2.0);
% Horizontal limit lines
plot([Va_kt, B_V, D_V], [n_pos_limit, n_pos_limit, n_pos_limit], 'k-', 'LineWidth', 2.0);
plot([F_V,   C_V],        [n_neg_limit, n_neg_limit],             'k-', 'LineWidth', 2.0);
% Vertical closing line at Vd
plot([Vd_kt, Vd_kt], [n_pos_limit, 0], 'k-', 'LineWidth', 2.0);
plot([Vd_kt, Vd_kt], [0, n_neg_limit], 'k-', 'LineWidth', 2.0);
% Zero-load line & axis
yline(0, 'k-', 'LineWidth', 1.2);
xline(0, 'k-', 'LineWidth', 1.2);
% Level-unaccelerated flight line (n = 1)
yline(1, 'k--', 'LineWidth', 1.2, 'Alpha', 0.5);

%--- Vertical dashed lines for Va, Vc, Vd ---
xline(Va_kt, 'k--', 'LineWidth', 1.2, 'Alpha', 0.7);
xline(Vc_kt, 'k--', 'LineWidth', 1.2, 'Alpha', 0.7);
xline(Vd_kt, 'k--', 'LineWidth', 1.2, 'Alpha', 0.7);

%--- Annotate key points ---
text(A_V+2,  A_n+0.1,     'A', 'FontSize', 18, 'FontWeight', 'bold');
text(B_V+2,  B_n+0.1,     'B', 'FontSize', 18, 'FontWeight', 'bold');
text(D_V-8,  D_n+0.1,     'D', 'FontSize', 18, 'FontWeight', 'bold');
text(C_V+2,  C_n-0.15,    'C', 'FontSize', 18, 'FontWeight', 'bold');

%--- Speed labels on x-axis ---
xticks_extra = unique(sort([get(gca,'XTick'), Va_kt, Vc_kt, Vd_kt]));
xticks(xticks_extra);

% Label VA, VC, VNE below x axis
ylims = ylim;
text(Va_kt, ylims(1)-0.07*diff(ylims), 'V_A', ...
    'FontSize',18,'HorizontalAlignment','center','FontWeight','bold');
text(Vc_kt, ylims(1)-0.07*diff(ylims), 'V_C', ...
    'FontSize',18,'HorizontalAlignment','center','FontWeight','bold');
text(Vd_kt, ylims(1)-0.07*diff(ylims), 'V_{NE}', ...
    'FontSize',18,'HorizontalAlignment','center','FontWeight','bold');

%--- Region labels ---
text((VS_pos_kt+Va_kt)/2+5, 0.8, {'Normal flight', 'envelope'}, ...
    'FontSize', 18, 'HorizontalAlignment','center', ...
    'Color',[0 0.4 0]);
text((Vc_kt+Vd_kt)/2, n_pos_limit/2, 'Caution region', ...
    'FontSize', 18, 'HorizontalAlignment','center', ...
    'Color',[0.6 0.4 0], 'Rotation', 0);
text(Vd_kt*1.02, n_pos_limit*0.5, {'Structural', 'failure', 'likely'}, ...
    'FontSize', 18, 'HorizontalAlignment','left', ...
    'Color',[0.6 0 0], 'Rotation', 90);
text(Vd_kt*0.5, n_pos_limit+0.6, 'Structural damage likely', ...
    'FontSize', 18, 'HorizontalAlignment','center', ...
    'Color',[0.7 0.3 0]);
text(Vd_kt*0.5, n_neg_limit-0.4, 'Structural damage likely', ...
    'FontSize', 18, 'HorizontalAlignment','center', ...
    'Color',[0.7 0.3 0]);

%--- Level un-accelerated flight annotation ---
text(Va_kt*0.4, 1.12, {'Level', 'unaccelerated', 'flight'}, ...
    'FontSize', 18, 'HorizontalAlignment','center');
text(0.5, 1, '\rightarrow', 'FontSize', 18);

%--- +ve / -ve lift labels ---
text(-18, n_pos_limit*0.5, '+ve lift', 'FontSize', 18, ...
    'Rotation', 90, 'HorizontalAlignment','center');
text(-18, n_neg_limit*0.5, '-ve lift', 'FontSize', 18, ...
    'Rotation', 90, 'HorizontalAlignment','center');

%% ── 5.  Axes formatting ────────────────────────────────────────────────
xlabel('Airspeed (KEAS)', 'FontSize', 18);
ylabel('Load factor,  n', 'FontSize', 18);
title('V-n Diagram', 'FontSize', 18);

xlim([0, Vd_kt * 1.08]);
ylim([n_neg_limit*1.6, n_pos_limit*1.6]);
yticks(n_neg_limit*1.5 : 0.5 : n_pos_limit*1.5);

set(gca, 'FontSize', 18, 'TickDir', 'out', 'Layer', 'top');

%% ── 6.  Print key values to console ───────────────────────────────────
fprintf('\n======= V-n Diagram Parameters (pulled from ADP) =======\n');
fprintf('  MTOM            = %10.0f kg\n',   MTOM);
fprintf('  Wing Area S     = %10.1f m²\n',  S);
fprintf('  Wing Loading    = %10.1f N/m²\n', WS);
fprintf('  Aspect Ratio AR = %10.2f\n',      AR);
fprintf('  Span            = %10.1f m\n',    ADP.Span);
fprintf('  CLmax (clean)   = %10.2f\n',      CLmax_pos);
fprintf('  CLmax (neg)     = %10.2f\n',      CLmax_neg);
fprintf('  n_pos limit     = %10.2f\n',      n_pos_limit);
fprintf('  n_neg limit     = %10.2f\n',      n_neg_limit);
fprintf('  Vs (pos, SL)    = %10.1f kt EAS\n', VS_pos_kt);
fprintf('  Vs (neg, SL)    = %10.1f kt EAS\n', VS_neg_kt);
fprintf('  Va              = %10.1f kt EAS\n', Va_kt);
fprintf('  Vc              = %10.1f kt EAS\n', Vc_kt);
fprintf('  Vd (Vne)        = %10.1f kt EAS\n', Vd_kt);
fprintf('  Cruise Mach     = %10.2f\n',       M_c);
fprintf('  Cruise Altitude = %10.0f m\n',     h_c);
fprintf('=========================================================\n\n');

end  % VnDiagram


%% ── Local ISA atmosphere model ─────────────────────────────────────────
function [rho, a, T, P] = atmos_isa(h)
% Simple ISA atmosphere (troposphere + lower stratosphere).
% h in metres, returns SI units.
    T0 = 288.15; P0 = 101325; rho0 = 1.225;
    g  = 9.80665; R  = 287.05; gamma = 1.4;
    L  = 0.0065;    % lapse rate [K/m]
    h11 = 11000;    % tropopause [m]

    if isscalar(h)
        if h <= h11
            T = T0 - L*h;
            P = P0*(T/T0)^(g/(R*L));
        else
            T = 216.65;
            P = 22632.1 * exp(-g*(h-h11)/(R*T));
        end
        rho = P/(R*T);
        a   = sqrt(gamma*R*T);
    else
        T = zeros(size(h)); P = zeros(size(h));
        trop = h <= h11;
        T(trop)  = T0 - L*h(trop);
        P(trop)  = P0.*(T(trop)/T0).^(g/(R*L));
        T(~trop) = 216.65;
        P(~trop) = 22632.1 .* exp(-g.*(h(~trop)-h11)./(R*216.65));
        rho = P./(R.*T);
        a   = sqrt(gamma*R.*T);
    end
end