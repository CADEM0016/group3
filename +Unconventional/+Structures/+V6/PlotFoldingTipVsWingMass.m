function PlotFoldingTipVsWingMass(G, loc, MB_Al, MB_CF, MB_Ti, MB_GlFRP)
% PlotFoldingTipVsWingMass  –  Folding wingtip length vs wing structural mass
%   with multiple material compositions on a dual-Y axis.
%
%   INPUTS
%     G        – WingGeometry output struct
%     loc      – AircraftParams output struct
%     MB_Al    – MassBuildup output for Aluminium 7010-T7451   (required)
%     MB_CF    – MassBuildup output for CFRP quasi-iso          (required)
%     MB_Ti    – MassBuildup output for Titanium Ti-6Al-4V      (optional, [] to skip)
%     MB_GlFRP – MassBuildup output for Glass-fibre / GFRP      (optional, [] to skip)
%
%   OUTPUTS  (both axes)
%     Left  Y  –  Total wing structural mass [tonnes]
%     Right Y  –  Fold mechanism mass penalty [kg]
%
%   USAGE (called from StructuresAll_V6.m after step 7)
%     Unconventional.Structures.V6.PlotFoldingTipVsWingMass(G, loc, MB_25g, MB_25g_CF);
%
%   or with optional materials:
%     Unconventional.Structures.V6.PlotFoldingTipVsWingMass(G, loc, MB_25g, MB_25g_CF, [], []);

% ── Argument handling ────────────────────────────────────────────────────────
if nargin < 5,  MB_Ti    = [];  end
if nargin < 6,  MB_GlFRP = [];  end

% ── Geometry / parameter unpacking ───────────────────────────────────────────
s            = G.s;
y            = G.y;
y_hinge      = G.y_hinge;
h_wb         = G.h_wb;
w_wb         = G.w_wb;
dy           = G.dy;
N            = G.N;

Span_taxi    = loc.Span_taxi;
f_sec        = loc.f_secondary;
f_hinge      = loc.f_hinge_mech;
f_lock       = loc.f_lock_mech;
f_act        = loc.f_actuator;
m_hinge_min  = loc.m_hinge_min;
Al           = loc.Al;

% ── Sweep range: 0 … 38 % of semi-span  (Code E always within range) ────────
b_tips = linspace(0, 0.38*s, 80);   % outer panel half-span [m] per side
b_codeE = s - Span_taxi/2;          % Code-E limit on this axis

% ── Colour / style palette ────────────────────────────────────────────────────
C = struct(...
    'Al',   [0.086 0.369 0.631], ...   % steel blue
    'CF',   [0.847 0.329 0.102], ...   % burnt orange
    'Ti',   [0.420 0.180 0.560], ...   % purple
    'GlFRP',[0.188 0.627 0.224], ...   % green
    'pen',  [0.780 0.110 0.110], ...   % red  (penalty)
    'grey', [0.55  0.55  0.55 ]);

% ── Assemble list of active materials ────────────────────────────────────────
matList = {'Al','CF'};
mbList  = {MB_Al, MB_CF};
lblList = {'Al 7010-T7451','CFRP quasi-iso'};
clrList = {C.Al, C.CF};

if ~isempty(MB_Ti)
    matList{end+1} = 'Ti';
    mbList{end+1}  = MB_Ti;
    lblList{end+1} = 'Ti-6Al-4V';
    clrList{end+1} = C.Ti;
end
if ~isempty(MB_GlFRP)
    matList{end+1} = 'GlFRP';
    mbList{end+1}  = MB_GlFRP;
    lblList{end+1} = 'GFRP quasi-iso';
    clrList{end+1} = C.GlFRP;
end
nMat = numel(matList);

% ── Pre-allocate sweep arrays ─────────────────────────────────────────────────
m_wing  = zeros(nMat, numel(b_tips));  % total wing mass [kg] per material
m_pen   = zeros(nMat, numel(b_tips));  % fold penalty [kg]    per material

% Penalty sub-components (Al governs hinge rib – only stored once)
m_ha  = zeros(1, numel(b_tips));   % hinge assembly
m_lk  = zeros(1, numel(b_tips));   % lock mechanism
m_ac  = zeros(1, numel(b_tips));   % fold actuator
m_rb  = zeros(1, numel(b_tips));   % hinge rib (Al alloy regardless of spar mat.)

% ── Main sweep ───────────────────────────────────────────────────────────────
for k = 1:numel(b_tips)
    y_h   = s - b_tips(k);                      % hinge y from CL
    [~, ih_k] = min(abs(y - y_h));              % nearest station index

    % Hinge rib mass — always Al alloy (pin / rib hardware)
    Q_rib = abs(MB_Al.m_total_dist(ih_k) / dy * w_wb(ih_k));  % simplified shear
    t_rib = max(1.5 * Q_rib / (h_wb(ih_k) * Al.tau_all), Al.t_min);
    m_rb(k) = Al.rho * 2 * (h_wb(ih_k) * w_wb(ih_k)) * t_rib;

    for m = 1:nMat
        MB = mbList{m};

        % Outer panel primary mass (stations 1..ih_k = tip side)
        m_out = 2 * sum(MB.m_total_dist(1:ih_k));

        % Inner panel primary mass  (stations ih_k..N = root side)
        m_inn = 2 * sum(MB.m_total_dist(ih_k:N));

        % Hinge assembly (fraction of outer primary; min floor)
        ha = max(f_hinge * m_out, m_hinge_min);

        % Fold penalty for this material
        pen = ha + f_lock * m_out + f_act * m_out + m_rb(k);

        % Total wing mass  (primary + secondary allowance + fold penalty)
        m_wing(m,k) = (m_out + m_inn) * (1 + f_sec) + pen;
        m_pen(m,k)  = pen;

        % Store Al sub-components for stacked area (m==1 → Al)
        if m == 1
            m_ha(k) = ha;
            m_lk(k) = f_lock * m_out;
            m_ac(k) = f_act  * m_out;
        end
    end
end

% ── Design-point index (actual hinge position) ────────────────────────────────
[~, k_des] = min(abs(b_tips - (s - y_hinge)));

% ── Fixed-wing reference lines (b_tip = 0 → no fold) ────────────────────────
m_fixed = zeros(1, nMat);
for m = 1:nMat
    m_fixed(m) = mbList{m}.m_total;   % from MassBuildup (no fold penalty)
end

% ═════════════════════════════════════════════════════════════════════════════
%   FIGURE 1 – Wing mass vs folding tip length  (dual Y-axis)
% ═════════════════════════════════════════════════════════════════════════════
fig1 = figure('Name','Folding Tip vs Wing Mass – Multi-Material', ...
              'Position',[80 80 1150 640]);

% Left axis – total wing mass [tonnes]
ax1 = axes(fig1);
hold(ax1,'on');  grid(ax1,'on');  box(ax1,'on');
ax1.FontSize = 11;
ax1.YColor   = [0.15 0.15 0.15];

hLeft = gobjects(nMat,1);
for m = 1:nMat
    hLeft(m) = plot(ax1, b_tips, m_wing(m,:)/1e3, '-', ...
        'Color', clrList{m}, 'LineWidth', 2.5, ...
        'DisplayName', lblList{m});
end

% Fixed-wing reference dashes on left axis
for m = 1:nMat
    yline(ax1, m_fixed(m)/1e3, '--', ...
        'Color', clrList{m}, 'LineWidth', 1.0, 'Alpha', 0.55, ...
        'Label', sprintf('%s fixed-wing', lblList{m}), ...
        'LabelHorizontalAlignment', 'left', ...
        'FontSize', 8, ...
        'HandleVisibility', 'off');
end

% Design-point markers (left axis)
for m = 1:nMat
    plot(ax1, b_tips(k_des), m_wing(m,k_des)/1e3, 'o', ...
        'Color', clrList{m}, 'MarkerFaceColor', clrList{m}, ...
        'MarkerSize', 9, 'HandleVisibility', 'off');
end

ylabel(ax1, 'Total wing structural mass  [t]', 'FontSize', 12);

% Right axis – fold mechanism penalty [kg]
ax2 = axes(fig1, 'Position', ax1.Position);
ax2.YAxisLocation = 'right';
ax2.Color         = 'none';
ax2.FontSize      = 11;
ax2.YColor        = C.pen;
hold(ax2,'on');

hRight = gobjects(nMat,1);
for m = 1:nMat
    hRight(m) = plot(ax2, b_tips, m_pen(m,:), ':', ...
        'Color', clrList{m}, 'LineWidth', 1.8, ...
        'DisplayName', sprintf('%s penalty', lblList{m}));
end

ylabel(ax2, 'Fold mechanism mass penalty  [kg]', 'FontSize', 12, ...
       'Color', C.pen);

% Code-E vertical line (drawn on ax1)
xl = xline(ax1, b_codeE, '-', ...
    'Color', [0.55 0.0 0.0], 'LineWidth', 2.0, ...
    'Label', 'Code E limit', ...
    'LabelHorizontalAlignment', 'right', ...
    'LabelVerticalAlignment',   'top', ...
    'FontSize', 9, ...
    'HandleVisibility', 'off');

% Shade the infeasible region (right of Code E)
x_fill = [b_codeE, max(b_tips), max(b_tips), b_codeE];
yl1    = ylim(ax1);
y_fill = [yl1(1), yl1(1), yl1(2), yl1(2)];
fill(ax1, x_fill, y_fill, [0.85 0.20 0.20], ...
    'FaceAlpha', 0.07, 'EdgeColor', 'none', ...
    'HandleVisibility', 'off');

% Link X axes so zoom/pan stays coupled
linkaxes([ax1, ax2], 'x');
xlim(ax1, [0, max(b_tips)]);
xlim(ax2, [0, max(b_tips)]);

xlabel(ax1, 'Folding tip length  b_{tip}  [m]  (per side)', 'FontSize', 12);

% Title
title(ax1, ...
    {'Folding Wingtip Length vs Wing Structural Mass', ...
     sprintf('Multiple materials  |  2.5g sizing  |  Span %.0f m  |  Code-E taxi span ≤ %.0f m', ...
     G.Span, Span_taxi)}, ...
    'FontWeight', 'bold', 'FontSize', 12);

% Combined legend – left solid lines + right dotted lines
legendHandles = [hLeft; hRight];
legend(ax1, legendHandles, 'Location', 'northwest', 'FontSize', 9, ...
    'NumColumns', 2, 'Box', 'on');

% Annotation box at design point
des_txt = sprintf(' Design point\n b_{tip} = %.2f m\n', b_tips(k_des));
for m = 1:nMat
    des_txt = sprintf('%s %s: %.0f t\n', des_txt, lblList{m}, m_wing(m,k_des)/1e3);
end
annotation(fig1, 'textbox', [0.62 0.14 0.25 0.20], ...
    'String', strtrim(des_txt), ...
    'FitBoxToText', 'on', ...
    'BackgroundColor', [1 1 1], ...
    'EdgeColor', [0.4 0.4 0.4], ...
    'FontSize', 8.5, 'LineWidth', 0.8);

% ═════════════════════════════════════════════════════════════════════════════
%   FIGURE 2 – Mass penalty breakdown + material comparison (dual Y-axis)
% ═════════════════════════════════════════════════════════════════════════════
fig2 = figure('Name','Fold Penalty & Material Mass Breakdown', ...
              'Position',[130 80 1150 640]);

ax3 = axes(fig2);
hold(ax3, 'on');  grid(ax3, 'on');  box(ax3, 'on');
ax3.FontSize = 11;

% Stacked area for Al penalty sub-components (always Al hardware)
a1 = area(ax3, b_tips, m_ha/1e3, ...
    'FaceColor', C.Al,              'FaceAlpha', 0.65, 'EdgeColor', 'none', ...
    'DisplayName', 'Hinge assembly (Al)');
a2 = area(ax3, b_tips, (m_ha+m_lk)/1e3, ...
    'FaceColor', [0.45 0.60 0.80],  'FaceAlpha', 0.65, 'EdgeColor', 'none', ...
    'DisplayName', 'Lock mechanism');
a3 = area(ax3, b_tips, (m_ha+m_lk+m_ac)/1e3, ...
    'FaceColor', [0.65 0.78 0.92],  'FaceAlpha', 0.65, 'EdgeColor', 'none', ...
    'DisplayName', 'Fold actuator');
a4 = area(ax3, b_tips, (m_ha+m_lk+m_ac+m_rb)/1e3, ...
    'FaceColor', C.grey,            'FaceAlpha', 0.45, 'EdgeColor', 'none', ...
    'DisplayName', 'Hinge rib');

% Total penalty line per material
hPen = gobjects(nMat,1);
lineStyles = {'-','--','-.',':'};
for m = 1:nMat
    hPen(m) = plot(ax3, b_tips, m_pen(m,:)/1e3, ...
        lineStyles{min(m,4)}, ...
        'Color', clrList{m}, 'LineWidth', 2.2, ...
        'DisplayName', sprintf('Total penalty – %s', lblList{m}));
end

% Code-E line
xline(ax3, b_codeE, '-', 'Color', [0.55 0.0 0.0], 'LineWidth', 2.0, ...
    'Label', 'Code E', 'LabelHorizontalAlignment', 'right', ...
    'FontSize', 9, 'HandleVisibility', 'off');
fill(ax3, x_fill, [ylim(ax3) fliplr(ylim(ax3))], [0.85 0.20 0.20], ...
    'FaceAlpha', 0.06, 'EdgeColor', 'none', 'HandleVisibility', 'off');

% Design-point markers
for m = 1:nMat
    plot(ax3, b_tips(k_des), m_pen(m,k_des)/1e3, 'o', ...
        'Color', clrList{m}, 'MarkerFaceColor', clrList{m}, ...
        'MarkerSize', 9, 'HandleVisibility', 'off');
end

ylabel(ax3, 'Fold mechanism mass penalty  [t]', 'FontSize', 12);
xlabel(ax3, 'Folding tip length  b_{tip}  [m]  (per side)', 'FontSize', 12);

% Right axis – penalty as % of MTOM
MTOM_ref = MB_Al.m_total / MB_Al.m_frac_MTOM;   % derive MTOM from any MB

ax4 = axes(fig2, 'Position', ax3.Position);
ax4.YAxisLocation = 'right';
ax4.Color         = 'none';
ax4.FontSize      = 11;
ax4.YColor        = [0.35 0.35 0.35];
hold(ax4, 'on');

for m = 1:nMat
    plot(ax4, b_tips, m_pen(m,:)/MTOM_ref*100, ...
        lineStyles{min(m,4)}, ...
        'Color', clrList{m}, 'LineWidth', 1.3, ...
        'HandleVisibility', 'off');
end

ylabel(ax4, 'Fold penalty  [% MTOM]', 'FontSize', 12, ...
       'Color', [0.35 0.35 0.35]);
linkaxes([ax3, ax4], 'x');
xlim(ax3, [0, max(b_tips)]);
xlim(ax4, [0, max(b_tips)]);

title(ax3, ...
    {'Fold Mechanism Mass Penalty – Material Comparison', ...
     sprintf('Stacked area = Al hardware sub-components  |  Lines = total penalty per spar material  |  b_{tip,des} = %.2f m', ...
     b_tips(k_des))}, ...
    'FontWeight', 'bold', 'FontSize', 12);

legend(ax3, [a1 a2 a3 a4 hPen'], 'Location', 'northwest', 'FontSize', 9);

% ═════════════════════════════════════════════════════════════════════════════
%   FIGURE 3 – Mass fraction comparison bar at the design point
% ═════════════════════════════════════════════════════════════════════════════
fig3 = figure('Name','Design-Point Mass Fraction – Material Comparison', ...
              'Position',[180 80 860 520]);

ax5 = axes(fig3);
hold(ax5, 'on');  grid(ax5, 'on');  box(ax5, 'on');
ax5.FontSize = 11;

% Build bar data [primary_inner, primary_outer, secondary, penalty]
barData = zeros(nMat, 4);
for m = 1:nMat
    MB   = mbList{m};
    ih_d = G.i_hinge;   % design-point hinge index from G

    m_out_d = 2 * sum(MB.m_total_dist(1:ih_d));
    m_inn_d = 2 * sum(MB.m_total_dist(ih_d:N));
    m_sec_d = f_sec * (m_out_d + m_inn_d);
    ha_d    = max(f_hinge * m_out_d, m_hinge_min);
    pen_d   = ha_d + f_lock * m_out_d + f_act * m_out_d + m_rb(k_des);

    barData(m,:) = [m_inn_d, m_out_d, m_sec_d, pen_d] / 1e3;
end

barColors = [0.25 0.55 0.85; ...   % inner primary  (light blue)
             0.09 0.34 0.62; ...   % outer primary  (dark blue)
             0.65 0.75 0.65; ...   % secondary      (sage)
             0.78 0.11 0.11];      % penalty        (red)

b = bar(ax5, barData, 'stacked');
for j = 1:4
    b(j).FaceColor = barColors(j,:);
    b(j).FaceAlpha = 0.82;
end

% Right Y – % MTOM
ax6 = axes(fig3, 'Position', ax5.Position);
ax6.YAxisLocation = 'right';
ax6.Color         = 'none';
ax6.FontSize      = 11;
ax6.YColor        = [0.35 0.35 0.35];
ax6.XTick         = [];
hold(ax6, 'on');

maxMass = max(sum(barData,2));
ylim(ax5, [0, maxMass*1.12]);
ylim(ax6, [0, maxMass*1.12 / MTOM_ref * 100]);

% MTOM reference line
yline(ax5, MTOM_ref/1e3 * 0.12, ':', 'Color', [0.55 0.0 0.0], ...
    'LineWidth', 1.5, 'Label', 'A380 12% MTOM ref', ...
    'LabelHorizontalAlignment', 'right', 'FontSize', 8, ...
    'HandleVisibility', 'off');

linkaxes([ax5, ax6], 'y');
ylim(ax5, [0, maxMass*1.12]);

ylabel(ax5, 'Wing structural mass  [t]', 'FontSize', 12);
ylabel(ax6, 'Mass as % MTOM', 'FontSize', 12, 'Color', [0.35 0.35 0.35]);
xlabel(ax5, 'Material', 'FontSize', 12);
ax5.XTickLabel = lblList;
ax5.XTick      = 1:nMat;

legend(ax5, b, {'Inner primary','Outer primary','Secondary','Fold penalty'}, ...
    'Location', 'northwest', 'FontSize', 10);

title(ax5, ...
    {'Design-Point Wing Mass Breakdown by Material', ...
     sprintf('b_{tip} = %.2f m  |  y_{hinge} = %.1f m  |  2.5g  |  Span = %.0f m', ...
     b_tips(k_des), y_hinge, G.Span)}, ...
    'FontWeight', 'bold', 'FontSize', 12);

% ── Console summary ──────────────────────────────────────────────────────────
fprintf('\n=== PlotFoldingTipVsWingMass – Design Point Summary ===\n');
fprintf('  Folding tip b_tip = %.2f m per side  |  y_hinge = %.1f m\n', ...
    b_tips(k_des), y_hinge);
fprintf('  Code-E limit:  b_tip ≤ %.2f m  ', b_codeE);
if b_codeE >= b_tips(k_des)
    fprintf('[PASS  margin %.2f m]\n', b_codeE - b_tips(k_des));
else
    fprintf('[FAIL  over   %.2f m]\n', b_tips(k_des) - b_codeE);
end
fprintf('%-26s  %8s  %8s  %8s\n', '', 'Wing [t]','Pen [kg]','% MTOM');
fprintf('%s\n', repmat('-',1,56));
for m = 1:nMat
    fprintf('  %-24s  %8.2f  %8.0f  %7.2f%%\n', ...
        lblList{m}, ...
        m_wing(m,k_des)/1e3, ...
        m_pen(m,k_des), ...
        m_wing(m,k_des)/MTOM_ref*100);
end
fprintf('%s\n', repmat('=',1,56));

end
