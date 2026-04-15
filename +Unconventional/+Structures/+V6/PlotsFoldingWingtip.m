function PlotsFoldingWingtip(loc, G, MB_Al, MB_CF, MB_Ti, MB_GlFRP, S_25g)
% Single clear plot:
% Folding wingtip length vs total wing structural mass for multiple materials.
% Optional S_25g: secondary Y-axis plots |M(y)| at the hinge/fold station vs b_tip
% (from the baseline SMT curve — varies with spanwise station). |M_root| is
% constant for fixed loads and is reported in the legend text only.

if nargin < 5, MB_Ti = []; end
if nargin < 6, MB_GlFRP = []; end
if nargin < 7, S_25g = []; end

s = G.s;
y = G.y;
N = G.N;
dy = G.dy;

% Sweep folding-tip length per side.
b_tips = linspace(0, 0.38 * s, 80);
b_des = s - loc.y_hinge;

mat_names = {'Al 7010-T7451', 'CFRP quasi-iso'};
mat_cols = {[0.00 0.45 0.74], [0.00 0.45 0.74]};
mb_list = {MB_Al, MB_CF};

if ~isempty(MB_Ti)
    mat_names{end+1} = 'Ti-6Al-4V';
    mat_cols{end+1} = [0.49 0.18 0.56];
    mb_list{end+1} = MB_Ti;
end
if ~isempty(MB_GlFRP)
    mat_names{end+1} = 'GFRP quasi-iso';
    mat_cols{end+1} = [0.47 0.67 0.19];
    mb_list{end+1} = MB_GlFRP;
end

n_mat = numel(mb_list);
m_total = zeros(n_mat, numel(b_tips));

for k = 1:numel(b_tips)
    y_h = s - b_tips(k);
    [~, ih] = min(abs(y - y_h));

    % Hardware rib mass uses Al properties (same hardware regardless of spar material).
    Q_rib = abs(MB_Al.m_total_dist(ih) / dy * G.w_wb(ih));
    t_rib = max(1.5 * Q_rib / (G.h_wb(ih) * loc.Al.tau_all), loc.Al.t_min);
    m_rib = loc.Al.rho * 2 * (G.h_wb(ih) * G.w_wb(ih)) * t_rib;

    for m = 1:n_mat
        MB = mb_list{m};
        m_out = 2 * sum(MB.m_total_dist(1:ih));
        m_inn = 2 * sum(MB.m_total_dist(ih:N));
        m_ha = max(loc.f_hinge_mech * m_out, loc.m_hinge_min);
        m_pen = m_ha + loc.f_lock_mech * m_out + loc.f_actuator * m_out + m_rib;
        m_total(m, k) = (m_out + m_inn) * (1 + loc.f_secondary) + m_pen;
    end
end

[~, k_des] = min(abs(b_tips - b_des));

% Bending moment vs fold position: evaluate baseline M(y) at each hinge station (varies along span).
M_hinge_MNm = [];
if ~isempty(S_25g) && isfield(S_25g, 'M')
    M_hinge_MNm = zeros(1, numel(b_tips));
    for k = 1:numel(b_tips)
        y_h = s - b_tips(k);
        [~, ih_k] = min(abs(y - y_h));
        M_hinge_MNm(k) = abs(S_25g.M(ih_k)) / 1e6;
    end
end

% Figure styling (dual yy-axis with blue primary x-axis and orange secondary axis)
C_left  = [0.00 0.45 0.74];   % MATLAB default blue — Al/CFRP and x-axis
C_right = [0.85 0.33 0.10];   % orange — |M| curve and right y-axis

figure('Name', 'Folding Wingtip vs Wing Mass', 'Color', 'w', 'Position', [100 100 1050 620]);
ax = gca;

yyaxis(ax, 'left');
hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
ax.GridAlpha = 0.4;
ax.XColor = 'k';
ax.TickDir = 'out';
ax.FontSize = 11;

for m = 1:n_mat
    plot(ax, b_tips, m_total(m, :) / 1e3, 'Color', mat_cols{m}, 'LineWidth', 2.4, ...
        'DisplayName', mat_names{m});
    plot(ax, b_tips(k_des), m_total(m, k_des) / 1e3, 'o', ...
        'Color', mat_cols{m}, 'MarkerFaceColor', mat_cols{m}, 'HandleVisibility', 'off');
end

ax.YColor = C_left;
hYl = ylabel(ax, 'Total wing structural mass [t]', 'FontSize', 12);
hYl.Color = C_left;

if ~isempty(S_25g) && ~isempty(M_hinge_MNm)
    yyaxis(ax, 'right');
    hold(ax, 'on');
    if isfield(S_25g, 'M_root')
        Mroot_MNm = abs(S_25g.M_root) / 1e6;
    else
        Mroot_MNm = abs(S_25g.M(N)) / 1e6;
    end
    plot(ax, b_tips, M_hinge_MNm, '--', ...
        'Color', C_right, 'LineWidth', 2.0, ...
        'DisplayName', sprintf('|M| at hinge vs b_{tip} (%s)', S_25g.loadcase));
    ax.YColor = C_right;
    hYr = ylabel(ax, '|M| at fold line [MNm]', 'FontSize', 12);
    hYr.Color = C_right;
    yyaxis(ax, 'left');
    % Root BM is fixed for this load model; note for the reader.
    annotation(ax.Parent, 'textbox', [0.14 0.02 0.62 0.06], 'Units', 'normalized', ...
        'String', sprintf('Reference: |M_{root}| = %.2f MNm (constant for fixed %s loads)', ...
        Mroot_MNm, S_25g.loadcase), ...
        'EdgeColor', [0.55 0.55 0.55], 'BackgroundColor', [1 1 1 0.92], ...
        'FontSize', 9, 'FitBoxToText', 'on', 'VerticalAlignment', 'bottom');
end

xlabel(ax, 'Folding tip length b_{tip} [m] (per side)', 'FontSize', 12, 'Color', 'k');
title(ax, 'Wing mass and bending moment at fold vs folding tip length', ...
    'FontWeight', 'bold', 'FontSize', 13);
legend(ax, 'Location', 'northwest', 'Box', 'on', 'FontSize', 10);
xlim(ax, [0, max(b_tips)]);

end
