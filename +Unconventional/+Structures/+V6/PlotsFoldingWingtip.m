function PlotsFoldingWingtip(loc, G, MB_Al, MB_CF, MB_Ti, MB_GlFRP, S_25g_Al, S_25g_CF)
% Single clear plot:
% Folding wingtip length vs one-side wing structural mass (Al; CFRP inputs ignored for plotting).
% Optional S_25g: secondary Y-axis plots total root bending moment varying with
% folding-tip length.

if nargin < 5, MB_Ti = []; end
if nargin < 6, MB_GlFRP = []; end
if nargin < 7, S_25g_Al = []; end
if nargin < 8, S_25g_CF = []; end

s = G.s;
y = G.y;
N = G.N;
dy = G.dy;

% Sweep folding-tip length per side (cap at 10 m for axis range).
b_tip_max = min(0.38 * s, 10);
b_tips = linspace(0, b_tip_max, 80);
mat_names = {'Al 7010-T7451'};
mat_cols = {[0.00 0.45 0.74]};
mb_list = {MB_Al};

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
        m_out = sum(MB.m_total_dist(1:ih));
        m_inn = sum(MB.m_total_dist(ih:N));
        m_ha = max(loc.f_hinge_mech * m_out, loc.m_hinge_min);
        m_pen = m_ha + loc.f_lock_mech * m_out + loc.f_actuator * m_out + m_rib;
        m_total(m, k) = (m_out + m_inn) * (1 + loc.f_secondary) + m_pen;
    end
end

% Root BM model for variable folding-tip length:
% Use span-cubic scaling as a first-order estimate:
%   M_root(b) = M_root_ref * (s_eff / s_ref)^3, where s_eff = y_hinge + b_tip.
Mroot_MNm_Al = [];
if ~isempty(S_25g_Al) && isfield(S_25g_Al, 'M')
    if isfield(S_25g_Al, 'M_root')
        Mroot_ref_Al = abs(S_25g_Al.M_root);
    else
        Mroot_ref_Al = abs(S_25g_Al.M(N));
    end
    s_ref = s;
    s_eff = loc.y_hinge + b_tips;
    Mroot_MNm_Al = (Mroot_ref_Al / 1e6) * (s_eff / s_ref).^3;
end

% Figure styling (dual yy-axis with blue primary x-axis and orange secondary axis)
C_left  = [0.00 0.45 0.74];   % MATLAB default blue - Al mass and x-axis
C_right = [0.85 0.33 0.10];   % orange - |M| curve and right y-axis

figure('Name', 'Folding Wingtip vs Wing Mass & WRBM', 'Color', 'w', 'Position', [100 100 1050 620]);
ax = gca;

yyaxis(ax, 'left');
hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
ax.GridAlpha = 0.4;
ax.XColor = 'k';
ax.TickDir = 'out';
ax.FontSize = 14;

for m = 1:n_mat
    plot(ax, b_tips, m_total(m, :) / 1e3, 'Color', mat_cols{m}, 'LineWidth', 2.4, ...
        'DisplayName', mat_names{m});
end

ax.YColor = C_left;
hYl = ylabel(ax, 'Wing structural mass [t]', 'FontSize', 18);
hYl.Color = C_left;

if ~isempty(Mroot_MNm_Al)
    yyaxis(ax, 'right');
    hold(ax, 'on');
    if isfield(S_25g_Al, 'loadcase'), lcA = S_25g_Al.loadcase; else, lcA = 'case'; end
    plot(ax, b_tips, Mroot_MNm_Al, '-', ...
        'Color', C_right, 'LineWidth', 2.0, ...
        'DisplayName', sprintf('Root bending moment (%s, Al)', lcA));

    ax.YColor = C_right;
    hYr = ylabel(ax, 'Root Bending moment [MNm]', 'FontSize', 18);
    hYr.Color = C_right;
    yyaxis(ax, 'left');
end

xlabel(ax, 'Folding tip length [m]', 'FontSize', 18, 'Color', 'k');
title(ax, 'Wing mass and WRBM vs folding tip length', ...
    'FontWeight', 'bold', 'FontSize', 22, 'Interpreter', 'tex');
legend(ax, 'Location', 'northwest', 'Box', 'on', 'FontSize', 14, 'Interpreter', 'tex');
xlim(ax, [0, b_tip_max]);

end
