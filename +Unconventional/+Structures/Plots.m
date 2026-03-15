function Plots(p, G, L, S, W, D, MB)

y        = fliplr(G.y);
flip_arr = @(x) fliplr(x);
lc       = S.loadcase;

clr1 = [0.00 0.45 0.74];
clr2 = [0.85 0.33 0.10];
clr3 = [0.47 0.67 0.19];
clr4 = [0.49 0.18 0.56];
clrk = [0.00 0.00 0.00];

hinge_x  = p.y_hinge;
engine_x = p.y_engine;

% =========================================================================
%  FIGURE 1 — DISTRIBUTED LOADS
% =========================================================================
figure('Name', sprintf('Distributed Loads — %s', lc), 'Position', [60 60 1200 500]);
tl1 = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl1, sprintf('Distributed Loads: %s  (n_{lim}=%.1f, n_{ult}=%.2f)', ...
    lc, S.n_limit, S.n_ult), 'FontWeight', 'bold');

nexttile(1);
hold on;
plot(y, flip_arr(L.lift_dist)/1e3, '-',  'Color', clr1, 'LineWidth', 2.5, 'DisplayName', 'Aerodynamic lift');
plot(y, flip_arr(L.w_wing)/1e3,    '--', 'Color', clr2, 'LineWidth', 1.8, 'DisplayName', 'Wing inertia relief');
plot(y, flip_arr(L.w_fuel)/1e3,    '--', 'Color', clr3, 'LineWidth', 1.8, 'DisplayName', 'Fuel inertia relief');
plot(y, flip_arr(L.net_dist)/1e3,  '-',  'Color', clrk, 'LineWidth', 2.5, 'DisplayName', 'Net load');
xline(hinge_x,  'k:', 'LineWidth', 1.5, 'Label', 'Fold hinge');
xline(engine_x, 'r:', 'LineWidth', 1.2, 'Label', 'Engine');
xlabel('y [m]  (root to tip)');  ylabel('Load intensity [kN/m]');
title('Spanwise Load Components');
legend('Location', 'northeast', 'FontSize', 8);  grid on;

nexttile(2);
lift_plot   = flip_arr(L.lift_dist)/1e3;
relief_plot = flip_arr(L.w_wing + L.w_fuel)/1e3;
area(y, lift_plot,   'FaceColor', clr1, 'FaceAlpha', 0.25, 'EdgeColor', clr1, 'LineWidth', 1.5, 'DisplayName', 'Lift');
hold on;
area(y, relief_plot, 'FaceColor', clr2, 'FaceAlpha', 0.35, 'EdgeColor', clr2, 'LineWidth', 1.5, 'DisplayName', 'Total relief');
xline(hinge_x, 'k:', 'LineWidth', 1.5, 'Label', 'Fold hinge');
xlabel('y [m]');  ylabel('Load intensity [kN/m]');
relief_pct = trapz(y, relief_plot) / trapz(y, lift_plot) * 100;
title(sprintf('Inertia Relief = %.0f%% of Lift', relief_pct));
legend('Location', 'northeast', 'FontSize', 8);  grid on;

% =========================================================================
%  FIGURE 2 — SMT DIAGRAMS
% =========================================================================
figure('Name', sprintf('SMT — %s', lc), 'Position', [60 60 1300 820]);
tl2 = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl2, sprintf('Shear / Bending / Torque  —  %s', lc), 'FontWeight', 'bold');

nexttile(1);
plot(y, flip_arr(S.Q)/1e6, '-', 'Color', clr1, 'LineWidth', 2.5);
xline(hinge_x,  'k:', 'LineWidth', 1.5, 'Label', 'Fold hinge');
xline(engine_x, 'r:', 'LineWidth', 1.2, 'Label', 'Engine');
xlabel('y [m]');  ylabel('Q [MN]');  title('Shear Force Q(y)');  grid on;
text(0.02, 0.85, sprintf('Root Q = %.2f MN', abs(S.Q_root)/1e6), ...
    'Units', 'norm', 'FontSize', 9, 'Color', clr1);

nexttile(2);
plot(y, flip_arr(S.Q)/1e6, '-', 'Color', clr1, 'LineWidth', 2);
xlim([0 p.y_hinge*1.5]);
xline(hinge_x, 'k:', 'LineWidth', 1.5, 'Label', 'Fold hinge');
xlabel('y [m]  (inboard detail)');  ylabel('Q [MN]');
title('Shear Force — Inboard Detail');  grid on;

nexttile(3);
plot(y, flip_arr(S.M)/1e6, '-', 'Color', clr2, 'LineWidth', 2.5);
xline(hinge_x,  'k:', 'LineWidth', 1.5, 'Label', 'Fold hinge');
xline(engine_x, 'r:', 'LineWidth', 1.2, 'Label', 'Engine');
xlabel('y [m]');  ylabel('M [MNm]');  title('Bending Moment M(y)');  grid on;
text(0.02, 0.85, sprintf('Root M = %.2f MNm', abs(S.M_root)/1e6), ...
    'Units', 'norm', 'FontSize', 9, 'Color', clr2);
text(0.02, 0.70, sprintf('Hinge M = %.2f MNm  (%.0f%% of root)', ...
    abs(S.M_hinge)/1e6, abs(S.M_hinge/S.M_root)*100), ...
    'Units', 'norm', 'FontSize', 9, 'Color', clr2);

nexttile(4);
hold on;
p_nf          = p;
p_nf.M_fuel   = 0;
p_nf.Mf_fuel  = 0;
G_nf = Unconventional.Structures.WingGeometry(p_nf);
L_nf = Unconventional.Structures.LoadDistribution(p_nf, G_nf, lc);
S_nf = Unconventional.Structures.SMT(p_nf, G_nf, L_nf);
plot(y, flip_arr(S_nf.M)/1e6, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.5, 'DisplayName', 'No fuel relief');
plot(y, flip_arr(S.M)/1e6,    '-',  'Color', clr2,          'LineWidth', 2.5, 'DisplayName', 'With fuel relief');
xline(hinge_x, 'k:', 'LineWidth', 1.5, 'Label', 'Fold hinge');
xlabel('y [m]');  ylabel('M [MNm]');
fuel_pct = (abs(S_nf.M_root) - abs(S.M_root)) / abs(S_nf.M_root) * 100;
title(sprintf('Fuel Relief: −%.0f%% Root BM', fuel_pct));
legend('Location', 'northeast', 'FontSize', 8);  grid on;

nexttile(5);
plot(y, flip_arr(S.T)/1e6, '-', 'Color', clr4, 'LineWidth', 2.5);
xline(hinge_x, 'k:', 'LineWidth', 1.5, 'Label', 'Fold hinge');
xlabel('y [m]');  ylabel('T [MNm]');  title('Torque T(y)');  grid on;
text(0.02, 0.85, sprintf('Root T = %.2f MNm', abs(S.T_root)/1e6), ...
    'Units', 'norm', 'FontSize', 9, 'Color', clr4);

nexttile(6);
axis off;
txt = {
    sprintf('\\bfLOAD CASE: %s\\rm', lc),
    sprintf('n_{limit} = %.1f  |  n_{ult} = %.2f', S.n_limit, S.n_ult),
    ' ',
    sprintf('\\bfROOT\\rm'),
    sprintf('Q = %.2f MN',   abs(S.Q_root)/1e6),
    sprintf('M = %.2f MNm',  abs(S.M_root)/1e6),
    sprintf('T = %.2f MNm',  abs(S.T_root)/1e6),
    ' ',
    sprintf('\\bfHINGE  y = %.1f m\\rm', p.y_hinge),
    sprintf('M = %.2f MNm  (%.0f%% root)', abs(S.M_hinge)/1e6, abs(S.M_hinge/S.M_root)*100),
    sprintf('M_Snorri = %.2f MNm', S.M_hinge_snorri/1e6),
};
text(0.05, 0.97, txt, 'Units', 'norm', 'VerticalAlignment', 'top', ...
    'FontSize', 10, 'Interpreter', 'tex');

% =========================================================================
%  FIGURE 3 — WINGBOX PROPERTIES
% =========================================================================
figure('Name', sprintf('Wingbox Properties — %s', lc), 'Position', [80 80 1300 650]);
tl3 = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl3, sprintf('Wingbox Properties  —  %s  (%s)', lc, W.mat_name), 'FontWeight', 'bold');

nexttile(1);
plot(y, flip_arr(W.t_skin_mm), '-', 'Color', clr1, 'LineWidth', 2);
xline(hinge_x, 'k:', 'LineWidth', 1.5, 'Label', 'Fold hinge');
xlabel('y [m]');  ylabel('t_{skin} [mm]');
title('Skin Thickness (torsion + buckling)');  grid on;

nexttile(2);
plot(y, flip_arr(W.A_cap_cm2), '-', 'Color', clr2, 'LineWidth', 2);
xline(hinge_x, 'k:', 'LineWidth', 1.5, 'Label', 'Fold hinge');
xlabel('y [m]');  ylabel('A_{cap} [cm^2]');
title('Spar Cap Area (bending)');  grid on;

nexttile(3);
plot(y, flip_arr(W.t_web_mm), '-', 'Color', clr4, 'LineWidth', 2);
xline(hinge_x, 'k:', 'LineWidth', 1.5, 'Label', 'Fold hinge');
xlabel('y [m]');  ylabel('t_{web} [mm]');
title('Spar Web Thickness (Jourawski shear)');  grid on;

nexttile(4);
semilogy(y, flip_arr(D.EI), '-', 'Color', clr1, 'LineWidth', 2);
xline(hinge_x, 'k:', 'LineWidth', 1.5, 'Label', 'Fold hinge');
xlabel('y [m]');  ylabel('EI [Nm^2]');
title('Bending Stiffness EI(y)');  grid on;

nexttile(5);
semilogy(y, flip_arr(D.GJ), '-', 'Color', clr2, 'LineWidth', 2);
xline(hinge_x, 'k:', 'LineWidth', 1.5, 'Label', 'Fold hinge');
xlabel('y [m]');  ylabel('GJ [Nm^2]');
title('Torsional Stiffness GJ(y)');  grid on;

nexttile(6);
bar_mass = [MB.m_skin, MB.m_caps, MB.m_webs, MB.m_secondary, MB.m_hinge] / 1e3;
bar_lbls = {'Skin', 'Caps', 'Webs', 'Secondary', 'Hinge'};
clrs_bar = [clr1; clr2; clr3; [0.5 0.5 0.5]; [0.8 0.6 0.0]];
bh = bar(bar_mass, 'FaceColor', 'flat');
bh.CData = clrs_bar;
set(gca, 'XTickLabel', bar_lbls, 'XTickLabelRotation', 20);
ylabel('Mass [t]');
title(sprintf('Mass Breakdown — Total %.0f kg', MB.m_total));  grid on;
for k = 1:5
    text(k, bar_mass(k) + 0.05, sprintf('%.1ft', bar_mass(k)), ...
        'HorizontalAlignment', 'center', 'FontSize', 8);
end

end
