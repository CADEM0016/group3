function Plots(adp, tlar, loc, G, L, S, W, D, MB)
% Three figures: distributed loads, SMT diagrams, wingbox properties.

y        = fliplr(G.y);
flip_arr = @(x) fliplr(x);
lc       = S.loadcase;

blue   = [0.00 0.45 0.74];
orange = [0.85 0.33 0.10];
green  = [0.47 0.67 0.19];
purple = [0.49 0.18 0.56];
black  = [0.00 0.00 0.00];

% =========================================================================
%  FIGURE 1 — Distributed loads
% =========================================================================
figure('Name', sprintf('Loads — %s', lc), 'Position', [60 60 1200 500]);
tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf('Distributed Loads: %s  (n_{lim}=%.1f, n_{ult}=%.2f)', ...
    lc, S.n_limit, S.n_ult), 'FontWeight', 'bold');

nexttile(1);  hold on;
plot(y, flip_arr(L.lift_dist)/1e3, '-',  'Color', blue,   'LineWidth', 2.5, 'DisplayName', 'Lift');
plot(y, flip_arr(L.w_wing)/1e3,    '--', 'Color', orange, 'LineWidth', 1.8, 'DisplayName', 'Wing relief');
plot(y, flip_arr(L.w_fuel)/1e3,    '--', 'Color', green,  'LineWidth', 1.8, 'DisplayName', 'Fuel relief');
plot(y, flip_arr(L.net_dist)/1e3,  '-',  'Color', black,  'LineWidth', 2.5, 'DisplayName', 'Net load');
xline(loc.y_hinge, 'k:', 'LineWidth', 1.5, 'Label', 'Fold hinge');
xline(G.y_engine,  'r:', 'LineWidth', 1.2, 'Label', 'Engine');
xlabel('y [m]');  ylabel('Load [kN/m]');  title('Load components');
legend('Location', 'northeast', 'FontSize', 8);  grid on;

nexttile(2);
lift_plot    = flip_arr(L.lift_dist)/1e3;
total_relief = L.w_wing + L.w_fuel;
relief_plot  = flip_arr(total_relief)/1e3;
area(y, lift_plot,   'FaceColor', blue,   'FaceAlpha', 0.25, 'EdgeColor', blue,   'LineWidth', 1.5, 'DisplayName', 'Lift');
hold on;
area(y, relief_plot, 'FaceColor', orange, 'FaceAlpha', 0.35, 'EdgeColor', orange, 'LineWidth', 1.5, 'DisplayName', 'Total relief');
xline(loc.y_hinge, 'k:', 'LineWidth', 1.5, 'Label', 'Fold hinge');
xlabel('y [m]');  ylabel('Load [kN/m]');
relief_pct = trapz(y, relief_plot) / trapz(y, lift_plot) * 100;
title(sprintf('Inertia relief = %.0f%% of lift', relief_pct));
legend('Location', 'northeast', 'FontSize', 8);  grid on;

% =========================================================================
%  FIGURE 2 — SMT diagrams
% =========================================================================
figure('Name', sprintf('SMT — %s', lc), 'Position', [60 60 1300 820]);
tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf('Shear / Bending / Torque — %s', lc), 'FontWeight', 'bold');

nexttile(1);
plot(y, flip_arr(S.Q)/1e6, '-', 'Color', blue, 'LineWidth', 2.5);
xline(loc.y_hinge, 'k:', 'LineWidth', 1.5, 'Label', 'Fold hinge');
xline(G.y_engine,  'r:', 'LineWidth', 1.2, 'Label', 'Engine');
xlabel('y [m]');  ylabel('Q [MN]');  title('Shear force Q(y)');  grid on;
text(0.02, 0.85, sprintf('Root Q = %.2f MN', abs(S.Q_root)/1e6), 'Units', 'norm', 'FontSize', 9, 'Color', blue);

nexttile(2);
plot(y, flip_arr(S.Q)/1e6, '-', 'Color', blue, 'LineWidth', 2);
xlim([0, loc.y_hinge * 1.5]);
xline(loc.y_hinge, 'k:', 'LineWidth', 1.5, 'Label', 'Fold hinge');
xlabel('y [m]  (inboard)');  ylabel('Q [MN]');  title('Shear — inboard detail');  grid on;

nexttile(3);
plot(y, flip_arr(S.M)/1e6, '-', 'Color', orange, 'LineWidth', 2.5);
xline(loc.y_hinge, 'k:', 'LineWidth', 1.5, 'Label', 'Fold hinge');
xline(G.y_engine,  'r:', 'LineWidth', 1.2, 'Label', 'Engine');
xlabel('y [m]');  ylabel('M [MNm]');  title('Bending moment M(y)');  grid on;
text(0.02, 0.85, sprintf('Root M = %.2f MNm', abs(S.M_root)/1e6), 'Units', 'norm', 'FontSize', 9, 'Color', orange);
text(0.02, 0.70, sprintf('Hinge M = %.2f MNm  (%.0f%% of root)', abs(S.M_hinge)/1e6, abs(S.M_hinge/S.M_root)*100), ...
    'Units', 'norm', 'FontSize', 9, 'Color', orange);

% Fuel relief demo — re-run chain with a zero-fuel copy of adp
nexttile(4);  hold on;
adp_nf         = adp;
adp_nf.Mf_Fuel = 0;
adp_nf.Mf_res  = 0;
G_nf = Unconventional.Structures.WingGeometry(adp_nf, tlar, loc);
L_nf = Unconventional.Structures.LoadDistribution(adp_nf, tlar, loc, G_nf, lc);
S_nf = Unconventional.Structures.SMT(loc, G_nf, L_nf);
plot(y, flip_arr(S_nf.M)/1e6, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.5, 'DisplayName', 'No fuel relief');
plot(y, flip_arr(S.M)/1e6,    '-',  'Color', orange,         'LineWidth', 2.5, 'DisplayName', 'With fuel relief');
xline(loc.y_hinge, 'k:', 'LineWidth', 1.5, 'Label', 'Fold hinge');
xlabel('y [m]');  ylabel('M [MNm]');
title(sprintf('Fuel relief: -%.0f%% root BM', (abs(S_nf.M_root)-abs(S.M_root))/abs(S_nf.M_root)*100));
legend('Location', 'northeast', 'FontSize', 8);  grid on;

nexttile(5);
plot(y, flip_arr(S.T)/1e6, '-', 'Color', purple, 'LineWidth', 2.5);
xline(loc.y_hinge, 'k:', 'LineWidth', 1.5, 'Label', 'Fold hinge');
xlabel('y [m]');  ylabel('T [MNm]');  title('Torque T(y)');  grid on;
text(0.02, 0.85, sprintf('Root T = %.2f MNm', abs(S.T_root)/1e6), 'Units', 'norm', 'FontSize', 9, 'Color', purple);

nexttile(6);  axis off;
text(0.05, 0.97, {
    sprintf('\bfLOAD CASE: %s\rm', lc),
    sprintf('n_{limit} = %.1f  |  n_{ult} = %.2f', S.n_limit, S.n_ult),
    ' ',
    sprintf('\bfROOT\rm'),
    sprintf('Q = %.2f MN',  abs(S.Q_root)/1e6),
    sprintf('M = %.2f MNm', abs(S.M_root)/1e6),
    sprintf('T = %.2f MNm', abs(S.T_root)/1e6),
    ' ',
    sprintf('\bfHINGE  y = %.1f m\rm', loc.y_hinge),
    sprintf('M = %.2f MNm  (%.0f%% of root)', abs(S.M_hinge)/1e6, abs(S.M_hinge/S.M_root)*100),
    sprintf('M Snorri = %.2f MNm', S.M_hinge_snorri/1e6),
}, 'Units', 'norm', 'VerticalAlignment', 'top', 'FontSize', 10, 'Interpreter', 'tex');

% =========================================================================
%  FIGURE 3 — Wingbox properties
% =========================================================================
figure('Name', sprintf('Wingbox — %s', lc), 'Position', [80 80 1300 650]);
tl = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf('Wingbox Properties — %s  (%s)', lc, W.mat_name), 'FontWeight', 'bold');

nexttile(1);
plot(y, flip_arr(W.t_skin_mm), '-', 'Color', blue, 'LineWidth', 2);
xline(loc.y_hinge, 'k:', 'LineWidth', 1.5, 'Label', 'Fold hinge');
xlabel('y [m]');  ylabel('t_{skin} [mm]');  title('Skin thickness');  grid on;

nexttile(2);
plot(y, flip_arr(W.A_cap_cm2), '-', 'Color', orange, 'LineWidth', 2);
xline(loc.y_hinge, 'k:', 'LineWidth', 1.5, 'Label', 'Fold hinge');
xlabel('y [m]');  ylabel('A_{cap} [cm^2]');  title('Spar cap area');  grid on;

nexttile(3);
plot(y, flip_arr(W.t_web_mm), '-', 'Color', purple, 'LineWidth', 2);
xline(loc.y_hinge, 'k:', 'LineWidth', 1.5, 'Label', 'Fold hinge');
xlabel('y [m]');  ylabel('t_{web} [mm]');  title('Web thickness');  grid on;

nexttile(4);
semilogy(y, flip_arr(D.EI), '-', 'Color', blue, 'LineWidth', 2);
xline(loc.y_hinge, 'k:', 'LineWidth', 1.5, 'Label', 'Fold hinge');
xlabel('y [m]');  ylabel('EI [Nm^2]');  title('Bending stiffness EI(y)');  grid on;

nexttile(5);
semilogy(y, flip_arr(D.GJ), '-', 'Color', orange, 'LineWidth', 2);
xline(loc.y_hinge, 'k:', 'LineWidth', 1.5, 'Label', 'Fold hinge');
xlabel('y [m]');  ylabel('GJ [Nm^2]');  title('Torsional stiffness GJ(y)');  grid on;

nexttile(6);
bar_t = [MB.m_skin, MB.m_caps, MB.m_webs, MB.m_secondary, MB.m_hinge] / 1e3;
bar_c = [blue; orange; green; [0.5 0.5 0.5]; [0.8 0.6 0.0]];
bh    = bar(bar_t, 'FaceColor', 'flat');
bh.CData = bar_c;
set(gca, 'XTickLabel', {'Skin','Caps','Webs','Secondary','Hinge'}, 'XTickLabelRotation', 20);
ylabel('Mass [t]');  title(sprintf('Mass breakdown — total %.0f kg', MB.m_total));  grid on;
for k = 1:5
    text(k, bar_t(k)+0.05, sprintf('%.1ft', bar_t(k)), 'HorizontalAlignment', 'center', 'FontSize', 8);
end

end
