function SensitivityStudy(adp, tlar, loc)
% Six-panel sensitivity study + SMT load-case comparison.
% Sweeps vary adp copies (Span, MTOM); loc stays fixed throughout.

fprintf('\n=== Sensitivity Study - Wing Structural Mass ===\n');

font_ref = 'Helvetica';
legend_fs = 8;

figure('Name', 'Wing Mass Sensitivity', 'Position', [50 50 1400 900]);
tl = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, 'Wing Structural Mass - Sensitivity Study', 'FontWeight', 'bold', 'FontSize', 13);

% ---- Panel 1: AR sweep  (constant WingArea, vary Span on adp copy)
ARs   = linspace(7, 13, 10);
m_emp = zeros(size(ARs));
m_II5 = zeros(size(ARs));
for k = 1:numel(ARs)
    ak      = adp;
    ak.Span = sqrt(ARs(k) * double(adp.WingArea));
    Gk      = Unconventional.Structures.V6.WingGeometry(ak, tlar, loc);
    m_emp(k) = Unconventional.Structures.V6.EmpiricalMass(ak, tlar, loc, Gk).m_total;
    m_II5(k) = run_II5(ak, tlar, loc, '2.5g', 'Al');
end
nexttile(1);
plot(ARs, m_emp/1e3, 'b-o', 'LineWidth', 2, 'DisplayName', 'Class I/II');  hold on;
plot(ARs, m_II5/1e3, 'r-s', 'LineWidth', 2, 'DisplayName', 'Class II.5');
xline(double(adp.Span)^2/double(adp.WingArea), 'k--', 'LineWidth', 1.5, 'Label', 'Design', 'DisplayName', 'Design');
xlabel('AR');  ylabel('Wing mass [t]');  title('Mass vs Aspect Ratio');
legend('Location', 'northwest', 'FontSize', legend_fs);  grid on;

% ---- Panel 2: MTOM sweep
MTOMs  = linspace(200e3, 420e3, 10);
m2_emp = zeros(size(MTOMs));
m2_II5 = zeros(size(MTOMs));
for k = 1:numel(MTOMs)
    ak      = adp;
    ak.MTOM = MTOMs(k);
    Gk      = Unconventional.Structures.V6.WingGeometry(ak, tlar, loc);
    m2_emp(k) = Unconventional.Structures.V6.EmpiricalMass(ak, tlar, loc, Gk).m_total;
    m2_II5(k) = run_II5(ak, tlar, loc, '2.5g', 'Al');
end
nexttile(2);
plot(MTOMs/1e3, m2_emp/1e3, 'b-o', 'LineWidth', 2, 'DisplayName', 'Class I/II');  hold on;
plot(MTOMs/1e3, m2_II5/1e3, 'r-s', 'LineWidth', 2, 'DisplayName', 'Class II.5');
xline(double(adp.MTOM)/1e3, 'k--', 'LineWidth', 1.5, 'Label', 'Design', 'DisplayName', 'Design');
xlabel('MTOM [t]');  ylabel('Wing mass [t]');  title('Mass vs MTOM');
legend('Location', 'northwest', 'FontSize', legend_fs);  grid on;

% ---- Panel 3: Wingspan sweep  (Code E and Code F limits marked)
spans  = linspace(55, 80, 11);
m3_emp = zeros(size(spans));
m3_II5 = zeros(size(spans));
for k = 1:numel(spans)
    ak      = adp;
    ak.Span = spans(k);
    Gk      = Unconventional.Structures.V6.WingGeometry(ak, tlar, loc);
    m3_emp(k) = Unconventional.Structures.V6.EmpiricalMass(ak, tlar, loc, Gk).m_total;
    m3_II5(k) = run_II5(ak, tlar, loc, '2.5g', 'Al');
end
nexttile(3);
plot(spans, m3_emp/1e3, 'b-o', 'LineWidth', 2, 'DisplayName', 'Class I/II');  hold on;
plot(spans, m3_II5/1e3, 'r-s', 'LineWidth', 2, 'DisplayName', 'Class II.5');
xline(65,              'g:', 'LineWidth', 2,   'Label', 'Code E 65m', 'DisplayName', 'Code E 65m');
xline(80,              'k:', 'LineWidth', 2,   'Label', 'Code F 80m', 'DisplayName', 'Code F 80m');
xline(double(adp.Span),'k--','LineWidth', 1.5, 'Label', 'Design', 'DisplayName', 'Design');
xlabel('Wingspan [m]');  ylabel('Wing mass [t]');  title('Mass vs Wingspan');
legend('Location', 'northwest', 'FontSize', legend_fs);  grid on;

% ---- Panel 4: Load case comparison
cases    = {'2.5g', '1g', 'neg1g'};
case_lbl = {'2.5g', '1g', '-1g'};
m4 = cellfun(@(c) run_II5(adp, tlar, loc, c, 'Al'), cases);
[m_crit, ic] = max(m4);
nexttile(4);
bh = bar(m4/1e3, 'FaceColor', 'flat');
bh.CData = repmat([0.2 0.5 0.8], 3, 1);
bh.CData(ic,:) = [0.8 0.1 0.1];
set(gca, 'XTickLabel', case_lbl);
ylabel('Wing mass [t]');  title('Mass by load case');  grid on;
for k = 1:3
    text(k, m4(k)/1e3 + 0.3, sprintf('%.1ft', m4(k)/1e3), 'HorizontalAlignment', 'center', 'FontSize', 9);
end
text(ic, m_crit/1e3 + 1.5, 'CRITICAL', 'Color', 'r', 'HorizontalAlignment', 'center', 'FontSize', 9, 'FontWeight', 'bold');

% ---- Panel 5: Material comparison
mats    = {'Al', 'CF'};
mat_lbl = {'Al 7075-T6', 'CFRP'};
G_dp    = Unconventional.Structures.V6.WingGeometry(adp, tlar, loc);
m5_emp  = cellfun(@(m) Unconventional.Structures.V6.EmpiricalMass(adp, tlar, loc, G_dp, m).m_total, mats);
m5_II5  = cellfun(@(m) run_II5(adp, tlar, loc, '2.5g', m), mats);
nexttile(5);
bh = bar([m5_emp; m5_II5]'/1e3);
bh(1).FaceColor = [0.2 0.5 0.8];
bh(2).FaceColor = [0.8 0.4 0.1];
set(gca, 'XTickLabel', mat_lbl);
ylabel('Wing mass [t]');  title('Material comparison');
legend({'Class I/II','Class II.5'}, 'Location', 'northeast', 'FontSize', legend_fs);  grid on;
saving_pct = (m5_II5(1) - m5_II5(2)) / m5_II5(1) * 100;
text(1.5, max(m5_emp)/1e3*0.5, sprintf('CFRP saves\n%.0f%%', saving_pct), ...
    'HorizontalAlignment', 'center', 'FontSize', 10, 'Color', [0 0.5 0]);

% ---- Panel 6: Fidelity ladder at design point
E_dp     = Unconventional.Structures.V6.EmpiricalMass(adp, tlar, loc, G_dp, 'Al');
m_II5_dp = run_II5(adp, tlar, loc, '2.5g', 'Al');
fid_vals = [E_dp.m_raymer, E_dp.m_torenbeek, E_dp.m_usaf, E_dp.m_total, m_II5_dp] / 1e3;
fid_lbl  = {'Raymer','Torenbeek','USAF','I/II avg','II.5'};
colors   = [0.2 0.5 0.8; 0.3 0.6 0.9; 0.4 0.7 1.0; 0.1 0.4 0.7; 0.8 0.2 0.2];
nexttile(6);
for k = 1:5
    bar(k, fid_vals(k), 'FaceColor', colors(k,:));  hold on;
end
set(gca, 'XTick', 1:5, 'XTickLabel', fid_lbl, 'XTickLabelRotation', 15);
ylabel('Wing mass [t]');  title('Fidelity ladder');  grid on;
yline(34, 'k--', 'LineWidth', 2, 'Label', 'B777F 34t');
yline(69, 'Color', [0.25 0.25 0.25], 'LineStyle', '-.', 'LineWidth', 2, ...
    'Label', 'A380 69t');
for k = 1:5
    text(k, fid_vals(k)+0.3, sprintf('%.1ft', fid_vals(k)), 'HorizontalAlignment', 'center', 'FontSize', 9);
end

% =========================================================================
%  FIGURE 2 - SMT comparison across all three load cases
% =========================================================================
G      = Unconventional.Structures.V6.WingGeometry(adp, tlar, loc);
y_plot = fliplr(G.y);
cases  = {'2.5g','1g','neg1g'};
lnames = {'2.5g','1g','-1g'};
clrs   = {[0.0 0.45 0.74],[0.47 0.67 0.19],[0.85 0.33 0.10]};
lss    = {'-', '--', '-.'};

figure('Name', 'SMT All Load Cases', 'Position', [100 100 1200 700]);
tl = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, 'Shear / Bending / Torque - All Load Cases', 'FontWeight', 'bold');

ax_Q = nexttile(1);  hold(ax_Q, 'on');
ax_M = nexttile(2);  hold(ax_M, 'on');
ax_T = nexttile(3);  hold(ax_T, 'on');

for k = 1:3
    Lk = Unconventional.Structures.V6.LoadDistribution(adp, tlar, loc, G, cases{k});
    Sk = Unconventional.Structures.V6.SMT(loc, G, Lk);
    plot(ax_Q, y_plot, fliplr(Sk.Q)/1e6, lss{k}, 'Color', clrs{k}, 'LineWidth', 2, 'DisplayName', lnames{k});
    plot(ax_M, y_plot, fliplr(Sk.M)/1e6, lss{k}, 'Color', clrs{k}, 'LineWidth', 2, 'DisplayName', lnames{k});
    plot(ax_T, y_plot, fliplr(Sk.T)/1e6, lss{k}, 'Color', clrs{k}, 'LineWidth', 2, 'DisplayName', lnames{k});
end

for ax = [ax_Q, ax_M, ax_T]
    xline(ax, loc.y_hinge, 'k:', 'LineWidth', 1.5, 'Label', 'Fold hinge', 'DisplayName', 'Fold hinge');
    xline(ax, G.y_engine,  'r:', 'LineWidth', 1.2, 'Label', 'Engine', 'DisplayName', 'Engine');
    grid(ax, 'on');
    xlabel(ax, 'y [m]');
    legend(ax, 'Location', 'northeast', 'FontSize', legend_fs);
end
ylabel(ax_Q, 'Q [MN]');   title(ax_Q, 'Shear Force');
ylabel(ax_M, 'M [MNm]');  title(ax_M, 'Bending Moment');
ylabel(ax_T, 'T [MNm]');  title(ax_T, 'Torque');

fig_ws = findall(0, 'Type', 'figure', 'Name', 'Wing Mass Sensitivity');
if ~isempty(fig_ws)
    set(findall(fig_ws, '-property', 'FontName'), 'FontName', font_ref);
end
fig_smt = findall(0, 'Type', 'figure', 'Name', 'SMT All Load Cases');
if ~isempty(fig_smt)
    set(findall(fig_smt, '-property', 'FontName'), 'FontName', font_ref);
end

end


function m = run_II5(adp, tlar, loc, lc, mat)
    Gk  = Unconventional.Structures.V6.WingGeometry(adp, tlar, loc);
    Lk  = Unconventional.Structures.V6.LoadDistribution(adp, tlar, loc, Gk, lc);
    Sk  = Unconventional.Structures.V6.SMT(loc, Gk, Lk);
    Wk  = Unconventional.Structures.V6.WingboxSizing(loc, Gk, Sk, mat);
    MBk = Unconventional.Structures.V6.MassBuildup(adp, loc, Gk, Wk);
    m   = MBk.m_total;
end
