function SensitivityStudy(p_base)

fprintf('\n========================================\n');
fprintf('  SENSITIVITY STUDY — Wing Structural Mass\n');
fprintf('========================================\n');

% =========================================================================
%  FIGURE 1 — SIX-PANEL SENSITIVITY STUDY
% =========================================================================
figure('Name', 'Wing Mass Sensitivity Study', 'Position', [50 50 1400 900]);
tl1 = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl1, 'Wing Structural Mass — Sensitivity Study', ...
    'FontWeight', 'bold', 'FontSize', 13);

% ---- Panel 1: Wing mass vs Aspect Ratio (constant wing area)
fprintf('  Panel 1: AR sweep ...\n');
ARs   = linspace(7, 13, 10);
m_emp = zeros(size(ARs));
m_II5 = zeros(size(ARs));

for k = 1:numel(ARs)
    pk          = p_base;
    pk.Span     = sqrt(ARs(k) * p_base.WingArea);
    pk.AR       = ARs(k);
    pk.c_root   = 2*pk.WingArea / (pk.Span*(1+pk.lambda));
    pk.c_tip    = pk.lambda * pk.c_root;
    pk.MAC      = (2/3)*pk.c_root*(1+pk.lambda+pk.lambda^2)/(1+pk.lambda);
    pk.y_engine = 0.35 * pk.Span/2;
    E_k         = Unconventional.Structures.Snorri.EmpiricalMass(pk);
    m_emp(k)    = E_k.m_total;
    m_II5(k)    = local_II5mass(pk, '2.5g', 'Al');
end

nexttile(1);
plot(ARs, m_emp/1e3, 'b-o', 'LineWidth', 2, 'DisplayName', 'Class I/II');
hold on;
plot(ARs, m_II5/1e3, 'r-s', 'LineWidth', 2, 'DisplayName', 'Class II.5');
xline(p_base.AR, 'k--', 'LineWidth', 1.5, 'Label', 'Design');
xlabel('Aspect Ratio [-]');  ylabel('Wing mass [t]');
title('Wing Mass vs Aspect Ratio');
legend('Location', 'northwest');  grid on;

% ---- Panel 2: Wing mass vs MTOM
fprintf('  Panel 2: MTOM sweep ...\n');
MTOMs  = linspace(200e3, 420e3, 10);
m2_emp = zeros(size(MTOMs));
m2_II5 = zeros(size(MTOMs));

for k = 1:numel(MTOMs)
    pk        = p_base;
    pk.MTOM   = MTOMs(k);
    pk.M_fuel = pk.Mf_fuel * pk.MTOM;
    pk.W_useful = pk.MTOM - pk.OEM;
    E_k       = Unconventional.Structures.Snorri.EmpiricalMass(pk);
    m2_emp(k) = E_k.m_total;
    m2_II5(k) = local_II5mass(pk, '2.5g', 'Al');
end

nexttile(2);
plot(MTOMs/1e3, m2_emp/1e3, 'b-o', 'LineWidth', 2, 'DisplayName', 'Class I/II');
hold on;
plot(MTOMs/1e3, m2_II5/1e3, 'r-s', 'LineWidth', 2, 'DisplayName', 'Class II.5');
xline(p_base.MTOM/1e3, 'k--', 'LineWidth', 1.5, 'Label', 'Design');
xlabel('MTOM [t]');  ylabel('Wing mass [t]');
title('Wing Mass vs MTOM');
legend('Location', 'northwest');  grid on;

% ---- Panel 3: Wing mass vs Wingspan (Code E and F limits)
fprintf('  Panel 3: Wingspan sweep ...\n');
spans  = linspace(55, 80, 11);
m3_emp = zeros(size(spans));
m3_II5 = zeros(size(spans));

for k = 1:numel(spans)
    pk          = p_base;
    pk.Span     = spans(k);
    pk.AR       = pk.Span^2 / pk.WingArea;
    pk.c_root   = 2*pk.WingArea / (pk.Span*(1+pk.lambda));
    pk.c_tip    = pk.lambda * pk.c_root;
    pk.MAC      = (2/3)*pk.c_root*(1+pk.lambda+pk.lambda^2)/(1+pk.lambda);
    pk.y_engine = 0.35 * pk.Span/2;
    E_k         = Unconventional.Structures.Snorri.EmpiricalMass(pk);
    m3_emp(k)   = E_k.m_total;
    m3_II5(k)   = local_II5mass(pk, '2.5g', 'Al');
end

nexttile(3);
plot(spans, m3_emp/1e3, 'b-o', 'LineWidth', 2, 'DisplayName', 'Class I/II');
hold on;
plot(spans, m3_II5/1e3, 'r-s', 'LineWidth', 2, 'DisplayName', 'Class II.5');
xline(65, 'g:', 'LineWidth', 2, 'Label', 'Code E taxi 65m');
xline(80, 'k:', 'LineWidth', 2, 'Label', 'Code F flight 80m');
xline(p_base.Span, 'k--', 'LineWidth', 1.5, 'Label', 'Design 72m');
xlabel('Flight wingspan [m]');  ylabel('Wing mass [t]');
title('Wing Mass vs Wingspan');
legend('Location', 'northwest');  grid on;

% ---- Panel 4: Load case comparison
fprintf('  Panel 4: Load case comparison ...\n');
lcs     = {'2.5g', '1g', 'neg1g'};
lc_lbls = {'2.5g manoeuvre', '1g level', '-1g inverted'};
m4      = zeros(1, 3);
for k = 1:3
    m4(k) = local_II5mass(p_base, lcs{k}, 'Al');
end
[m_crit, ic] = max(m4);

nexttile(4);
b4 = bar(m4/1e3, 'FaceColor', 'flat');
b4.CData = repmat([0.2 0.5 0.8], 3, 1);
b4.CData(ic, :) = [0.8 0.1 0.1];
set(gca, 'XTickLabel', lc_lbls);
ylabel('Wing mass [t]');
title('Wing Mass by Load Case (Class II.5)');  grid on;
for k = 1:3
    text(k, m4(k)/1e3 + 0.3, sprintf('%.1f t', m4(k)/1e3), ...
        'HorizontalAlignment', 'center', 'FontSize', 9);
end
text(ic, m_crit/1e3 + 1.5, 'CRITICAL', 'Color', 'r', ...
    'HorizontalAlignment', 'center', 'FontSize', 9, 'FontWeight', 'bold');

% ---- Panel 5: Material comparison
fprintf('  Panel 5: Material comparison ...\n');
mats     = {'Al', 'CF'};
mat_lbls = {'Al 7075-T6', 'CFRP'};
m5_emp   = zeros(1, 2);
m5_II5   = zeros(1, 2);
for k = 1:2
    E_k       = Unconventional.Structures.Snorri.EmpiricalMass(p_base, mats{k});
    m5_emp(k) = E_k.m_total;
    m5_II5(k) = local_II5mass(p_base, '2.5g', mats{k});
end

nexttile(5);
bar_data = [m5_emp; m5_II5]' / 1e3;
bh = bar(bar_data);
bh(1).FaceColor = [0.2 0.5 0.8];
bh(2).FaceColor = [0.8 0.4 0.1];
set(gca, 'XTickLabel', mat_lbls);
ylabel('Wing mass [t]');
title('Material Comparison');
legend({'Class I/II', 'Class II.5'}, 'Location', 'northeast');  grid on;
saving_pct = (m5_II5(1) - m5_II5(2)) / m5_II5(1) * 100;
text(1.5, max(m5_emp)/1e3*0.5, sprintf('CFRP saves\n%.0f%%', saving_pct), ...
    'HorizontalAlignment', 'center', 'FontSize', 10, 'Color', [0 0.5 0]);

% ---- Panel 6: Fidelity ladder
fprintf('  Panel 6: Fidelity ladder ...\n');
E_dp     = Unconventional.Structures.Snorri.EmpiricalMass(p_base, 'Al');
m_II5_dp = local_II5mass(p_base, '2.5g', 'Al');

fid_vals = [E_dp.m_raymer, E_dp.m_torenbeek, E_dp.m_usaf, ...
            E_dp.m_total,  m_II5_dp] / 1e3;
fid_lbls = {'Raymer', 'Torenbeek', 'USAF', 'I/II avg', 'II.5 (2.5g)'};
colors5  = [0.2 0.5 0.8; 0.3 0.6 0.9; 0.4 0.7 1.0; 0.1 0.4 0.7; 0.8 0.2 0.2];

nexttile(6);
for k = 1:5
    bar(k, fid_vals(k), 'FaceColor', colors5(k,:));  hold on;
end
set(gca, 'XTickLabel', fid_lbls, 'XTickLabelRotation', 15);
ylabel('Wing mass [t]');
title('Fidelity Ladder — Design Point');  grid on;
yline(34, 'k--', 'LineWidth', 2, 'Label', 'B777F ref 34t');
for k = 1:5
    text(k, fid_vals(k) + 0.3, sprintf('%.1ft', fid_vals(k)), ...
        'HorizontalAlignment', 'center', 'FontSize', 9);
end

% =========================================================================
%  FIGURE 2 — SMT COMPARISON: ALL THREE LOAD CASES
% =========================================================================
fprintf('\n  SMT comparison figure ...\n');
G = Unconventional.Structures.Snorri.WingGeometry(p_base);

cases    = {'2.5g', '1g', 'neg1g'};
lc_names = {'2.5g manoeuvre', '1g level', '-1g inverted'};
clrs     = {[0.0 0.45 0.74], [0.47 0.67 0.19], [0.85 0.33 0.10]};
lss      = {'-', '--', '-.'};

figure('Name', 'SMT Comparison — All Load Cases', 'Position', [100 100 1200 700]);
tl2 = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl2, 'Shear / Bending / Torque — All Load Cases', 'FontWeight', 'bold');

ax_Q = nexttile(1);  hold(ax_Q, 'on');
ax_M = nexttile(2);  hold(ax_M, 'on');
ax_T = nexttile(3);  hold(ax_T, 'on');

y_plot = fliplr(G.y);

for k = 1:3
    Lk = Unconventional.Structures.Snorri.LoadDistribution(p_base, G, cases{k});
    Sk = Unconventional.Structures.Snorri.SMT(p_base, G, Lk);
    plot(ax_Q, y_plot, fliplr(Sk.Q)/1e6, lss{k}, 'Color', clrs{k}, 'LineWidth', 2, 'DisplayName', lc_names{k});
    plot(ax_M, y_plot, fliplr(Sk.M)/1e6, lss{k}, 'Color', clrs{k}, 'LineWidth', 2, 'DisplayName', lc_names{k});
    plot(ax_T, y_plot, fliplr(Sk.T)/1e6, lss{k}, 'Color', clrs{k}, 'LineWidth', 2, 'DisplayName', lc_names{k});
end

for ax = [ax_Q, ax_M, ax_T]
    xline(ax, p_base.y_hinge,  'k:', 'LineWidth', 1.5, 'Label', 'Fold hinge');
    xline(ax, p_base.y_engine, 'r:', 'LineWidth', 1.2, 'Label', 'Engine');
    grid(ax, 'on');
    xlabel(ax, 'y [m] (root to tip)');
    legend(ax, 'Location', 'best', 'FontSize', 8);
end
ylabel(ax_Q, 'Q [MN]');    title(ax_Q, 'Shear Force');
ylabel(ax_M, 'M [MNm]');   title(ax_M, 'Bending Moment');
ylabel(ax_T, 'T [MNm]');   title(ax_T, 'Torque');

fprintf('\nSensitivity study complete.\n');

end


function m = local_II5mass(p, lc, mat)
    Gk  = Unconventional.Structures.Snorri.WingGeometry(p);
    Lk  = Unconventional.Structures.Snorri.LoadDistribution(p, Gk, lc);
    Sk  = Unconventional.Structures.Snorri.SMT(p, Gk, Lk);
    Wk  = Unconventional.Structures.Snorri.WingboxSizing(p, Gk, Sk, mat);
    MBk = Unconventional.Structures.Snorri.MassBuildup(p, Gk, Wk);
    m   = MBk.m_total;
end
