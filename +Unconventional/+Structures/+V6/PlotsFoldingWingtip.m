function PlotsFoldingWingtip(loc, G, S_25g, S_1g, W_25g, D_25g, FT)
% Folding wingtip structural outputs - six-panel figure.

y      = fliplr(G.y);
ih     = G.i_hinge;
ih_pl  = G.N - ih + 1;   % flipped index for plotting

blue   = [0.00 0.45 0.74];
orange = [0.85 0.33 0.10];
green  = [0.47 0.67 0.19];
purple = [0.49 0.18 0.56];
red    = [0.80 0.10 0.10];

figure('Name', 'Folding Wingtip', 'Position', [80 80 1400 800]);
tl = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf('Folding Wingtip Structural Analysis  —  y_{hinge} = %.1f m', loc.y_hinge), ...
    'FontWeight', 'bold');

% Panel 1: SMT comparison flight vs fold load case at outer panel
nexttile(1);  hold on;
Q_fl   = fliplr(abs(S_25g.Q)) / 1e6;
Q_fold = fliplr(abs(S_1g.Q))  * loc.n_limit_fold / 1e6;
plot(y, Q_fl,   '-',  'Color', blue,   'LineWidth', 2, 'DisplayName', sprintf('2.5g flight'));
plot(y, Q_fold, '--', 'Color', orange, 'LineWidth', 2, 'DisplayName', sprintf('%.1fg fold', loc.n_limit_fold));
xline(loc.y_hinge, 'k:', 'LineWidth', 2, 'Label', 'Fold hinge');
xlim([loc.y_hinge*0.6, G.s*1.05]);
xlabel('y [m]');  ylabel('Q [MN]');  title('Outer panel shear — flight vs fold');
legend('Location', 'northwest');  grid on;

nexttile(2);  hold on;
M_fl   = fliplr(abs(S_25g.M)) / 1e6;
M_fold = fliplr(abs(S_1g.M))  * loc.n_limit_fold / 1e6;
plot(y, M_fl,   '-',  'Color', blue,   'LineWidth', 2, 'DisplayName', '2.5g flight');
plot(y, M_fold, '--', 'Color', orange, 'LineWidth', 2, 'DisplayName', sprintf('%.1fg fold', loc.n_limit_fold));
xline(loc.y_hinge, 'k:', 'LineWidth', 2, 'Label', 'Fold hinge');
xlim([loc.y_hinge*0.6, G.s*1.05]);
xlabel('y [m]');  ylabel('M [MNm]');  title('Outer panel BM — flight vs fold');
legend('Location', 'northwest');  grid on;

% Panel 3: EI and GJ across hinge - stiffness discontinuity
nexttile(3);  hold on;
semilogy(y, fliplr(D_25g.EI), '-',  'Color', blue,   'LineWidth', 2, 'DisplayName', 'EI');
semilogy(y, fliplr(D_25g.GJ), '--', 'Color', orange, 'LineWidth', 2, 'DisplayName', 'GJ');
xline(loc.y_hinge, 'k:', 'LineWidth', 2, 'Label', 'Fold hinge');
text(loc.y_hinge + 0.3, FT.EI_outer*1.5, ...
    sprintf('EI ratio %.1f×\nGJ ratio %.1f×', FT.EI_ratio, FT.GJ_ratio), ...
    'FontSize', 8, 'Color', blue);
xlim([loc.y_hinge*0.6, G.s*1.05]);
xlabel('y [m]');  ylabel('[Nm²]');  title('EI / GJ discontinuity at hinge');
legend('Location', 'northeast');  grid on;

% Panel 4: Skin and cap sizing in outer panel
nexttile(4);  hold on;
y_out = y(ih_pl:end);
yyaxis left;
plot(y_out, fliplr(W_25g.t_skin_mm(1:ih)), '-',  'Color', blue,   'LineWidth', 2);
ylabel('t_{skin} [mm]');
yyaxis right;
plot(y_out, fliplr(W_25g.A_cap_cm2(1:ih)), '--', 'Color', orange, 'LineWidth', 2);
ylabel('A_{cap} [cm²]');
xline(loc.y_hinge, 'k:', 'LineWidth', 2, 'Label', 'Fold hinge');
xlabel('y [m]');  title('Outer panel skin & cap sizing');
legend({'t_{skin}','A_{cap}'}, 'Location', 'northwest');  grid on;

% Panel 5: Outer panel mass breakdown bar chart
nexttile(5);
labels = {'Box primary','Lock mech','Actuator','Hinge assy'};
vals   = [FT.m_outer_prim, FT.m_lock, FT.m_actuator, FT.m_hinge_assy] / 1e3;
clrs   = [blue; green; purple; red];
bh     = bar(vals, 'FaceColor', 'flat');
bh.CData = clrs;
set(gca, 'XTickLabel', labels, 'XTickLabelRotation', 20);
ylabel('Mass [t]');
title(sprintf('Outer panel mass  —  total %.0f kg', FT.m_outer_total));
for k = 1:4
    text(k, vals(k)+0.02, sprintf('%.2ft', vals(k)), 'HorizontalAlignment', 'center', 'FontSize', 8);
end
grid on;

% Panel 6: Summary text
nexttile(6);  axis off;
txt = {
    sprintf('\\bfFOLD HINGE  y = %.1f m\\rm', FT.y_hinge),
    sprintf('Outer panel span   %.3f m', FT.b_tip),
    ' ',
    sprintf('\\bfTAXI COMPLIANCE\\rm'),
    sprintf('Folded span        %.2f m', FT.span_taxi),
    sprintf('Code E limit       65.00 m'),
    sprintf('Margin             %+.2f m  %s', FT.code_E_margin, ...
        ternary(FT.code_E_margin >= 0, '(PASS)', '(FAIL)')),
    ' ',
    sprintf('\\bfHINGE PIN  (titanium, double-shear)\\rm'),
    sprintf('Diameter           %.1f mm', FT.d_pin*1e3),
    sprintf('Bearing thickness  %.1f mm', FT.t_bearing*1e3),
    ' ',
    sprintf('\\bfHINGE LOADS  (2.5g flight)\\rm'),
    sprintf('Q = %.3f MN', FT.Q_hinge_25g/1e6),
    sprintf('M = %.3f MNm', FT.M_hinge_25g/1e6),
    sprintf('T = %.3f MNm', FT.T_hinge_25g/1e6),
    ' ',
    sprintf('\\bfFOLD PENALTY vs fixed wing\\rm'),
    sprintf('%+.0f kg', FT.m_fold_penalty),
};
text(0.05, 0.97, txt, 'Units', 'norm', 'VerticalAlignment', 'top', ...
    'FontSize', 9.5, 'Interpreter', 'tex');

end

function out = ternary(cond, a, b)
    if cond,  out = a;  else,  out = b;  end
end
