function wing_structures_viz(~, loc, G, L, S, W, ~, MB, ~, MB_CF, ~)
% Focused visualizer:
% 1) 2D structural/load summary
% 2) 3D wing + wingbox view
% 3) Folding tip length vs wing mass (delegated to existing model plot)

if nargin < 10, MB_CF = struct(); end

y = fliplr(G.y);      % root -> tip
f = @(x) fliplr(x);

% ---------- 2D visualisation ----------
figure('Name','Wing Structures 2D','Color','w');
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

nexttile; hold on; grid on; box on;
xLE = f(G.x_LE);
xTE = f(G.x_LE + G.chord);
plot(y, xLE, 'b', 'DisplayName','Leading edge');
plot(y, xTE, 'r', 'DisplayName','Trailing edge');
plot(y, f(G.x_LE + loc.fs_fwd * G.chord), '--', 'Color',[0 0.6 0], 'DisplayName','Front spar');
plot(y, f(G.x_LE + loc.fs_aft * G.chord), '--', 'Color',[0.85 0.33 0.10], 'DisplayName','Rear spar');
xline(G.y_kink, 'Color', [0.20 0.20 0.20], 'LineStyle', '--', 'DisplayName','Kink');
xline(loc.y_hinge, 'k:', 'DisplayName','Fold hinge');
xline(G.y_engine1, 'm-.', 'DisplayName','Engine 1');
xline(G.y_engine2, 'c-.', 'DisplayName','Engine 2');
xlabel('y [m]'); ylabel('x [m]'); title('Planform + Structural Stations');
legend('Location','best');

nexttile; hold on; grid on; box on;
plot(y, f(L.lift_dist)/1e3, 'b', 'DisplayName','Lift');
plot(y, f(-L.w_wing)/1e3, '--', 'Color',[0 0.6 0], 'DisplayName','Wing relief');
plot(y, f(-L.w_fuel)/1e3, '--', 'Color',[0.85 0.33 0.10], 'DisplayName','Fuel relief');
plot(y, f(L.net_dist)/1e3, 'k', 'DisplayName','Net load');
xline(loc.y_hinge, 'k:');
xlabel('y [m]'); ylabel('Load [kN/m]'); title('Distributed Loads');
legend('Location','best');

nexttile; hold on; grid on; box on;
yyaxis left; plot(y, f(S.Q)/1e6, 'b'); ylabel('Q [MN]');
yyaxis right; plot(y, f(S.M)/1e6, 'r--'); ylabel('M [MNm]');
xline(loc.y_hinge, 'k:'); xlabel('y [m]');
title('Shear and Bending Moment');

nexttile; hold on; grid on; box on;
plot(y, f(W.t_skin_mm), 'b', 'DisplayName','Skin');
plot(y, f(W.A_cap_cm2), 'r--', 'DisplayName','Cap area');
plot(y, f(W.t_web_mm), 'Color',[0.49 0.18 0.56], 'LineStyle','-.', 'DisplayName','Web');
xline(loc.y_hinge, 'k:');
xlabel('y [m]'); ylabel('Sizing metric'); title('Wingbox Sizing Trends');
legend('Location','best');

% ---------- 3D visualisation ----------
figure('Name','Wing Structures 3D','Color','w');
ax = axes; hold(ax,'on'); grid(ax,'on'); box(ax,'on');

naca_yt = @(x) 5*0.12*(0.2969*sqrt(x) - 0.1260*x - 0.3516*x.^2 + 0.2843*x.^3 - 0.1015*x.^4);
idx3 = unique(round(linspace(1, G.N, 50)));
y3 = G.y(idx3); c3 = G.chord(idx3); xl3 = G.x_LE(idx3); tc3 = G.tc(idx3); h3 = G.h_wb(idx3);
xfs3 = xl3 + loc.fs_fwd * c3;
xrs3 = xl3 + loc.fs_aft * c3;

xi = linspace(0,1,28); nS = numel(idx3);
Xu = zeros(nS,numel(xi)); Yu = Xu; Zu = Xu; Xl = Xu; Yl = Xu; Zl = Xu;
for j = 1:nS
    yt = naca_yt(xi) * c3(j) * tc3(j) / 0.12;
    Xu(j,:) = xl3(j) + xi * c3(j);  Yu(j,:) = y3(j);  Zu(j,:) = yt;
    Xl(j,:) = xl3(j) + xi * c3(j);  Yl(j,:) = y3(j);  Zl(j,:) = -yt;
end

surf(ax, Xu, Yu, Zu, 'FaceAlpha',0.22, 'FaceColor',[0 0.45 0.74], 'EdgeColor','none');
surf(ax, Xl, Yl, Zl, 'FaceAlpha',0.12, 'FaceColor',[0 0.45 0.74], 'EdgeColor','none');
plot3(ax, xfs3, y3,  h3/2, 'g', 'LineWidth',2, 'DisplayName','Front spar');
plot3(ax, xfs3, y3, -h3/2, 'g', 'LineWidth',2, 'HandleVisibility','off');
plot3(ax, xrs3, y3,  h3/2, 'Color',[0.85 0.33 0.10], 'LineWidth',2, 'DisplayName','Rear spar');
plot3(ax, xrs3, y3, -h3/2, 'Color',[0.85 0.33 0.10], 'LineWidth',2, 'HandleVisibility','off');

[~, ih3] = min(abs(y3 - loc.y_hinge));
plot3(ax, [xl3(ih3), xl3(ih3)+c3(ih3)], [loc.y_hinge loc.y_hinge], [0 0], ...
    'm', 'LineWidth',3, 'DisplayName','Fold hinge');

xlabel(ax,'x [m]'); ylabel(ax,'y [m]'); zlabel(ax,'z [m]');
title(ax,'3D Wing Surface with Wingbox Spars');
view(ax,-52,22); pbaspect(ax,[2.1 5 0.45]); legend(ax,'Location','best');

% Folding-tip-vs-mass plot is generated in PlotsFoldingWingtip.m
% to keep this visualizer focused on 2D/3D geometry and loads.

end
