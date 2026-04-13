function PlotWingAndFoldSensitivity(G, loc, MB_Al, MB_CF)

Span         = G.Span;
s            = G.s;
y            = G.y;              % tip→root station array
chord        = G.chord;
h_wb         = G.h_wb;
w_wb         = G.w_wb;
x_LE         = G.x_LE;
sweep_LE     = deg2rad(G.sweep_LE_deg);
lambda       = G.lambda;
c_root       = G.c_root;
c_tip        = G.c_tip;
y_hinge      = G.y_hinge;
y_eng1       = G.y_engine1;
y_eng2       = G.y_engine2;
dy           = G.dy;
N            = G.N;

Span_taxi    = loc.Span_taxi;
fs_fwd       = loc.fs_fwd;
fs_aft       = loc.fs_aft;
f_sec        = loc.f_secondary;
f_hinge      = loc.f_hinge_mech;
f_lock       = loc.f_lock_mech;
f_act        = loc.f_actuator;
m_hinge_min  = loc.m_hinge_min;
Al           = loc.Al;
CF           = loc.CF;

% Design-point outer panel mass (already computed by MassBuildup)
m_outer_Al  = MB_Al.m_outer_primary;
m_outer_CF  = MB_CF.m_outer_primary;
m_total_Al  = MB_Al.m_total;
m_total_CF  = MB_CF.m_total;

% Colour palette
C = struct('blue',[0.086 0.369 0.631], 'orange',[0.847 0.329 0.102], ...
           'green',[0.188 0.627 0.224], 'purple',[0.42 0.18 0.56], ...
           'red',[0.78 0.11 0.11],      'grey',[0.60 0.60 0.60]);

%  FIGURE 1 - 2D PLANFORM

figure('Name','Wing Planform - 2D Top View','Position',[60 60 1100 580]);
ax = axes; hold on; grid on; box on; axis equal;

x_TE = x_LE + chord;                       % trailing-edge x array
x_FS = x_LE + fs_fwd * chord;              % front spar
x_RS = x_LE + fs_aft  * chord;             % rear spar
y_plot = fliplr(y);                         % root→tip for plotting

% helper: flip array to match y_plot direction
fl = @(v) fliplr(v);

% Wing fill - outer (folding) panel lighter
ih = G.i_hinge;
fill([fl(x_LE(1:ih)),  fl(x_TE(1:ih))],  [fl(y(1:ih)),  fl(y(1:ih))],  ...
    [0.80 0.90 1.0], 'EdgeColor','none');                   % outer tint
fill([fl(x_LE(ih:N)), fl(x_TE(ih:N))],  [fl(y(ih:N)), fl(y(ih:N))],  ...
    [0.72 0.82 0.94], 'EdgeColor','none');                  % inner tint

% Mirror to port side
for sgn = [-1 1]
    plot(fl(x_LE),  sgn*y_plot, '-',  'Color',C.blue,   'LineWidth',2.2);  % LE
    plot(fl(x_TE),  sgn*y_plot, '-',  'Color',C.blue,   'LineWidth',2.2);  % TE
    plot(fl(x_FS),  sgn*y_plot, '--', 'Color',C.orange, 'LineWidth',1.5);  % front spar
    plot(fl(x_RS),  sgn*y_plot, '--', 'Color',C.red,    'LineWidth',1.5);  % rear spar
    % hinge line
    plot([x_FS(ih) x_RS(ih)], sgn*[y_hinge y_hinge], '-', ...
        'Color',C.purple, 'LineWidth',3);
    % engine stations
    scatter(x_LE(G.i_engine1)+0.5*chord(G.i_engine1), sgn*y_eng1, ...
        80, C.orange, 'filled', 'Marker','^');
    scatter(x_LE(G.i_engine2)+0.5*chord(G.i_engine2), sgn*y_eng2, ...
        80, C.orange, 'filled', 'Marker','v');
end

% Code E taxi limit
yline( Span_taxi/2, ':', 'Color',C.green, 'LineWidth',1.8, 'Label','Code E');
yline(-Span_taxi/2, ':', 'Color',C.green, 'LineWidth',1.8, 'HandleVisibility','off');

% Fuselage box
fill([-0.5 c_root+0.5 c_root+0.5 -0.5], [-3.15 -3.15 3.15 3.15], ...
    [0.70 0.72 0.76], 'EdgeColor','none', 'FaceAlpha',0.35);

xlabel('x [m]  (streamwise)');  ylabel('y [m]  (spanwise)');
title(sprintf('Wing Planform - AR %.2f  \\lambda %.2f  \\Lambda_{LE} %.1f°  b %.0f m', ...
    G.AR, lambda, G.sweep_LE_deg, Span), 'FontWeight','bold');
legend({'Folding tip','Inner wing','LE / TE','Front spar (15%)','Rear spar (60%)', ...
    'Fold hinge','Engine stations','Code E'}, 'Location','northeast','FontSize',8);

%  FIGURE 2 - 3D WING SURFACE

figure('Name','Wing - 3D Isometric','Position',[100 60 1000 620]);
ax3 = axes; hold on; grid on; box on;

nc = 30;                                    % chordwise points
xi = linspace(0,1,nc);
% NACA symmetric thickness shape
zt = @(xi_) 0.4*xi_.^0.5 - 0.15*xi_ - 0.35*xi_.^2 + 0.28*xi_.^3;

% Build top/bottom surfaces from G arrays (tip→root, so plot root→tip)
X_top = zeros(nc,N);  X_bot = zeros(nc,N);
Y_mat = zeros(nc,N);  Z_top = zeros(nc,N); Z_bot = zeros(nc,N);
for j = 1:N
    t_half = 0.5 * h_wb(j);
    X_top(:,j) = (x_LE(j) + xi*chord(j))';
    X_bot(:,j) = X_top(:,j);
    Z_top(:,j) = (zt(xi)*t_half)';
    Z_bot(:,j) = -Z_top(:,j);
    Y_mat(:,j) = y(j);
end
C_chord = repmat(chord, nc, 1);             % colour by chord length

for sgn = [1 -1]
    surf(X_top, sgn*Y_mat, Z_top, C_chord, 'EdgeColor','none','FaceAlpha',0.85);
    surf(X_bot, sgn*Y_mat, Z_bot, C_chord, 'EdgeColor','none','FaceAlpha',0.45);
end

% Hinge line (both sides)
for sgn = [1 -1]
    plot3([x_FS(ih) x_RS(ih)], sgn*[y_hinge y_hinge], [0 0], ...
        '-', 'Color',C.purple, 'LineWidth',3);
end

% Engine markers
for sgn = [1 -1]
    for ie = [G.i_engine1 G.i_engine2]
        plot3(x_LE(ie)+0.5*chord(ie), sgn*y(ie), 0, 'o', ...
            'MarkerFaceColor',C.orange, 'MarkerEdgeColor','k', 'MarkerSize',10);
    end
end

colormap(ax3, flipud(cool(256)));
cb = colorbar('Location','eastoutside');
cb.Label.String = 'Chord [m]';
clim([c_tip, c_root]);
view(215, 28);  axis equal;
xlabel('x [m]');  ylabel('y [m]');  zlabel('z [m]');
title('Wing 3D Surface - NACA thickness, colour = chord','FontWeight','bold');
lighting gouraud;  light('Position',[100 50 50]);

%  FIGURES 3 & 4 - FOLDING TIP SENSITIVITY

b_tips = linspace(0, 0.38*s, 60);          % outer panel span per side [m]

m_Al = zeros(size(b_tips));
m_CF = zeros(size(b_tips));
m_ha = zeros(size(b_tips));   % hinge assembly
m_lk = zeros(size(b_tips));   % lock mechanism
m_ac = zeros(size(b_tips));   % fold actuator
m_rb = zeros(size(b_tips));   % hinge rib (from G geometry at hinge)

for k = 1:numel(b_tips)
    y_h = s - b_tips(k);                   % hinge position at this sweep step
    [~, ih_k] = min(abs(y - y_h));         % nearest station in G

    % Outer panel primary mass - scale from MB using dist arrays
    % MB.m_skin_dist / m_total_dist are per-station kg (one semi-wing)
    m_out_Al = 2*(sum(MB_Al.m_skin_dist(1:ih_k)) + ...
                  sum(MB_Al.m_total_dist(1:ih_k)-MB_Al.m_skin_dist(1:ih_k)));
    m_out_CF = 2*(sum(MB_CF.m_skin_dist(1:ih_k)) + ...
                  sum(MB_CF.m_total_dist(1:ih_k)-MB_CF.m_skin_dist(1:ih_k)));

    % Fold mechanism masses (same fractions as loc)
    ha_Al = max(f_hinge*m_out_Al, m_hinge_min);
    ha_CF = max(f_hinge*m_out_CF, m_hinge_min);

    % Hinge rib from actual G geometry at this hinge station
    Q_rib = abs(MB_Al.m_total_dist(ih_k)*1/dy * w_wb(ih_k));  % simplified
    t_rib = max(1.5*Q_rib / (h_wb(ih_k)*Al.tau_all), Al.t_min);
    m_rb(k) = Al.rho * 2*(h_wb(ih_k)*w_wb(ih_k)) * t_rib;

    pen_Al = ha_Al + f_lock*m_out_Al + f_act*m_out_Al + m_rb(k);
    pen_CF = ha_CF + f_lock*m_out_CF + f_act*m_out_CF + m_rb(k);

    % Inner panel total mass (stations ih_k → N)
    inner_Al = 2*sum(MB_Al.m_total_dist(ih_k:N));
    inner_CF = 2*sum(MB_CF.m_total_dist(ih_k:N));
    sec_inner_Al = f_sec * inner_Al;
    sec_inner_CF = f_sec * inner_CF;

    m_Al(k) = inner_Al + sec_inner_Al + 2*sum(MB_Al.m_total_dist(1:ih_k)) + ...
               f_sec*2*sum(MB_Al.m_total_dist(1:ih_k)) + pen_Al;
    m_CF(k) = inner_CF + sec_inner_CF + 2*sum(MB_CF.m_total_dist(1:ih_k)) + ...
               f_sec*2*sum(MB_CF.m_total_dist(1:ih_k)) + pen_CF;

    m_ha(k) = ha_Al;
    m_lk(k) = f_lock * m_out_Al;
    m_ac(k) = f_act  * m_out_Al;
end

% Design point index
[~, k_des] = min(abs(b_tips - (s-y_hinge)));
b_codeE    = s - Span_taxi/2;              % Code E limit on b_tip axis

% Figure 3 - Wing mass vs folding tip length
figure('Name','Wing Mass vs Folding Tip Length','Position',[140 60 1050 560]);
hold on; grid on; box on;

plot(b_tips, m_Al/1e3, '-',  'Color',C.blue,   'LineWidth',2.5, ...
    'DisplayName','Al 7010-T7451');
plot(b_tips, m_CF/1e3, '-',  'Color',C.orange, 'LineWidth',2.5, ...
    'DisplayName','CFRP quasi-iso');
yline(m_total_Al/1e3, '--', 'Color',C.blue,   'LineWidth',1.2, ...
    'Label','Al fixed-wing', 'LabelHorizontalAlignment','left');
yline(m_total_CF/1e3, '--', 'Color',C.orange, 'LineWidth',1.2, ...
    'Label','CF fixed-wing', 'LabelHorizontalAlignment','left');

% Code E limit
xline(b_codeE, ':', 'Color',C.red, 'LineWidth',1.8, ...
    'Label','Code E limit', 'LabelHorizontalAlignment','right');

% Design point markers
plot(b_tips(k_des), m_Al(k_des)/1e3, 'o', ...
    'Color',C.blue,   'MarkerFaceColor',C.blue,   'MarkerSize',10, ...
    'DisplayName',sprintf('Design Al %.0f kg', m_Al(k_des)));
plot(b_tips(k_des), m_CF(k_des)/1e3, 'o', ...
    'Color',C.orange, 'MarkerFaceColor',C.orange, 'MarkerSize',10, ...
    'DisplayName',sprintf('Design CF %.0f kg', m_CF(k_des)));

xlabel('Folding tip length  b_{tip}  [m]  (per side)');
ylabel('Wing structural mass  [t]');
title({'Wing Mass vs Folding Tip Length', ...
    sprintf('Al & CFRP | 2.5g | Span %.0f m | MTOM %.0f t', Span, MB_Al.m_frac_MTOM^-1*m_total_Al/1e3)}, ...
    'FontWeight','bold');
legend('Location','best','FontSize',10);

% ── Figure 4 - Fold penalty breakdown ────────────────────────────────
figure('Name','Fold Mechanism Weight Penalty','Position',[180 60 1050 560]);
hold on; grid on; box on;

m_total_pen = m_ha + m_lk + m_ac + m_rb;

area(b_tips, m_ha/1e3,             'FaceColor',C.blue,   'FaceAlpha',0.7, 'EdgeColor','none','DisplayName','Hinge assembly');
area(b_tips, (m_ha+m_lk)/1e3,     'FaceColor',C.orange, 'FaceAlpha',0.7, 'EdgeColor','none','DisplayName','Lock mechanism');
area(b_tips, (m_ha+m_lk+m_ac)/1e3,'FaceColor',C.green,  'FaceAlpha',0.7, 'EdgeColor','none','DisplayName','Fold actuator');
area(b_tips, m_total_pen/1e3,      'FaceColor',C.red,    'FaceAlpha',0.5, 'EdgeColor','none','DisplayName','Hinge rib');
plot(b_tips, m_total_pen/1e3, '-k','LineWidth',2.2,'DisplayName','Total penalty');

xline(b_codeE, ':', 'Color',C.red, 'LineWidth',1.8, ...
    'Label','Code E limit', 'LabelHorizontalAlignment','right');
plot(b_tips(k_des), m_total_pen(k_des)/1e3, 'ok', ...
    'MarkerFaceColor',C.blue, 'MarkerSize',10, ...
    'DisplayName',sprintf('Design point +%.0f kg', m_total_pen(k_des)));

xlabel('Folding tip length  b_{tip}  [m]  (per side)');
ylabel('Fold mechanism mass penalty  [t]');
title({'Folding Wingtip - Mechanism Weight Penalty Breakdown', ...
    sprintf('Al 7010-T7451 | Design b_{tip} = %.2f m | Total penalty %.0f kg', ...
    b_tips(k_des), m_total_pen(k_des))}, 'FontWeight','bold');
legend('Location','northwest','FontSize',10);

end
