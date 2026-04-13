function InteractiveWingFold(L)

clc; close all;

%% ------------------ LOAD V6 MODEL ------------------
addpath('Structures');

adp  = Unconventional.ADP();
tlar = cast.TLAR.Unconventional();
loc  = Unconventional.Structures.V6.AircraftParams();

% Run full geometry + loads pipeline
Unconventional.Structures.V6.WingGeometry(adp, tlar, loc);
G = Unconventional.Structures.V6.WingGeometry(adp, tlar, loc);

% Try to get real loads (depends on your structure setup)
try
    L = Unconventional.Structures.V6.LoadDistribution(adp, tlar, loc, G);
    useRealLoads = true;
catch
    warning('Real V6 loads not found -> using fallback elliptical distribution');
    useRealLoads = false;
end

b_total = adp.Span;
y_hinge = 32.5;

%% ------------------ UI FIGURE ------------------
f = figure('Name','Interactive Folding Wing + REAL Loads','Color','w','Position',[100 100 1100 600]);

uicontrol('Style','text','Position',[20 60 120 20],'String','Fold Angle');
s_angle = uicontrol('Style','slider','Min',0,'Max',90,'Value',25,...
    'Position',[20 40 150 20],'Callback',@updatePlot);

ax = axes('Parent',f,'Position',[0.25 0.1 0.7 0.8]);

updatePlot();

%% ------------------ UPDATE FUNCTION ------------------
function updatePlot(~,~)

    cla(ax);

    theta = deg2rad(s_angle.Value);

    %% ---------- GEOMETRY ----------
    y_full = G.y;
    chord_full = G.chord;
    x_le_full = G.x_le;

    %% ---------- LOADS ----------
    if useRealLoads && isfield(L,'Lift')
        lift_dist = L.Lift;  % REAL loads
    elseif useRealLoads && isfield(L,'q')
        lift_dist = L.q;     % distributed load
    elseif useRealLoads && isfield(L,'Load')
        lift_dist = L.Load;  % alternative naming
    elseif useRealLoads && isfield(L,'dLdy')
        lift_dist = L.dLdy;  % another possible naming
    else
        % fallback
        span_half = max(y_full);
        lift_dist = sqrt(1 - (y_full/span_half).^2);
    end

    % Normalize for visualization
    lift_dist = lift_dist ./ max(abs(lift_dist));

    %% ---------- SPLIT ----------
    mask_inner = y_full <= y_hinge;
    mask_fold  = y_full > y_hinge;

    y_inner = y_full(mask_inner);
    y_fold  = y_full(mask_fold);

    chord_inner = chord_full(mask_inner);
    chord_fold  = chord_full(mask_fold);

    x_inner_le = x_le_full(mask_inner);
    x_fold_le  = x_le_full(mask_fold);

    lift_inner = lift_dist(mask_inner);
    lift_fold  = lift_dist(mask_fold);

    %% ---------- SURFACES ----------
    [X1,Y1] = meshgrid([0 1], y_inner);
    X1 = X1 .* chord_inner' + x_inner_le';
    Z1 = zeros(size(X1));

    [X2,Y2] = meshgrid([0 1], y_fold);
    X2 = X2 .* chord_fold' + x_fold_le';
    Z2 = zeros(size(X2));

    %% ---------- ROTATION ----------
    R = [cos(theta) 0 sin(theta);
         0          1 0;
        -sin(theta) 0 cos(theta)];

    x_hinge = x_inner_le(end);

    for i = 1:numel(X2)
        vec = R * [X2(i)-x_hinge; Y2(i)-y_hinge; Z2(i)];
        X2(i) = vec(1) + x_hinge;
        Y2(i) = vec(2) + y_hinge;
        Z2(i) = vec(3);
    end

    %% ---------- COLOR MAP ----------
    C1 = repmat(lift_inner',1,2);
    C2 = repmat(lift_fold',1,2);

    surf(ax, X1, Y1, Z1, C1, 'EdgeColor','none'); hold(ax,'on');
    surf(ax, X2, Y2, Z2, C2, 'EdgeColor','none');

    % Mirror
    surf(ax, X1, -Y1, Z1, C1, 'EdgeColor','none');
    surf(ax, X2, -Y2, Z2, C2, 'EdgeColor','none');

    %% ---------- LOAD VECTORS ----------
    scale = 5;

    % Inner
    for i = 1:length(y_inner)
        quiver3(ax, x_inner_le(i), y_inner(i), 0, ...
            0, 0, lift_inner(i)*scale, 'k');
    end

    % Fold (rotated)
    for i = 1:length(y_fold)
        pt = R * [x_fold_le(i)-x_hinge; y_fold(i)-y_hinge; 0];
        x = pt(1) + x_hinge;
        y = pt(2) + y_hinge;
        z = pt(3);

        vec = R * [0;0;lift_fold(i)*scale];

        quiver3(ax, x, y, z, vec(1), vec(2), vec(3), 'r');
    end

    %% ---------- VISUAL ----------
    colormap(ax, jet);
    colorbar;

    xlabel('x'); ylabel('y'); zlabel('z');
    title(sprintf('Fold Angle: %.1f deg | REAL Load Distribution', rad2deg(theta)));

    axis equal; grid on;
    view(35,25);
    camlight; lighting gouraud;

end

end
