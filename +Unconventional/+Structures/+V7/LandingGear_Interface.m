%% LandingGear_Interface — Wing Structures outputs for Role 8

% Wing CG for longitudinal gear placement ───────────────────────────
y_CG_wing   = WS.y_CG_wing;       % m   spanwise wing CG from CL

% Wing mass for MTOW-based wheel sizing ─────────────────────────────
m_wing      = WS.m_wing_total;    % kg  input to total MTOM calculation

% Engine geometry for clearance check ───────────────────────────────
y_engine    = WS.y_engine;        % m   engine spanwise position
Span        = WS.Span;            % m   full flight span

fprintf('\n--- Wing Structures → Landing Gear handoff ---\n');
fprintf('  Wing structural mass         %8.0f kg\n', m_wing);
fprintf('  Wing spanwise CG             %8.2f m from CL\n', y_CG_wing);
fprintf('  Engine spanwise position     %8.2f m from CL\n', y_engine);
fprintf('  Full wingspan                %8.2f m\n', Span);
fprintf('\n  NOTE: Landing load case not yet in Wing Structures model.\n');
fprintf('  Collaborate per handbook Sprint 3 to add landing reaction factor.\n');
