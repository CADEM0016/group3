function [] = AeroConvergence(obj)
%AEROCONVERGENCE Summary of this function goes here

% Inputs
rho    = 1.225; % Air density [kg/m^3]
Vstall = 70; % Stall speed [m/s]
Vto    = 80; % Takeoff speed [m/s]
Vland  = 75; % Landing speed [m/s]

CLmax  = 2.2; % Initial CLmax estimate
takeoff_flap = 35; % takeoff flap deflection [degrees]

tol = 0.1;
error = 1;
iter = 0;


takeoff_alt = 50; % Roughly sea level plus some

% Initial guess
S = obj.WingArea;

while error > tol

    iter = iter + 1;
    S_old = S;

    % Stall constraint -> Wing area
    WS = 0.5 * rho * Vstall^2 * CLmax;
    S  = obj.MTOM / WS;
    
    AR = (obj.WingSpan^2)/(S);

    c_root = (2*S)/(b*(1+taper));
    c_tip  = taper*c_root;

    % Run AVL (get lift slope and span efficiency)
    [CL_alpha, alpha0, e] = runAVL(b,c_root,c_tip);
    
    % % Workout the takeoff mach
    % mach = Unconventional.aerodynamics.mach_from_velocity(vto,takeoff_alt);

    % Drag build-up
    dragBuildup(obj,takeoff_alt,takeoff_flap); % Assigns to ADP.AeroPolar.CD0

    % Construct drag polar
    k = 1/(pi*AR*ADP.e);

    % Required CL at takeoff
    CL_to = obj.MTOM / (0.5*rho*Vto^2*S);

    % Required AoA
    alpha_to = CL_to/CL_alpha + alpha0;

    % Drag at takeoff
    CD_to = ADP.AeroPolar.CD0 + k*CL_to^2;

    % Drag force
    D_to = 0.5*rho*Vto^2*S*CD_to;

    % Aerodynamic sanity checks
    LD_to = CL_to/CD_to;

    if alpha_to > 12
        warning("AoA too high")
    end

    if LD_to < 8
        warning("Drag too high")
    end

    % Convergence check
    error = abs(S - S_old)/S;

    fprintf("Iter %d | Wing Area %.2f m^2 | AoA %.2f deg | L/D %.2f\n", ...
            iter, S, alpha_to, LD_to)

end

% Output
fprintf("\nConverged Geometry\n")
fprintf("Wing area: %.2f m^2\n",S)
fprintf("Span: %.2f m\n",b)
fprintf("Root chord: %.2f m\n",c_root)
fprintf("Tip chord: %.2f m\n",c_tip)
fprintf("CL_alpha: %.3f /deg\n",CL_alpha)
fprintf("Span efficiency: %.3f\n",e)


end