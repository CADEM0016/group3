%% =========================================================
% ExampleUnconventionalMachSweep_oscar_FIXED.m
%
% Mach sweep to compare:
%   - Block Fuel
%   - Cruise Drag Coefficient
%   - Cruise TSFC
% and show how they relate.
%
% FIXED:
%   ADP.RootChord does not exist on class Unconventional.ADP,
%   so a baseline lookup-table root chord is used instead.
%% =========================================================

clear
clc
close all

%% ---------------------------------------------------------
% Load AVL lookup table
%% ---------------------------------------------------------
baseDir = fileparts(matlab.desktop.editor.getActiveFilename);
lookupPath = fullfile(baseDir, '../+Unconventional/+lookup/+aerodynamics/cruise_lookup_table.mat');
S = load(lookupPath);
R = S.Results;

%% ---------------------------------------------------------
% Baseline aircraft definition
%% ---------------------------------------------------------
ADP0 = Unconventional.ADP();
ADP0.TLAR = cast.TLAR.Unconventional();
ADP0.TLAR.M_c = 0.84;

% --------------------- set Unconventional specific parameters ---------------------
ADP0.FuselageLength = 65;                      % m
ADP0.KinkPos = 10;                            % m
ADP0.CabinRadius = 6.3;                       % m
ADP0.CabinLength = 50;                        % m
ADP0.CockpitLength = 5;                       % m
ADP0.WingPos = 0.44 * ADP0.FuselageLength;    % m
ADP0.V_HT = 0.97;
ADP0.V_VT = 0.072;
ADP0.HtpPos = 0.85 * ADP0.FuselageLength;     % m
ADP0.VtpPos = 0.82 * ADP0.FuselageLength;     % m

ADP0.WingArea = 750;                          % m^2 guess
ADP0.Span = 74;                               % m

% -------------------------- class-I estimates ---------------------------
ADP0.MTOM = 490000;                           % kg
ADP0.Mf_Fuel = 0.32;
ADP0.Mf_res  = 0.03;
ADP0.Mf_Ldg  = 0.62;
ADP0.Mf_TOC  = 0.975;

%% ---------------------------------------------------------
% Sweep setup
%% ---------------------------------------------------------
M_vec = 0.60:0.02:0.90;
altitude = 11000;                 % m
g = 9.81;
gamma_air = 1.4;
R_air = 287.05;

% IMPORTANT FIX:
% ADP does not have a RootChord property, so use the baseline
% root chord that your AVL lookup table was built around.
rootChordBaseline = 15.0;         % m
spanBaseline      = 74.0;         % m

% Optional fixed lookup choices if your table is only built for one geometry
% You can change these if needed:
useFixedLookupGeometry = true;

%% ---------------------------------------------------------
% Preallocate results
%% ---------------------------------------------------------
BlockFuel   = nan(size(M_vec));   % kg
TripFuel    = nan(size(M_vec));   % kg
ResFuel     = nan(size(M_vec));   % kg
MissionTime = nan(size(M_vec));   % s or whatever your function returns
MfTOC_used  = nan(size(M_vec));   % -

CL_cruise   = nan(size(M_vec));
CD_cruise   = nan(size(M_vec));
DragReq     = nan(size(M_vec));   % N
TSFC        = nan(size(M_vec));   % model units
mtoms       = nan(size(M_vec));   % kg
fuelMass    = nan(size(M_vec));   % kg

%% ---------------------------------------------------------
% Main sweep
%% ---------------------------------------------------------
for i = 1:length(M_vec)

    M = M_vec(i);

    fprintf('Running case %d / %d, Mach = %.2f\n', i, length(M_vec), M);

    % -----------------------------------------------------
    % Resize aircraft at this Mach
    % -----------------------------------------------------
    ADP = ADP0;
    ADP.TLAR.M_c = M;

    ADP = Unconventional.Size(ADP);

    mtoms(i) = ADP.MTOM;
    fuelMass(i) = ADP.Mf_Fuel * ADP.MTOM;

    % -----------------------------------------------------
    % Mission analysis
    % -----------------------------------------------------
    [BlockFuel(i), TripFuel(i), ResFuel(i), MfTOC_used(i), MissionTime(i)] = ...
        Unconventional.MissionAnalysis_oscar(ADP, ADP.TLAR.RangeDes, ADP.MTOM);

    % -----------------------------------------------------
    % Representative cruise weight
    % -----------------------------------------------------
    W_cruise = MfTOC_used(i) * ADP.MTOM * g;    % N

    % Use the current sized wing area if available
    Sref = ADP.WingArea;

    % -----------------------------------------------------
    % ISA atmosphere at cruise altitude
    % -----------------------------------------------------
    [T, a, p, rho] = isa_atmosphere(altitude);

    V = M * a;
    q = 0.5 * rho * V^2;

    % Required CL for steady level cruise
    CLreq = W_cruise / (q * Sref);
    CL_cruise(i) = CLreq;

    % -----------------------------------------------------
    % Match lookup table condition
    % -----------------------------------------------------
    if useFixedLookupGeometry
        spanMatch = nearestValue(S.spanVec, spanBaseline);
        rootChordMatch = nearestValue(S.rootChordVec, rootChordBaseline);
    else
        % If later you discover the correct ADP class property for chord,
        % replace rootChordBaseline with that property.
        spanMatch = nearestValue(S.spanVec, ADP.Span);
        rootChordMatch = nearestValue(S.rootChordVec, rootChordBaseline);
    end

    machMatch = nearestValue(S.machVec, M);
    altMatch  = nearestValue(S.altVec, altitude);

    idx = [R.WingSpan]  == spanMatch & ...
          [R.Mach]      == machMatch & ...
          [R.RootChord] == rootChordMatch & ...
          [R.Altitude]  == altMatch;

    if ~any(idx)
        warning('No lookup data found for Mach %.2f. Skipping aero interpolation.', M);
        continue
    end

    CLtab  = [R(idx).CL];
    CDtab  = [R(idx).CD];
    CD0tab = [R(idx).CD0];

    % Remove NaNs if any
    valid = ~(isnan(CLtab) | isnan(CDtab) | isnan(CD0tab));
    CLtab  = CLtab(valid);
    CDtab  = CDtab(valid);
    CD0tab = CD0tab(valid);

    if isempty(CLtab)
        warning('Lookup data empty after NaN cleanup for Mach %.2f.', M);
        continue
    end

    % Sort by CL for interpolation
    [CLtab, ord] = sort(CLtab);
    CDtab  = CDtab(ord);
    CD0tab = CD0tab(ord);

    % Total cruise CD
    CDtot = interp1(CLtab, CDtab + CD0tab, CLreq, 'linear', 'extrap');

    CD_cruise(i) = CDtot;

    % Required drag = required thrust in steady level cruise
    DragReq(i) = q * Sref * CDtot;

    % -----------------------------------------------------
    % Simple BPR-derived TSFC model
    % -----------------------------------------------------
    % Same style as your existing lower script
    theta = sqrt(T / 288.15);

    % Example based on "UltraFan-style" average BPR
    BPR = 13.5;

    Acoef = 19 * exp(-0.12 * BPR) * 1e-6;
    SFCc  = 25 * exp(-0.05 * BPR) * 1e-6;

    TSFC(i) = (SFCc / theta - Acoef) / M;

end

%% ---------------------------------------------------------
% Plot 1: Block Fuel vs Mach
%% ---------------------------------------------------------
figure('Color','w')
plot(M_vec, BlockFuel/1000, '-o', 'LineWidth', 2)
grid on
box on
xlabel('Cruise Mach', 'FontSize', 16)
ylabel('Block Fuel [tonnes]', 'FontSize', 16)
title('Block Fuel vs Cruise Mach', 'FontSize', 18)
set(gca, 'FontSize', 14)

%% ---------------------------------------------------------
% Plot 2: Cruise CD vs Mach
%% ---------------------------------------------------------
figure('Color','w')
plot(M_vec, CD_cruise, '-s', 'LineWidth', 2)
grid on
box on
xlabel('Cruise Mach', 'FontSize', 16)
ylabel('Cruise C_D [-]', 'FontSize', 16)
title('Cruise Drag Coefficient vs Cruise Mach', 'FontSize', 18)
set(gca, 'FontSize', 14)

%% ---------------------------------------------------------
% Plot 3: TSFC vs Mach
%% ---------------------------------------------------------
figure('Color','w')
plot(M_vec, TSFC, '-d', 'LineWidth', 2)
grid on
box on
xlabel('Cruise Mach', 'FontSize', 16)
ylabel('TSFC', 'FontSize', 16)
title('TSFC vs Cruise Mach', 'FontSize', 18)
set(gca, 'FontSize', 14)

%% ---------------------------------------------------------
% Plot 4: Block Fuel vs Cruise CD
%% ---------------------------------------------------------
figure('Color','w')
plot(CD_cruise, BlockFuel/1000, '-o', 'LineWidth', 2)
grid on
box on
xlabel('Cruise C_D [-]', 'FontSize', 16)
ylabel('Block Fuel [tonnes]', 'FontSize', 16)
title('Block Fuel vs Cruise Drag Coefficient', 'FontSize', 18)
set(gca, 'FontSize', 14)

%% ---------------------------------------------------------
% Plot 5: Block Fuel vs TSFC
%% ---------------------------------------------------------
figure('Color','w')
plot(TSFC, BlockFuel/1000, '-o', 'LineWidth', 2)
grid on
box on
xlabel('TSFC', 'FontSize', 16)
ylabel('Block Fuel [tonnes]', 'FontSize', 16)
title('Block Fuel vs TSFC', 'FontSize', 18)
set(gca, 'FontSize', 14)

%% ---------------------------------------------------------
% Combined report-style figure
%% ---------------------------------------------------------
figure('Color','w')
tiledlayout(3,1)

nexttile
plot(M_vec, BlockFuel/1000, '-o', 'LineWidth', 2)
grid on
box on
xlabel('Cruise Mach')
ylabel('Block Fuel [t]')
title('Block Fuel vs Mach')

nexttile
plot(M_vec, CD_cruise, '-s', 'LineWidth', 2)
grid on
box on
xlabel('Cruise Mach')
ylabel('C_D')
title('Cruise Drag Coefficient vs Mach')

nexttile
plot(M_vec, TSFC, '-d', 'LineWidth', 2)
grid on
box on
xlabel('Cruise Mach')
ylabel('TSFC')
title('TSFC vs Mach')

%% ---------------------------------------------------------
% Console summary
%% ---------------------------------------------------------
disp(' ')
disp('================ SUMMARY ================')
disp(table(M_vec(:), mtoms(:), BlockFuel(:), CD_cruise(:), TSFC(:), ...
    'VariableNames', {'Mach','MTOM_kg','BlockFuel_kg','CD_cruise','TSFC'}))

%% =========================================================
% Helper functions
%% =========================================================

function val = nearestValue(vec, x)
    [~, idx] = min(abs(vec - x));
    val = vec(idx);
end

function [T, a, p, rho] = isa_atmosphere(h)
    % ISA up to lower stratosphere
    % h in m

    g0 = 9.80665;
    R  = 287.05;
    gamma_air = 1.4;

    T0 = 288.15;
    p0 = 101325;
    L  = -0.0065;

    if h <= 11000
        T = T0 + L*h;
        p = p0 * (T/T0)^(-g0/(L*R));
    else
        T = 216.65;
        p11 = p0 * (216.65/T0)^(-g0/(L*R));
        p = p11 * exp(-g0*(h-11000)/(R*T));
    end

    rho = p / (R*T);
    a = sqrt(gamma_air * R * T);
end