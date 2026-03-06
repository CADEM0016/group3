%% =========================================================================
%  run_all.m  —  +Structures package
%  MASTER RUN SCRIPT — Wing Structural Analysis, MDO Development Phase
%  CADEM0016 Group Design Project, University of Bristol 2025-26
%
%  HOW TO RUN:
%    1. In MATLAB, navigate to:
%         GitHub/group3/Unconventional/
%    2. Run this file:
%         run('+Structures/run_all.m')
%       OR add the Unconventional folder to your path and type:
%         Structures.run_all   % if converted to a function later
%
%  WHAT THIS SCRIPT DOES (in order):
%    Step 0 — Load aircraft parameters (single source of truth)
%    Step 1 — Class I/II empirical mass (Raymer + Torenbeek)
%    Step 2 — Wing geometry at all spanwise stations
%    Step 3 — Load distribution (three CS-25 load cases)
%    Step 4 — SMT integration (shear, bending, torque)
%    Step 5 — Wingbox sizing (skin, spar caps, webs)
%    Step 6 — Stiffness distributions EI(y) and GJ(y)
%    Step 7 — Mass buildup from sized cross-sections
%    Step 8 — Fidelity comparison (I/II vs II.5)
%    Step 9 — All plots (loads, SMT, wingbox, stiffness)
%    Step 10 — Sensitivity studies (AR, MTOM, wingspan, material)
%
%  ALL FUNCTIONS are inside +Structures — ZERO external dependencies.
%  This script runs completely standalone from the rest of the repo.
%
%  OUTPUT VARIABLES (available in workspace after running):
%    p      — all aircraft parameters
%    G      — wing geometry at all stations
%    L_25g, L_1g, L_n1g  — load distributions for three cases
%    S_25g, S_1g, S_n1g  — SMT results
%    W_25g                — wingbox sizing (critical case)
%    D_25g                — stiffness distributions
%    MB_25g               — mass buildup
%    E_Al, E_CF           — empirical mass estimates
% =========================================================================

clear; clc; close all;

% ---- Set path so MATLAB finds the +Structures package
% Run from: GitHub/group3/Unconventional/
% If already in the right folder this line does nothing harmful
this_dir = fileparts(mfilename('fullpath'));
parent   = fileparts(this_dir);   % = GitHub/group3/Unconventional
addpath(parent);

fprintf('=========================================================\n');
fprintf('  Wing Structural Analysis — MDO Development Phase\n');
fprintf('  CADEM0016 GDP, University of Bristol 2025-26\n');
fprintf('  Unconventional config: wide-body + folding wingtips\n');
fprintf('=========================================================\n\n');

% =========================================================================
%  STEP 0 — AIRCRAFT PARAMETERS
% =========================================================================
fprintf('STEP 0: Loading aircraft parameters ...\n');
p = StandAlone.Aircraftparams();

fprintf('  MTOM:        %.0f kg  (%.1f t)\n', p.MTOM, p.MTOM/1e3);
fprintf('  Span:        %.1f m  (flight)\n',  p.Span);
fprintf('  Taxi span:   %.1f m  (Code E limit)\n', p.Span_taxi);
fprintf('  Fold hinge:  y = %.1f m\n', p.y_hinge);
fprintf('  Wing area:   %.1f m^2\n', p.WingArea);
fprintf('  AR:          %.2f\n', p.AR);
fprintf('  Fuel mass:   %.0f kg  (%.1f t)\n', p.M_fuel, p.M_fuel/1e3);

% =========================================================================
%  STEP 1 — CLASS I/II EMPIRICAL MASS
% =========================================================================
fprintf('\n=========================================================\n');
fprintf('STEP 1: Class I/II Empirical Wing Mass\n');
fprintf('=========================================================\n');

E_Al = Structures.EmpiricalMass(p, 'Al');
E_CF = Structures.EmpiricalMass(p, 'CF');

fprintf('\n  Aluminium:  %.0f kg  (%.1f%% MTOM)\n', E_Al.m_total, E_Al.m_frac_MTOM*100);
fprintf('  CFRP:       %.0f kg  (%.1f%% MTOM)\n',   E_CF.m_total, E_CF.m_frac_MTOM*100);
fprintf('  CFRP saves: %.0f kg  (%.1f%%)\n', ...
        E_Al.m_total - E_CF.m_total, ...
        (E_Al.m_total - E_CF.m_total)/E_Al.m_total*100);

% =========================================================================
%  STEP 2 — WING GEOMETRY
% =========================================================================
fprintf('\n=========================================================\n');
fprintf('STEP 2: Wing Geometry\n');
fprintf('=========================================================\n');

G = Structures.WingGeometry(p);

% =========================================================================
%  STEP 3 — LOAD DISTRIBUTIONS (all three CS-25 cases)
% =========================================================================
fprintf('\n=========================================================\n');
fprintf('STEP 3: Load Distributions\n');
fprintf('=========================================================\n');

L_25g = Structures.LoadDistribution(p, G, '2.5g');
L_1g  = Structures.LoadDistribution(p, G, '1g');
L_n1g = Structures.LoadDistribution(p, G, 'neg1g');

% =========================================================================
%  STEP 4 — SMT INTEGRATION
% =========================================================================
fprintf('\n=========================================================\n');
fprintf('STEP 4: SMT Integration\n');
fprintf('=========================================================\n');

S_25g = Structures.SMT(p, G, L_25g);
S_1g  = Structures.SMT(p, G, L_1g);
S_n1g = Structures.SMT(p, G, L_n1g);

fprintf('\n  Root bending moments:\n');
fprintf('    2.5g:   %.2f MNm\n', abs(S_25g.M_root)/1e6);
fprintf('    1g:     %.2f MNm\n', abs(S_1g.M_root)/1e6);
fprintf('    neg1g:  %.2f MNm\n', abs(S_n1g.M_root)/1e6);
fprintf('  → CRITICAL CASE: 2.5g (highest bending moment)\n');

% =========================================================================
%  STEP 5 — WINGBOX SIZING
%  Size for 2.5g (critical) — also run for neg1g for comparison
% =========================================================================
fprintf('\n=========================================================\n');
fprintf('STEP 5: Wingbox Sizing\n');
fprintf('=========================================================\n');

W_25g  = Structures.WingboxSizing(p, G, S_25g,  'Al');
W_n1g  = Structures.WingboxSizing(p, G, S_n1g,  'Al');
W_25g_CF = Structures.WingboxSizing(p, G, S_25g, 'CF');

% =========================================================================
%  STEP 6 — STIFFNESS DISTRIBUTIONS EI(y) AND GJ(y)
% =========================================================================
fprintf('\n=========================================================\n');
fprintf('STEP 6: Stiffness Distributions EI(y) and GJ(y)\n');
fprintf('=========================================================\n');

D_25g    = Structures.StiffnessDistribution(p, G, W_25g);
D_25g_CF = Structures.StiffnessDistribution(p, G, W_25g_CF);

% =========================================================================
%  STEP 7 — MASS BUILDUP
% =========================================================================
fprintf('\n=========================================================\n');
fprintf('STEP 7: Mass Buildup\n');
fprintf('=========================================================\n');

MB_25g    = Structures.MassBuildup(p, G, W_25g);
MB_25g_CF = Structures.MassBuildup(p, G, W_25g_CF);

% =========================================================================
%  STEP 8 — FIDELITY COMPARISON TABLE
% =========================================================================
fprintf('\n=========================================================\n');
fprintf('STEP 8: Fidelity Comparison — Class I/II vs Class II.5\n');
fprintf('=========================================================\n');

fprintf('\n');
fprintf('  +-------------------------------+----------+-----------+\n');
fprintf('  | Method                        | Mass [kg]| %% MTOM   |\n');
fprintf('  +-------------------------------+----------+-----------+\n');
fprintf('  | Class I/II   Raymer           | %8.0f | %6.2f%%   |\n', E_Al.m_raymer,      E_Al.m_raymer/p.MTOM*100);
fprintf('  | Class I/II   Torenbeek        | %8.0f | %6.2f%%   |\n', E_Al.m_torenbeek,   E_Al.m_torenbeek/p.MTOM*100);
fprintf('  | Class I/II   Avg + hinge (Al) | %8.0f | %6.2f%%   |\n', E_Al.m_total,       E_Al.m_frac_MTOM*100);
fprintf('  | Class I/II   Avg + hinge (CF) | %8.0f | %6.2f%%   |\n', E_CF.m_total,       E_CF.m_frac_MTOM*100);
fprintf('  +-------------------------------+----------+-----------+\n');
fprintf('  | Class II.5   2.5g (Al)        | %8.0f | %6.2f%%   |\n', MB_25g.m_total,     MB_25g.m_frac_MTOM*100);
fprintf('  | Class II.5   2.5g (CFRP)      | %8.0f | %6.2f%%   |\n', MB_25g_CF.m_total,  MB_25g_CF.m_total/p.MTOM*100);
fprintf('  +-------------------------------+----------+-----------+\n');
fprintf('  | B777F reference               |    34000 |   9.76%%  |\n');
fprintf('  +-------------------------------+----------+-----------+\n');
fprintf('\n  Diff I/II vs II.5 (Al): %+.0f kg  (%+.1f%%)\n', ...
        MB_25g.m_total - E_Al.m_total, ...
        (MB_25g.m_total - E_Al.m_total)/E_Al.m_total*100);

% =========================================================================
%  STEP 9 — PLOTS
% =========================================================================
fprintf('\n=========================================================\n');
fprintf('STEP 9: Generating Plots\n');
fprintf('=========================================================\n');

% Main analysis plots for the critical load case (2.5g, Al)
Structures.Plots(p, G, L_25g, S_25g, W_25g, D_25g, MB_25g);

% =========================================================================
%  STEP 10 — SENSITIVITY STUDIES
% =========================================================================
fprintf('\n=========================================================\n');
fprintf('STEP 10: Sensitivity Studies\n');
fprintf('=========================================================\n');

Structures.SensitivityStudy(p);

% =========================================================================
%  FINAL SUMMARY
% =========================================================================
fprintf('\n=========================================================\n');
fprintf('  FINAL SUMMARY — DESIGN POINT\n');
fprintf('  Unconventional: wide-body + folding wingtip\n');
fprintf('  Span=%.1fm  MTOM=%.0ft  AR=%.1f  Material=Al 7075-T6\n', ...
        p.Span, p.MTOM/1e3, p.AR);
fprintf('=========================================================\n');
fprintf('  Class I/II wing mass:      %7.0f kg  (%.1f%% MTOM)\n', E_Al.m_total, E_Al.m_frac_MTOM*100);
fprintf('  Class II.5 wing mass:      %7.0f kg  (%.1f%% MTOM)\n', MB_25g.m_total, MB_25g.m_frac_MTOM*100);
fprintf('  CFRP saving (vs Al II.5):  %7.0f kg  (%.1f%%)\n', ...
        MB_25g.m_total - MB_25g_CF.m_total, ...
        (MB_25g.m_total - MB_25g_CF.m_total)/MB_25g.m_total*100);
fprintf('  EI at root:                %.4e Nm^2\n', D_25g.EI_root);
fprintf('  GJ at root:                %.4e Nm^2\n', D_25g.GJ_root);
fprintf('  EI at fold hinge:          %.4e Nm^2\n', D_25g.EI_hinge);
fprintf('  GJ at fold hinge:          %.4e Nm^2\n', D_25g.GJ_hinge);
fprintf('=========================================================\n\n');