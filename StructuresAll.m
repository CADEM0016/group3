%% =========================================================================
% Unconventional wide-body freighter with folding wingtips
%
%  Steps:
%    0  — Aircraft parameters (single source of truth)
%    1  — Class I/II empirical mass (Raymer + Torenbeek + USAF)
%    2  — Wing geometry (Snorri Ch9: chord, MAC, Oswald e)
%    3  — Load distributions (CS-25: 2.5g, 1g, -1g)
%    4  — SMT integration (shear, bending, torque)
%    5  — Wingbox sizing (bending + Bredt-Batho torsion + buckling)
%    6  — Stiffness distributions EI(y) and GJ(y)
%    7  — Mass buildup (direct: rho*V, with CG and inertia)
%    8  — Fidelity comparison table
%    9  — Plots
%   10  — Sensitivity studies
% =========================================================================

clear;  clc;  close all;

this_dir = fileparts(mfilename('fullpath'));
parent   = fileparts(this_dir);
addpath(parent);

fprintf('=========================================================\n');
fprintf('  Wing Structural Analysis — CADEM0016 Group 3\n');
fprintf('  Unconventional: wide-body freighter + folding wingtips\n');
fprintf('=========================================================\n\n');

% =========================================================================
%  STEP 0 — PARAMETERS
% =========================================================================
fprintf('STEP 0: Aircraft parameters\n');
p = Unconventional.Structures.AircraftParams();

fprintf('  MTOM:        %.0f kg  (%.1f t)\n',  p.MTOM, p.MTOM/1e3);
fprintf('  OEM:         %.0f kg\n',              p.OEM);
fprintf('  Payload:     %.0f kg\n',              p.Payload);
fprintf('  Useful load: %.0f kg  (MTOM - OEM)\n', p.W_useful);
fprintf('  Fuel:        %.0f kg  (%.0f%% MTOM)\n', p.M_fuel, p.Mf_fuel*100);
fprintf('  Reserve:     %.0f kg  (%.0f%% of fuel)\n', p.M_fuel_res, p.Mf_res*100);
fprintf('  Span:        %.1f m  flight  /  %.1f m  taxi (Code E)\n', p.Span, p.Span_taxi);
fprintf('  Fold hinge:  y = %.1f m from CL\n', p.y_hinge);
fprintf('  AR:          %.2f\n', p.AR);
fprintf('  MAC:         %.2f m\n', p.MAC);
fprintf('  q_cruise:    %.0f Pa  (M=%.2f at %.0f m)\n', p.q_cruise, p.M_cruise, p.Alt_cruise);

% =========================================================================
%  STEP 1 — CLASS I/II EMPIRICAL MASS
% =========================================================================
fprintf('\n=========================================================\n');
fprintf('STEP 1: Class I/II Empirical Wing Mass\n');
fprintf('=========================================================\n');

E_Al = Unconventional.Structures.EmpiricalMass(p, 'Al');
E_CF = Unconventional.Structures.EmpiricalMass(p, 'CF');

fprintf('\n  Aluminium:   %.0f kg  (%.1f%% MTOM)\n', E_Al.m_total, E_Al.m_frac_MTOM*100);
fprintf('  CFRP:        %.0f kg  (%.1f%% MTOM)\n',  E_CF.m_total, E_CF.m_frac_MTOM*100);
fprintf('  CFRP saving: %.0f kg  (%.1f%%)\n', ...
    E_Al.m_total - E_CF.m_total, (E_Al.m_total - E_CF.m_total)/E_Al.m_total*100);

% =========================================================================
%  STEP 2 — WING GEOMETRY
% =========================================================================
fprintf('\n=========================================================\n');
fprintf('STEP 2: Wing Geometry\n');
fprintf('=========================================================\n');

G = Unconventional.Structures.WingGeometry(p);

% =========================================================================
%  STEP 3 — LOAD DISTRIBUTIONS
% =========================================================================
fprintf('\n=========================================================\n');
fprintf('STEP 3: Load Distributions  (CS-25 cases)\n');
fprintf('=========================================================\n');

L_25g = Unconventional.Structures.LoadDistribution(p, G, '2.5g');
L_1g  = Unconventional.Structures.LoadDistribution(p, G, '1g');
L_n1g = Unconventional.Structures.LoadDistribution(p, G, 'neg1g');

fprintf('\n  Total aircraft lift at 2.5g:  %.3f MN  (= n*MTOM*g)\n', L_25g.L_total/1e6);
fprintf('  Snorri M_root check (L*b/8):  %.3f MNm\n', L_25g.M_root_snorri/1e6);
fprintf('  Folding tip lift per panel:   %.3f kN  (Snorri: 2L/b * delta_b)\n', L_25g.L_tip/1e3);

% =========================================================================
%  STEP 4 — SMT INTEGRATION
% =========================================================================
fprintf('\n=========================================================\n');
fprintf('STEP 4: SMT Integration\n');
fprintf('=========================================================\n');

S_25g = Unconventional.Structures.SMT(p, G, L_25g);
S_1g  = Unconventional.Structures.SMT(p, G, L_1g);
S_n1g = Unconventional.Structures.SMT(p, G, L_n1g);

fprintf('\n  Root bending moments:\n');
fprintf('    2.5g:   %.2f MNm\n', abs(S_25g.M_root)/1e6);
fprintf('    1g:     %.2f MNm\n', abs(S_1g.M_root)/1e6);
fprintf('    neg1g:  %.2f MNm\n', abs(S_n1g.M_root)/1e6);
fprintf('  Critical case: 2.5g\n');

% =========================================================================
%  STEP 5 — WINGBOX SIZING
% =========================================================================
fprintf('\n=========================================================\n');
fprintf('STEP 5: Wingbox Sizing\n');
fprintf('         (bending / Bredt-Batho torsion / buckling / Jourawski shear)\n');
fprintf('=========================================================\n');

W_25g    = Unconventional.Structures.WingboxSizing(p, G, S_25g, 'Al');
W_n1g    = Unconventional.Structures.WingboxSizing(p, G, S_n1g, 'Al');
W_25g_CF = Unconventional.Structures.WingboxSizing(p, G, S_25g, 'CF');

% =========================================================================
%  STEP 6 — STIFFNESS DISTRIBUTIONS
% =========================================================================
fprintf('\n=========================================================\n');
fprintf('STEP 6: Stiffness Distributions  EI(y) and GJ(y)\n');
fprintf('=========================================================\n');

D_25g    = Unconventional.Structures.StiffnessDistribution(p, G, W_25g);
D_25g_CF = Unconventional.Structures.StiffnessDistribution(p, G, W_25g_CF);

% =========================================================================
%  STEP 7 — MASS BUILDUP
% =========================================================================
fprintf('\n=========================================================\n');
fprintf('STEP 7: Mass Buildup  (rho*V per component + CG + inertia)\n');
fprintf('=========================================================\n');

MB_25g    = Unconventional.Structures.MassBuildup(p, G, W_25g);
MB_25g_CF = Unconventional.Structures.MassBuildup(p, G, W_25g_CF);

% =========================================================================
%  STEP 8 — FIDELITY COMPARISON TABLE
% =========================================================================
fprintf('\n=========================================================\n');
fprintf('STEP 8: Fidelity Comparison\n');
fprintf('=========================================================\n\n');

fprintf('  +----------------------------------+----------+-----------+\n');
fprintf('  | Method                           | Mass [kg]|  %% MTOM  |\n');
fprintf('  +----------------------------------+----------+-----------+\n');
fprintf('  | Class I/II  Raymer               | %8.0f | %6.2f%%   |\n', E_Al.m_raymer,    E_Al.m_raymer/p.MTOM*100);
fprintf('  | Class I/II  Torenbeek            | %8.0f | %6.2f%%   |\n', E_Al.m_torenbeek, E_Al.m_torenbeek/p.MTOM*100);
fprintf('  | Class I/II  USAF                 | %8.0f | %6.2f%%   |\n', E_Al.m_usaf,      E_Al.m_usaf/p.MTOM*100);
fprintf('  | Class I/II  3-method avg  (Al)   | %8.0f | %6.2f%%   |\n', E_Al.m_total,     E_Al.m_frac_MTOM*100);
fprintf('  | Class I/II  3-method avg  (CF)   | %8.0f | %6.2f%%   |\n', E_CF.m_total,     E_CF.m_frac_MTOM*100);
fprintf('  +----------------------------------+----------+-----------+\n');
fprintf('  | Class II.5  2.5g  (Al)           | %8.0f | %6.2f%%   |\n', MB_25g.m_total,   MB_25g.m_frac_MTOM*100);
fprintf('  | Class II.5  2.5g  (CFRP)         | %8.0f | %6.2f%%   |\n', MB_25g_CF.m_total, MB_25g_CF.m_total/p.MTOM*100);
fprintf('  +----------------------------------+----------+-----------+\n');
fprintf('  | B777F reference                  |    34000 |   9.76%%  |\n');
fprintf('  +----------------------------------+----------+-----------+\n');
fprintf('\n  I/II vs II.5 difference (Al): %+.0f kg  (%+.1f%%)\n', ...
    MB_25g.m_total - E_Al.m_total, ...
    (MB_25g.m_total - E_Al.m_total)/E_Al.m_total*100);

% =========================================================================
%  STEP 9 — PLOTS
% =========================================================================
fprintf('\n=========================================================\n');
fprintf('STEP 9: Plots\n');
fprintf('=========================================================\n');

Unconventional.Structures.Plots(p, G, L_25g, S_25g, W_25g, D_25g, MB_25g);

% =========================================================================
%  STEP 10 — SENSITIVITY STUDIES
% =========================================================================
fprintf('\n=========================================================\n');
fprintf('STEP 10: Sensitivity Studies\n');
fprintf('=========================================================\n');

Unconventional.Structures.SensitivityStudy(p);

% =========================================================================
%  FINAL SUMMARY
% =========================================================================
fprintf('\n=========================================================\n');
fprintf('  FINAL SUMMARY — DESIGN POINT\n');
fprintf('  Span=%.1fm  MTOM=%.0ft  AR=%.2f  Al 7075-T6\n', ...
    p.Span, p.MTOM/1e3, p.AR);
fprintf('=========================================================\n');
fprintf('  Class I/II wing mass:        %7.0f kg  (%.2f%% MTOM)\n', E_Al.m_total,    E_Al.m_frac_MTOM*100);
fprintf('  Class II.5 wing mass (Al):   %7.0f kg  (%.2f%% MTOM)\n', MB_25g.m_total,  MB_25g.m_frac_MTOM*100);
fprintf('  Class II.5 wing mass (CF):   %7.0f kg  (%.2f%% MTOM)\n', MB_25g_CF.m_total, MB_25g_CF.m_total/p.MTOM*100);
fprintf('  CFRP saving vs Al II.5:      %7.0f kg  (%.1f%%)\n', ...
    MB_25g.m_total - MB_25g_CF.m_total, ...
    (MB_25g.m_total - MB_25g_CF.m_total)/MB_25g.m_total*100);
fprintf('  Wing CG spanwise:            %7.2f m  from centreline\n', MB_25g.y_CG_wing);
fprintf('  Wing roll inertia:           %.4e kg*m^2\n', MB_25g.I_roll_wing);
fprintf('  EI root:                     %.4e Nm^2\n', D_25g.EI_root);
fprintf('  GJ root:                     %.4e Nm^2\n', D_25g.GJ_root);
fprintf('  EI hinge:                    %.4e Nm^2\n', D_25g.EI_hinge);
fprintf('  GJ hinge:                    %.4e Nm^2\n', D_25g.GJ_hinge);
fprintf('  Hinge M (SMT):               %.3f MNm\n', abs(S_25g.M_hinge)/1e6);
fprintf('  Hinge M (Snorri L_tip*d):    %.3f MNm\n', S_25g.M_hinge_snorri/1e6);
fprintf('=========================================================\n\n');
