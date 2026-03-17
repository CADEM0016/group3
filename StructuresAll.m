%% StructuresAll - Wing Structural Analysis
%
%  Global objects (adp, tlar) supply all shared aircraft values.
%  Local constants (loc) supply structures-specific values from AircraftParams.
%
%  Pipeline:  loc → geometry → loads → SMT → sizing → stiffness → mass

clear;  clc;  close all;
addpath(fileparts(fileparts(mfilename('fullpath'))));

% Global objects
adp  = Unconventional.ADP();
tlar = cast.TLAR();

% Structures-specific constants
loc = Unconventional.Structures.AircraftParams();

% Step 1 — wing geometry
G = Unconventional.Structures.WingGeometry(adp, tlar, loc);

% Step 2 — Class I/II empirical mass
E_Al = Unconventional.Structures.EmpiricalMass(adp, tlar, loc, G, 'Al');
E_CF = Unconventional.Structures.EmpiricalMass(adp, tlar, loc, G, 'CF');

% Step 3 — load distributions  (three CS-25 cases)
L_25g = Unconventional.Structures.LoadDistribution(adp, tlar, loc, G, '2.5g');
L_1g  = Unconventional.Structures.LoadDistribution(adp, tlar, loc, G, '1g');
L_n1g = Unconventional.Structures.LoadDistribution(adp, tlar, loc, G, 'neg1g');

% Step 4 — SMT integration
S_25g = Unconventional.Structures.SMT(loc, G, L_25g);
S_1g  = Unconventional.Structures.SMT(loc, G, L_1g);
S_n1g = Unconventional.Structures.SMT(loc, G, L_n1g);

% Step 5 — wingbox sizing  (2.5g governs; CF included for comparison)
W_25g    = Unconventional.Structures.WingboxSizing(loc, G, S_25g, 'Al');
W_n1g    = Unconventional.Structures.WingboxSizing(loc, G, S_n1g, 'Al');
W_25g_CF = Unconventional.Structures.WingboxSizing(loc, G, S_25g, 'CF');

% Step 6 — stiffness distributions EI(y) and GJ(y)
D_25g    = Unconventional.Structures.StiffnessDistribution(G, W_25g);
D_25g_CF = Unconventional.Structures.StiffnessDistribution(G, W_25g_CF);

% Step 7 — mass buildup
MB_25g    = Unconventional.Structures.MassBuildup(adp, loc, G, W_25g);
MB_25g_CF = Unconventional.Structures.MassBuildup(adp, loc, G, W_25g_CF);

% Step 8 — fidelity comparison table
fprintf('\n+----------------------------------+----------+---------+\n');
fprintf('| Method                           | Mass [kg]|  %%MTOM |\n');
fprintf('+----------------------------------+----------+---------+\n');
fprintf('| I/II  Raymer                     | %8.0f | %6.2f%% |\n', E_Al.m_raymer,    E_Al.m_raymer/adp.MTOM*100);
fprintf('| I/II  Torenbeek                  | %8.0f | %6.2f%% |\n', E_Al.m_torenbeek, E_Al.m_torenbeek/adp.MTOM*100);
fprintf('| I/II  USAF                       | %8.0f | %6.2f%% |\n', E_Al.m_usaf,      E_Al.m_usaf/adp.MTOM*100);
fprintf('| I/II  avg + hinge  (Al)          | %8.0f | %6.2f%% |\n', E_Al.m_total,     E_Al.m_frac_MTOM*100);
fprintf('| I/II  avg + hinge  (CF)          | %8.0f | %6.2f%% |\n', E_CF.m_total,     E_CF.m_frac_MTOM*100);
fprintf('+----------------------------------+----------+---------+\n');
fprintf('| II.5  2.5g  (Al)                 | %8.0f | %6.2f%% |\n', MB_25g.m_total,    MB_25g.m_frac_MTOM*100);
fprintf('| II.5  2.5g  (CF)                 | %8.0f | %6.2f%% |\n', MB_25g_CF.m_total, MB_25g_CF.m_total/adp.MTOM*100);
fprintf('+----------------------------------+----------+---------+\n');
fprintf('| B777F reference                  |    34000 |   9.76%% |\n');
fprintf('+----------------------------------+----------+---------+\n');

% Step 9 — plots
Unconventional.Structures.Plots(adp, loc, G, L_25g, S_25g, W_25g, D_25g, MB_25g);

% Step 10 — sensitivity studies
Unconventional.Structures.SensitivityStudy(adp, tlar, loc);
