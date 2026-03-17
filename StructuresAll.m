%  adp  - Unconventional.ADP()           team global object
%  tlar - cast.TLAR.Unconventional()     team global object
%  loc  - AircraftParams()               structures-only constants

clear;  clc;  close all;
addpath(fileparts(fileparts(mfilename('fullpath'))));

adp  = Unconventional.ADP();
tlar = cast.TLAR.Unconventional();
loc  = Unconventional.Structures.AircraftParams();

% Set unconventional design values on adp.
% These are properties declared in ADP.m with no default for this concept.
% Replace with live MDO loop output once converged.
adp.MTOM     = 348700;   % kg
adp.OEM      = 145000;   % kg
adp.Span     = 72.0;     % m
adp.WingArea = 436.8;    % m²
adp.KinkPos  = 10.0;     % m
adp.Mf_Fuel  = 0.19;
adp.Mf_res   = 0.05;

% 1 — geometry
G = Unconventional.Structures.WingGeometry(adp, tlar, loc);

% 2 — Class I/II empirical mass
E_Al = Unconventional.Structures.EmpiricalMass(adp, tlar, loc, G, 'Al');
E_CF = Unconventional.Structures.EmpiricalMass(adp, tlar, loc, G, 'CF');

% 3 — load distributions
L_25g = Unconventional.Structures.LoadDistribution(adp, tlar, loc, G, '2.5g');
L_1g  = Unconventional.Structures.LoadDistribution(adp, tlar, loc, G, '1g');
L_n1g = Unconventional.Structures.LoadDistribution(adp, tlar, loc, G, 'neg1g');

% 4 — SMT integration
S_25g = Unconventional.Structures.SMT(loc, G, L_25g);
S_1g  = Unconventional.Structures.SMT(loc, G, L_1g);
S_n1g = Unconventional.Structures.SMT(loc, G, L_n1g);

% 5 — wingbox sizing  (2.5g governs; CF for comparison)
W_25g    = Unconventional.Structures.WingboxSizing(loc, G, S_25g, 'Al');
W_n1g    = Unconventional.Structures.WingboxSizing(loc, G, S_n1g, 'Al');
W_25g_CF = Unconventional.Structures.WingboxSizing(loc, G, S_25g, 'CF');

% 6 — stiffness distributions
D_25g    = Unconventional.Structures.StiffnessDistribution(G, W_25g);
D_25g_CF = Unconventional.Structures.StiffnessDistribution(G, W_25g_CF);

% 7 — mass buildup
MB_25g    = Unconventional.Structures.MassBuildup(adp, loc, G, W_25g);
MB_25g_CF = Unconventional.Structures.MassBuildup(adp, loc, G, W_25g_CF);

% 8 — fidelity comparison table
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

% 9 — plots
Unconventional.Structures.Plots(adp, tlar, loc, G, L_25g, S_25g, W_25g, D_25g, MB_25g);

% 10 — sensitivity studies
Unconventional.Structures.SensitivityStudy(adp, tlar, loc);
