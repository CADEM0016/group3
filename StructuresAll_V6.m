%  adp  - Unconventional.ADP()           team global object
%  tlar - cast.TLAR.Unconventional()     team global object
%  loc  - AircraftParams()               structures-only constants

clear;  clc;  close all;
addpath(fileparts(fileparts(mfilename('fullpath'))));

adp  = Unconventional.ADP();
tlar = cast.TLAR.Unconventional();
loc  = Unconventional.Structures.V6.AircraftParams();

% Flight span 79.75 m folds to 65.0 m (Code E) at y_hinge = 32.5 m.

adp.MTOM     = 575000;   % kg
adp.OEM      = 277000;   % kg
adp.Span     = 79.75;    % m   flight span
adp.WingArea = 845.0;    % m²
adp.KinkPos  = 14.0;     % m
adp.Mf_Fuel  = 0.45;
adp.Mf_res   = 0.05;
adp.TLAR     = tlar;
adp.WingLoading = adp.MTOM * SI.g / adp.WingArea;
if isempty(adp.WingPos)
    adp.WingPos = adp.WingStation;   % default global wing placement station
end
if isempty(adp.Mf_TOC)
    adp.Mf_TOC = 0.97;               % default top-of-climb mass fraction
end

if ~isempty(adp.Thrust)
    Unconventional.geom.engine(adp);   
else
    warning('StructuresAll_V6:GlobalPreStep', ...
        'adp.Thrust is empty, skipping Unconventional.geom.engine(adp).');
end
Unconventional.geom.wing(adp);         % fills adp.c_ac (and x_ac)

% Sanity checks for shared inputs when available.
if ~isempty(adp.Engine)
    assert(isprop(adp.Engine,'Mass') && ~isempty(adp.Engine.Mass), ...
        'Missing adp.Engine.Mass from global engine model');
end
assert(isprop(adp,'c_ac') && ~isempty(adp.c_ac), ...
    'Missing adp.c_ac from global wing geometry model');
assert(isprop(adp,'e') && ~isempty(adp.e), ...
    'Missing adp.e from global aero/ADP model');

% 1 - geometry
G = Unconventional.Structures.V6.WingGeometry(adp, tlar, loc);

% 2 - Class I/II empirical mass
E_Al = Unconventional.Structures.V6.EmpiricalMass(adp, tlar, loc, G, 'Al');
E_CF = Unconventional.Structures.V6.EmpiricalMass(adp, tlar, loc, G, 'CF');

% 3 - load distributions
L_25g = Unconventional.Structures.V6.LoadDistribution(adp, tlar, loc, G, '2.5g');
L_1g  = Unconventional.Structures.V6.LoadDistribution(adp, tlar, loc, G, '1g');
L_n1g = Unconventional.Structures.V6.LoadDistribution(adp, tlar, loc, G, 'neg1g');

% 4 - SMT integration
S_25g = Unconventional.Structures.V6.SMT(loc, G, L_25g);
S_1g  = Unconventional.Structures.V6.SMT(loc, G, L_1g);
S_n1g = Unconventional.Structures.V6.SMT(loc, G, L_n1g);

% 5 - wingbox sizing  (2.5g governs; CF for comparison)
W_25g    = Unconventional.Structures.V6.WingboxSizing(loc, G, S_25g, 'Al');
W_n1g    = Unconventional.Structures.V6.WingboxSizing(loc, G, S_n1g, 'Al');
W_25g_CF = Unconventional.Structures.V6.WingboxSizing(loc, G, S_25g, 'CF');

% 6 - stiffness distributions
D_25g    = Unconventional.Structures.V6.StiffnessDistribution(G, W_25g);
D_25g_CF = Unconventional.Structures.V6.StiffnessDistribution(G, W_25g_CF);

% 7 - mass buildup
MB_25g    = Unconventional.Structures.V6.MassBuildup(adp, loc, G, W_25g);
MB_25g_CF = Unconventional.Structures.V6.MassBuildup(adp, loc, G, W_25g_CF);

% 8a - folding wingtip structural analysis
FT = Unconventional.Structures.V6.FoldingWingtip(loc, G, S_25g, S_1g, S_n1g, W_25g, D_25g, MB_25g);

% 8b - fidelity comparison table
fprintf('\n+----------------------------------+----------+---------+\n');
fprintf('| Method                           | Mass [kg]|  %%MTOM |\n');
fprintf('+----------------------------------+----------+---------+\n');
fprintf('| I/II  Raymer                     | %8.0f | %6.2f%% |\n', E_Al.m_raymer,    E_Al.m_raymer/adp.MTOM*100);
fprintf('| I/II  Torenbeek                  | %8.0f | %6.2f%% |\n', E_Al.m_torenbeek, E_Al.m_torenbeek/adp.MTOM*100);
fprintf('| I/II  USAF                       | %8.0f | %6.2f%% |\n', E_Al.m_usaf,      E_Al.m_usaf/adp.MTOM*100);
fprintf('| I/II  avg + hinge  (Al)          | %8.0f | %6.2f%% |\n', E_Al.m_total,     E_Al.m_frac_MTOM*100);
fprintf('| I/II  avg + hinge  (CF)          | %8.0f | %6.2f%% |\n', E_CF.m_total,     E_CF.m_frac_MTOM*100);
fprintf('+----------------------------------+----------+---------+\n');
fprintf('| II.5  2.5g  (Al)  excl. tip     | %8.0f | %6.2f%% |\n', MB_25g.m_total,    MB_25g.m_frac_MTOM*100);
fprintf('| II.5  2.5g  (CF)  excl. tip     | %8.0f | %6.2f%% |\n', MB_25g_CF.m_total, MB_25g_CF.m_total/adp.MTOM*100);
fprintf('| II.5  fold tip mechanism         | %8.0f | %6.2f%% |\n', FT.m_fold_penalty, FT.m_fold_penalty/adp.MTOM*100);
fprintf('| II.5  total incl. fold tip (Al)  | %8.0f | %6.2f%% |\n', MB_25g.m_total+FT.m_fold_penalty, (MB_25g.m_total+FT.m_fold_penalty)/adp.MTOM*100);
fprintf('+----------------------------------+----------+---------+\n');
fprintf('| A380 reference                   |    69000 |  12.00%% |\n');
fprintf('+----------------------------------+----------+---------+\n');

% 9 - plots
Unconventional.Structures.V6.Plots(adp, tlar, loc, G, L_25g, S_25g, W_25g, D_25g, MB_25g);

% 9b - folding wingtip plots
Unconventional.Structures.V6.PlotsFoldingWingtip(loc, G, S_25g, S_1g, W_25g, D_25g, FT);

% 10 - sensitivity studies
Unconventional.Structures.V6.SensitivityStudy(adp, tlar, loc);

% 11 - Plots of Folding Tip Sensitivity
Unconventional.Structures.V6.PlotFoldingTipVsWingMass(G, loc, MB_25g, MB_25g_CF);
