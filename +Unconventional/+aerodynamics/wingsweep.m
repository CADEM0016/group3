function sweepDeg = wingsweep(cruiseMach)
%REQUIRED_WING_SWEEP_FROM_CRITICAL_MACH
% Returns the required wing sweep angle in degrees based on a simple
% critical-Mach normal-flow relation:
%
%   M_crit = M_cruise * cos(Lambda)
%
% Rearranged:
%
%   Lambda = acos(M_crit / M_cruise)
%
% This is a first-pass conceptual-design estimate.

    Mcrit = 0.72; % assumed section critical Mach number for first-pass sizing

    if cruiseMach <= 0
        error('cruiseMach must be positive.')
    end

    ratio = Mcrit / cruiseMach;

    if ratio >= 1
        sweepDeg = 0;
    else
        sweepDeg = rad2deg(acos(ratio));
    end
end