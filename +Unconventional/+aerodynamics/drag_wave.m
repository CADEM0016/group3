function [CDw, MDD, A, B] = drag_wave(M, Mcrit, MmaxD, dCDmaxD)
%SNORRI_WAVE_DRAG  Transonic wave-drag rise using Gudmundsson's method.
%
%   [CDw, MDD, A, B] = snorri_wave_drag(M, Mcrit, MmaxD, dCDmaxD)
%
% Inputs
%   M         : Mach number (scalar or vector)
%   Mcrit     : critical Mach number
%   MmaxD     : Mach number where wave-drag rise has essentially saturated
%   dCDmaxD   : maximum wave-drag increment, Delta CD_maxD
%
% Outputs
%   CDw       : wave-drag coefficient increment at Mach number(s) M
%   MDD       : drag-divergence Mach number using Boeing definition CDw=0.002
%   A, B      : spline constants in CDw = 0.5*dCDmaxD*(1 + tanh(A*M + B))
%
% Notes
%   This implements the hyperbolic-tangent wave-drag spline described by
%   Snorri Gudmundsson for transonic drag rise.
%
%   Boundary conditions used to determine A and B:
%       at M = Mcrit,  CDw = 0.0001
%       at M = MmaxD,  CDw = dCDmaxD - 0.0001
%
%   Boeing drag-divergence definition:
%       MDD occurs when CDw = 0.002
%
% Example
%   M = linspace(0.6,1.05,300);
%   [CDw,MDD] = snorri_wave_drag(M,0.80,1.05,0.03);
%   plot(M,CDw), grid on
%   xlabel('Mach'), ylabel('C_{Dw}')
%   title(sprintf('Wave drag rise, MDD = %.4f', MDD))

    arguments
        M (:,1) double
        Mcrit (1,1) double {mustBePositive}
        MmaxD (1,1) double {mustBePositive}
        dCDmaxD (1,1) double {mustBePositive}
    end

    if MmaxD <= Mcrit
        error('MmaxD must be greater than Mcrit.');
    end

    if dCDmaxD <= 0.002
        error('dCDmaxD must be greater than 0.002 for Boeing MDD to exist.');
    end

    % Small offsets used by Gudmundsson to avoid tanh asymptotes
    epsLow  = 1e-4;
    epsHigh = 1e-4;

    % Solve for spline constants A and B
    yCrit = 2*epsLow/dCDmaxD - 1;
    yMax  = 2*(dCDmaxD - epsHigh)/dCDmaxD - 1;

    A = (atanh(yMax) - atanh(yCrit)) / (MmaxD - Mcrit);
    B = atanh(yCrit) - A*Mcrit;

    % Wave-drag increment
    CDw = 0.5*dCDmaxD .* (1 + tanh(A.*M + B));

    % Boeing drag-divergence Mach number: CDw = 0.002
    yMDD = 2*0.002/dCDmaxD - 1;

    if abs(yMDD) >= 1
        MDD = NaN;
        warning('Requested dCDmaxD does not permit a real Boeing MDD from CDw=0.002.');
    else
        MDD = (atanh(yMDD) - B) / A;
    end
end