function dCD = flap_drag(tc, cf_c, delta_f_deg, Sflap, Sref)
% SNORRI_SLOTTED_FLAP_DRAG
% Exact Snorri slotted-flap drag increment method:
%
% dCD = D1 * D2 * (Sflap / Sref)
%
% Inputs:
%   tc          = thickness/chord ratio (must be 0.12, 0.21, or 0.30)
%   cf_c        = flap chord ratio, cf/c
%   delta_f_deg = flap deflection in degrees
%   Sflap       = flapped wing area
%   Sref        = wing reference area
%
% Output:
%   dCD         = additive drag increment due to flap deployment

    % -------------------------
    % Delta 1 (function of cf/c)
    % -------------------------
    if abs(tc - 0.12) < 1e-6
        D1 = 179.32*cf_c^4 - 111.6*cf_c^3 + 28.929*cf_c^2 + 2.3705*cf_c - 0.0089;
    elseif abs(tc - 0.21) < 1e-6 || abs(tc - 0.30) < 1e-6
        D1 = 8.2658*cf_c^2 + 3.4564*cf_c + 0.0054;
    else
        error('tc must be 0.12, 0.21, or 0.30 for Snorri slotted flap method.')
    end

    % -------------------------
    % Delta 2 (function of flap deflection)
    % -------------------------
    if abs(tc - 0.12) < 1e-6
        D2 = -2.4416e-12*delta_f_deg^6 + 6.3942e-10*delta_f_deg^5 ...
           - 6.2028e-8*delta_f_deg^4  + 2.4984e-6*delta_f_deg^3 ...
           - 1.8922e-5*delta_f_deg^2  + 3.1582e-4*delta_f_deg ...
           + 6.9698e-5;

    elseif abs(tc - 0.21) < 1e-6
        D2 =  6.2317e-11*delta_f_deg^5 - 1.3354e-8*delta_f_deg^4 ...
           + 6.4833e-7*delta_f_deg^3  + 2.1134e-5*delta_f_deg^2 ...
           - 2.6425e-4*delta_f_deg    + 5.2279e-4;

    elseif abs(tc - 0.30) < 1e-6
        D2 = -3.7252e-7*delta_f_deg^3 + 5.4024e-5*delta_f_deg^2 ...
           - 4.4994e-4*delta_f_deg    + 1.1175e-3;
    end

    % -------------------------
    % Final Snorri equation
    % -------------------------
    dCD = D1 * D2 * (Sflap / Sref);
end