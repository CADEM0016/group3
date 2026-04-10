function delta_r = required_rudder_deflection(ac, cond)
% Returns required rudder deflection [rad]
%
% cond should represent either OEI or crosswind case

    q = 0.5 * cond.rho * cond.V^2;

    % Engine-out yawing moment
    N_eng = cond.T_live * cond.y_engine;

    % External/crosswind sideslip contribution is modeled through beta
    beta = cond.beta;

    denom = q * ac.wing.S * ac.wing.b * ac.stability.Cn_delta_r;

    num = q * ac.wing.S * ac.wing.b * ac.stability.Cn_beta * beta + N_eng;

    delta_r = - num / denom;
end