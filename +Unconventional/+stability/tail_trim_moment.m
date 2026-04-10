function Mreq = tail_trim_moment(ac, cond)
% Returns required tail balancing moment [N m]
%
% ac   : aircraft data struct
% cond : condition struct

    q = 0.5 * cond.rho * cond.V^2;

    h_cg = cond.xcg / ac.wing.mac;
    h_ac = ac.aero.xac / ac.wing.mac;

    % Wing-body pitching moment coefficient about CG
    Cm_wb = ac.aero.Cm0 + cond.CL * (h_cg - h_ac);

    M_wb = q * ac.wing.S * ac.wing.mac * Cm_wb;

    % Thrust moment about CG
    M_thrust = cond.T * ac.propulsion.zThrust;

    % Flap moment (set zero if not in use)
    M_flap = cond.Mflap;

    % Tail must cancel all of this
    Mreq = -(M_wb + M_thrust + M_flap);
end