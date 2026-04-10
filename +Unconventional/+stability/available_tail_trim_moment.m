function Mtail = tail_trim_moment(ac, cond, deltae)
% Returns actual tail pitching moment [N m] for a given elevator deflection

    q = 0.5 * cond.rho * cond.V^2;
    qt = ac.tail.horizontal.eta * q;

    alpha_t = cond.alpha + ac.tail.horizontal.it - cond.downwash;

    CLt = ac.tail.horizontal.a * (alpha_t + ac.tail.horizontal.tau * deltae);

    CLt = max(min(CLt, ac.tail.horizontal.CLmax), ac.tail.horizontal.CLmin);

    Lt = qt * ac.tail.horizontal.S * CLt;

    Mtail = -Lt * ac.tail.horizontal.lt;
end