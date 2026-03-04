function W_wing = WingWeightPhysics(ac)
%WINGWEIGHTPHYSICS Computes physics-based wing structural mass

loads = Structures.WingLoads(ac);

sizing = Structures.WingSizing(ac, loads);

W_wing = sizing.mass;

end