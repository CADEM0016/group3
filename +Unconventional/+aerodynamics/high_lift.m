function [outputArg1,outputArg2] = high_lift(obj,rho,V,cmac,mu)
%HIGH_LIFT Summary of this function goes here
% ESDU 91014 - Multi-Slotted Flap Analysis



Lambda_0 = atan( tan(Lambda_quarter) + (1/A)*((1 - lambda)/(1 + lambda)) );

Lambda_1 = atan( tan(Lambda_quarter) - (3/A)*((1 - lambda)/(1 + lambda)) );

Lambda_h = atan( tan(Lambda_quarter) + (4/A)*( (1/4 - 0.70) * ((1 - lambda)/(1 + lambda)) ) );

end