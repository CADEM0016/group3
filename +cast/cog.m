function [Cog,PercentMAC] = cog(Masses)
% COG Summary of this function goes here
% Given the mass and geometry object works out the CG
% location as well as its percentage to the MAC for stability

arguments (Input)
    Masses cast.MassObj
end

arguments (Output)
    Cog (1,2) double
    PercentMAC (1,1) double
end

Cog = sum([Masses.X].*[Masses.m],2)./sum([Masses.m]);
end