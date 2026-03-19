function M = mach_from_velocity(V, alt)
% MACH_FROM_VELOCITY Computes Mach number from velocity
%
% Inputs:
% V   - velocity [m/s]
% alt - altitude [m]
%
% Output:
% M   - Mach number

% ISA temperature model
if alt < 11000
    T = 288.15 - 0.0065*alt;
else
    T = 216.65;
end

% Constants
gamma = 1.4;     % ratio of specific heats
R = 287;         % gas constant for air

% Speed of sound
a = sqrt(gamma*R*T);

% Mach number
M = V/a;

end