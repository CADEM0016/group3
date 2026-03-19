
classdef TLAR
    %TLAR Top-Level Aircraft (Design) Requirements
    
    properties
        Crew
        Range = 10000*10^3     % Harmonic Range
        Payload     % Max. Payload
        V_ld        % Landing Speed
        V_app       % approach speed
        V_climb     % climb speed (CAS)
        GroundRun
        GroundRunLanding
        M_c         % cruise Mach number
        Alt_max     % max altitude in m
        Alt_cruise  % Cruise Altitude
        CrewMass    % Mass of the Crew
    end

    properties
        % Performance Properties - mission analysis
        RangeA % London to Singapore
        RangeB % Singapore to Melborne
    end

    properties
        M_alt % Mach number at each alititude to be limited by either M_c or V_climb
    end

    % alternate airport diversion properties
    properties
        Alt_alternate = 22e3./SI.ft;
        Range_alternate = 350000;
        Loiter = 30./SI.min; % 30 minutes in seconds
    end
    methods(Static)
        function obj = Unconventional
            obj = cast.TLAR();
            obj.GroundRun = 2950; %m
            obj.GroundRunLanding = 2500; %m
            obj.M_c = 0.85;
<<<<<<< Updated upstream
            obj.RangeA = 10888000;%/SI.Nmile;% m (from nautical miles) - london singapore
            obj.RangeB= 6024100;%/SI.Nmile;% m (from nautical miles) - singapore melbourne
=======

%merge stuff

>>>>>>> Stashed changes
            obj.RangeA = 10888000;%/SI.Nmile;% m (from nautical miles) - london singapore ------- find study of things
            obj.RangeB = 6024100;%/SI.Nmile;% m (from nautical miles) - singapore melbourne
            obj.GroundRun = 2830; %m
            obj.GroundRunLanding = 1500; %m
            obj.M_c = 0.78;

<<<<<<< Updated upstream
=======
%merge stuff


>>>>>>> Stashed changes
            obj.Alt_max = 39e3./SI.ft; %m (39,000ft)
            obj.Alt_cruise = 34e3./SI.ft;
            obj.Crew = 4;
            obj.Payload = 138500;
            obj.CrewMass = (80+10)*obj.Crew;
            obj.V_app = 200./SI.knt;
            obj.V_ld = 150./SI.knt;
            obj.V_climb = 250/SI.knt;
        end
    end
end