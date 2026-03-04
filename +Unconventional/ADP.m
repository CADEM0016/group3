classdef ADP < handle
    %ADP Aircraft Design Parameters for a B777F


    % Top level Design Parameters
    properties
        TLAR
        Engine
        AeroPolar
    end
    
    % Masses
    properties
        MTOM = 490000    % Maximum take-off mass
        OEM     % Operational Empty Mass
        Mf_Ldg  % maximum landing mass fraction (e.g. MLDG = MTOM*Mf_Ldg)
        Mf_Fuel % fuel mass fraction 
        Mf_TOC  % "top of climb" mass fraction
        Mf_res  % "Resevre Fuel" mass fraction
    end

    % constraint Paramters
    properties
        ThrustToWeightRatio  % 
        WingLoading          % 
    end
    
    % Aerodynamic
    properties
        % ------------------------- Tail -------------------------
        % GUESSES CURRENTLY
        % Tail volume
        V_HT = 0.9; % Horizontal Tail Volume
        V_VT = 0.07; % Vertical tail volume
        % Tail sweep
        HT_sweep= 30; % sweep at the Leading edge of the horizontal tail (deg)
        VT_sweep = 38; % sweep at the Leading edge of the vertical tail (deg)
        % Tail aspect ratio
        VT_AR = 1.5 % Vertical tail aspect ratio 
        HT_AR = 5 % Horizontal tail aspect ratio 
        % Tail taper ratio 
        VT_TR = 0.46
        HT_TR = 0.44
        % --------------------- aero properties ----------------------
        Cl_max = 1.5;   % airfoil amx Cl for wing
        
        Delta_Cl_ld = 1; % Extra CL during landing
        Delta_Cl_to = 0.8; % Extra CL at take-off

        CD_TO = 0.03;     % CD in ground run
        CL_TO = 0.8;      % CL during ground run        
        CD_LDG = 0.03;    % CD in ground run on landing
        CL_LDG = 0.8;     % CL during ground run on landing
        CL_cruise = 0.5;  % CL during cruise

        LD_c = 16;        % Lift to drag ratio in cruise
        LD_app = 10;      % Lift to drag ratio during landing
        CD0 = 0.02;       % Zero-lift drag coefficent
        e = 0.8;          % Oswald Efficency Factor
    end

    % Sizing Flags (whether to Adjust certain values during sizing process)
    properties
        isSizeEng = true; % whether to change engine maximum Thrust Value
        isSizeWing = true; % whether to size the wing
    end

    % Concrete properties
    properties
        Thrust;

        % planfrom specific
        Span;
        WingArea;
        KinkPos;    % y position of wing kink 
        WingPos;    % Wing position along fuselage
        HtpPos;     % HTP pos along fuselage
        VtpPos;     % VTP pos along fuselage

        Mstar = 0.935; % wing technology factor

        % Empenage Specific - Rough guesses
        HtpArea = 300;
        VtpArea= 200;
    end

    % useful properties
    properties
        c_ac % mean geometric chord of main wing
        x_ac % x location of mean geometeric chord
        c_ach % mean geometric chord of HTP
        c_acv % mean geometric chord of VTP
    end

    % fuselage properties
    properties
        FuselageLength = 65.625; % Total length of the fuselage
        FuselageDiameter = 6.2; % Diameter of the fuselage
        Decks = 2; % Number of flight decks
        CockpitLength= 6;
        CabinRadius = 3.15;
        CabinLength = 50.375;
        SidewallThickness= 0.15; % Outer thickness (m)
    end

    methods
        function out = AR(obj)
            out = obj.Span^2/obj.WingArea;
        end
    end
end