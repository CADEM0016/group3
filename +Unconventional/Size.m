function [ADP, out] = Size(ADP)
% SIZE  Iteratively size the aircraft until MTOM converges
%
%   Builds geometry, updates aerodynamics, runs mission analysis and
%   estimates required MTOM until convergence.
%
%   Structural wing mass is computed via Structures.WingWeightPhysics(ADP)
%   and fed back into the empty mass each iteration.

delta = inf;

while delta > 1

    %% -- Constraint Analysis --
    Unconventional.ConstraintAnalysis(ADP);

    %% -- Build Geometry --
    [~, B7Mass] = Unconventional.BuildGeometry(ADP);

    %% -- Update Aerodynamics --
    Unconventional.UpdateAero(ADP);

    %% -- Mission Analysis (two ranges, use worst case) --
    [BlockFuelA, TripFuelA, ResFuelA, Mf_TOC_A, MissionTimeA] = ...
        Unconventional.MissionAnalysis(ADP, ADP.TLAR.RangeA, ADP.MTOM);

    [BlockFuelB, TripFuelB, ResFuelB, Mf_TOC_B, MissionTimeB] = ...
        Unconventional.MissionAnalysis(ADP, ADP.TLAR.RangeB, ADP.MTOM);

    BlockFuel   = max(BlockFuelA,   BlockFuelB);
    TripFuel    = max(TripFuelA,    TripFuelB);
    ResFuel     = max(ResFuelA,     ResFuelB);
    Mf_TOC      = max(Mf_TOC_A,     Mf_TOC_B);
    MissionTime = max(MissionTimeA, MissionTimeB);

    %% -- Wing Structural Mass (Physics-Based) --
    try
        W_wing_struct = Structures.WingWeightPhysics(ADP);
    catch ME
        warning('Structures:Size:fallback', ...
            'WingWeightPhysics failed (%s). Using zero wing structural mass.', ...
            ME.message);
        W_wing_struct = 0;
    end

    %% -- Compute OEM --
    % Exclude fuel and payload mass objects from empty mass sum
    idx = contains([B7Mass.Name], 'Fuel',    'IgnoreCase', true) | ...
          contains([B7Mass.Name], 'Payload', 'IgnoreCase', true);

    ADP.OEM = sum([B7Mass(~idx).m]) + W_wing_struct;

    %% -- Estimate MTOM --
    mtom = sum([B7Mass(1:end-2).m]) + W_wing_struct + ADP.TLAR.Payload + BlockFuel;

    delta    = abs(ADP.MTOM - mtom);
    ADP.MTOM = mtom;

    %% -- Update Mass Fractions --
    ADP.Mf_Fuel = BlockFuel / ADP.MTOM;
    ADP.Mf_TOC  = Mf_TOC;
    ADP.Mf_Ldg  = (ADP.MTOM - TripFuel) / ADP.MTOM;
    ADP.Mf_res  = ResFuel / ADP.MTOM;

    %% -- Collect Outputs --
    out             = struct();
    out.BlockFuel   = BlockFuel;
    out.MissionTime = MissionTime;
    out.DOC         = BlockFuel * 1;   % placeholder cost model
    out.ATR         = BlockFuel;       % placeholder ATR model
    out.W_wing_struct = W_wing_struct;

end

end