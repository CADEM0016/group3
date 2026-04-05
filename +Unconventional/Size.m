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
    fprintf("delta_conv: %g\n",delta)
    %% -- Constraint Analysis --
    Unconventional.ConstraintAnalysis(ADP);

    %% -- Build Geometry --
    [~, B7Mass] = Unconventional.BuildGeometry(ADP);

    %% -- Update Aerodynamics --
    Unconventional.UpdateAero(ADP);

    %% -- Mission Analysis (two ranges, use worst case) --


    [BlockFuel,TripFuel,ResFuel,Mf_TOC,MissionTime] = Unconventional.MissionAnalysis_oscar(ADP,ADP.TLAR.RangeDes, ADP.MTOM);
    

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
    
    fprintf("OEW %g\n",sum(([B7Mass(1:end-2).m])+W_wing_struct))
    fprintf("BlockFuel %f\n",BlockFuel) 
    fprintf("Payload mass %f\n",ADP.TLAR.Payload)
    fprintf("ADP.MTOM %f\n",ADP.MTOM)
    fprintf("mtom %f\n",mtom) 

    delta    = abs(ADP.MTOM - mtom);
    ADP.MTOM = mtom;

% function [ADP,out] = Size(ADP)
% % interatively build the model, run mission analysis and estimate required
% %  MTOM untill covnergence
% delta = inf;
% while delta>1
%     % constraint Analysis
%     Unconventional.ConstraintAnalysis(ADP);
% 
%     % build geometry
%     [~,B7Mass] = Unconventional.BuildGeometry(ADP);
% 
%     % update Aero
%     Unconventional.UpdateAero(ADP);
% 
%     % mission Analysis
% %    ADP.TLAR.Range
%     [BlockFuel,TripFuel,ResFuel,Mf_TOC,MissionTime] = Unconventional.MissionAnalysis(ADP,ADP.TLAR.RangeDes, ADP.MTOM);
%     TripFuel
% 
% %merge stash    
% 
%     % calc OEM
%     idx = contains([B7Mass.Name],"Fuel","IgnoreCase",true) | contains([B7Mass.Name],"Payload","IgnoreCase",true);
%     ADP.OEM = sum([B7Mass(~idx).m]);
%     % estimate MTOM
%     mtom = sum([B7Mass(1:end-2).m])+ADP.TLAR.Payload+BlockFuel;
%     delta = abs(ADP.MTOM - mtom);
% %>>>>>>> Stashed changes
    
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