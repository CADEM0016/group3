function [ADP,out] = Size(ADP)
% interatively build the model, run mission analysis and estimate required
%  MTOM untill covnergence
delta = inf;
while delta>1
    % constraint Analysis
    B777.ConstraintAnalysis(ADP);
    
    % build geometry
    [~,B7Mass] = B777.BuildGeometry(ADP);
    
    % update Aero
    B777.UpdateAero(ADP);
    
    % mission Analysis

    %DOUBLE CHECK ITERATIONS
    %        T_Static = 374.5e3 %double check connection
    %        T2W = 0.3; % check connections
    %        MTOM = 271484; %check connections
    %        T_new = T2W*MTOM*9.81; %Check T2W & MTOM connection    
    %

    [BlockFuelA,TripFuelA,ResFuelA,Mf_TOC_A,MissionTimeA] = B777.MissionAnalysis(ADP,ADP.TLAR.RangeA, ADP.MTOM);
    [BlockFuelB,TripFuelB,ResFuelB,Mf_TOC_B,MissionTimeB] = B777.MissionAnalysis(ADP,ADP.TLAR.RangeB, ADP.MTOM);

    BlockFuel = max(BlockFuelA, BlockFuelB);
    TripFuel  = max(TripFuelA,  TripFuelB);
    ResFuel   = max(ResFuelA,   ResFuelB);
    Mf_TOC    = max(Mf_TOC_A,   Mf_TOC_B);
    MissionTime = max(MissionTimeA, MissionTimeB);

    % calc OEM
    idx = contains([B7Mass.Name],"Fuel","IgnoreCase",true) | contains([B7Mass.Name],"Payload","IgnoreCase",true);
    ADP.OEM = sum([B7Mass(~idx).m]);
    % estimate MTOM
    mtom = sum([B7Mass(1:end-2).m])+ADP.TLAR.Payload+BlockFuel;
    delta = abs(ADP.MTOM - mtom);
    ADP.MTOM = mtom;
    ADP.Mf_Fuel = BlockFuel /ADP.MTOM;
    ADP.Mf_TOC = Mf_TOC;
    ADP.Mf_Ldg = (ADP.MTOM-TripFuel)/ADP.MTOM;
    ADP.Mf_res = ResFuel/ADP.MTOM;
    %estimate outut parameters
    out = struct();
    out.BlockFuel = BlockFuel;
    out.DOC = BlockFuel*1;
    out.ATR = BlockFuel;
end
end