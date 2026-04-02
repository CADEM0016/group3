function [CL, CD] = lookup_data(span, alpha, mach, rootChord, altitude)

persistent FCL FCD

if isempty(FCL)
    S = load('AVL_lookup_table.mat');
    R = S.Results;

    CLgrid = nan(numel(S.spanVec), numel(S.alphaVec), numel(S.machVec), ...
                 numel(S.rootChordVec), numel(S.altVec));
    CDgrid = CLgrid;

    for k = 1:numel(R)
        iSpan  = find(abs(S.spanVec      - R(k).WingSpan)   < 1e-9, 1);
        iAlpha = find(abs(S.alphaVec     - R(k).Alpha)      < 1e-9, 1);
        iMach  = find(abs(S.machVec      - R(k).Mach)       < 1e-9, 1);
        iRoot  = find(abs(S.rootChordVec - R(k).RootChord)  < 1e-9, 1);
        iAlt   = find(abs(S.altVec       - R(k).Altitude)   < 1e-9, 1);

        if ~isempty(iSpan) && ~isempty(iAlpha) && ~isempty(iMach) && ~isempty(iRoot) && ~isempty(iAlt)
            CLgrid(iSpan,iAlpha,iMach,iRoot,iAlt) = R(k).CL;
            CDgrid(iSpan,iAlpha,iMach,iRoot,iAlt) = R(k).CD;
        end
    end

    FCL = griddedInterpolant({S.spanVec,S.alphaVec,S.machVec,S.rootChordVec,S.altVec}, CLgrid, 'linear', 'nearest');
    FCD = griddedInterpolant({S.spanVec,S.alphaVec,S.machVec,S.rootChordVec,S.altVec}, CDgrid, 'linear', 'nearest');
end

CL = FCL(span, alpha, mach, rootChord, altitude);
CD = FCD(span, alpha, mach, rootChord, altitude);

end