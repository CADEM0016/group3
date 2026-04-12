function out = plotCGbubble(obj,opts)
arguments
    obj
    opts.FuelFractions = 0:0.25:1
    opts.PalletCounts = 0:45
    opts.PalletMass = obj.TLAR.Payload/45      % kg per pallet if full payload = 45 pallets
    opts.CargoStart = 7                        % m from datum
    opts.NLong = 15
    opts.NAbreast = 3
    opts.PalletPitch = []                      % leave empty -> uses CabinLength/NLong
    opts.Patterns string = ["fwd","aft","ctr"]
end

% --- base aircraft masses (no fuel / no cargo) ---
GeomObj = struct.empty; massObj = struct.empty;
for fn = ["wing","empenage","fuselage","engine","landingGear"]
    [g,m] = Unconventional.geom.(fn)(obj);
    GeomObj = [GeomObj,g];
    massObj = [massObj,m];
end
m0 = [massObj.m];
x0 = arrayfun(@(k) k.X(1), massObj);
M0 = sum(m0.*x0); 
W0 = sum(m0);

% fuel x-location = wing x-location (fallback included)
iw = find(strcmpi(arrayfun(@(k) string(k.Name), massObj),"Wing"),1);
if isempty(iw), xFuel = obj.WingPos + 0.15*obj.c_ac; else, xFuel = massObj(iw).X(1); end
mFuelMax = obj.MTOM*obj.Mf_Fuel;

% pallet station x-locations
if isempty(opts.PalletPitch), opts.PalletPitch = obj.CabinLength/opts.NLong; end
xRow = opts.CargoStart + ((1:opts.NLong)-0.5)*opts.PalletPitch;   % 15 longitudinal stations

% MAC conversion
if numel(obj.mac)==2
    x2mac = @(x) 100*(x-obj.mac(1))/obj.mac(2);   % obj.mac = [x_LEMAC, MAC]
else
    x2mac = @(x) 100*x/obj.mac;                    % if obj.mac is just MAC
end

% --- generate cases ---
P = [];
for pat = opts.Patterns
    for nf = opts.FuelFractions
        for np = opts.PalletCounts
            [mCargo,xCargo] = cargoCase(np,pat,xRow,opts.NAbreast,opts.PalletMass);
            W = W0 + nf*mFuelMax + mCargo;
            xcg = (M0 + nf*mFuelMax*xFuel + mCargo*xCargo)/W;
            P = [P; x2mac(xcg), W, np, nf, string(pat)]; %#ok<AGROW>
        end
    end
end

% --- plot ---
figure; hold on; grid on; box on
mk = struct("fwd","o","aft","s","ctr","^");
for pat = opts.Patterns
    I = P(:,5)==string(pat);
    scatter(P(I,1),P(I,2),40+220*str2double(P(I,4)),str2double(P(I,3)),'filled',mk.(pat), ...
        'DisplayName',char(pat));
end
xlabel('CG (%MAC)'); ylabel('Mass');
cb = colorbar; cb.Label.String = 'Number of pallets';
title('CG loading cloud / bubble plot');
legend('Location','best');

out.points = P;
out.baseMass = W0;
out.baseCG = x2mac(M0/W0);
out.xRow = xRow;
out.GeomObj = GeomObj;
out.massObj = massObj;
end

function [m,xcg] = cargoCase(np,pat,xRow,nAbreast,mPal)
if np==0, m=0; xcg=0; return; end
nLong = numel(xRow);
rowLoad = zeros(1,nLong);
nFull = floor(np/nAbreast);
nRem  = mod(np,nAbreast);

switch char(pat)
    case 'fwd'
        idx = 1:nLong;
    case 'aft'
        idx = nLong:-1:1;
    otherwise % 'ctr'
        [~,c] = min(abs((1:nLong)-(nLong+1)/2));
        idx = [c, c+(-1:-1:1), c+(1:nLong)]; idx = idx(idx>=1 & idx<=nLong);
end

if nFull>0, rowLoad(idx(1:nFull)) = nAbreast*mPal; end
if nRem>0,  rowLoad(idx(nFull+1)) = nRem*mPal; end

m = sum(rowLoad);
xcg = sum(rowLoad.*xRow)/m;
end