c%% Evaluate mission fuel evolution (stateful, fixed fuel capacity)

clear
clc

%% Run sizing first
scripts.ExampleUnconventional

%% =========================
% Fixed aircraft/design constants
% ==========================

OEM     = ADP.OEM;
Payload = ADP.TLAR.Payload;

% This is a mission/design constant for fixed-payload study
FuelCapacity = ADP.MTOM - OEM - Payload;


%% =========================
% Mission definition
% ==========================

 
AirportPairs = [
"LHR-MEL"
"MEL-PVG"
"PVG-SUZ"
"SUZ-BAH"
"BAH-JED"
"JED-MIA"
"MIA-YUL"
"YUL-MCM"
"MCM-MAD"
"MAD-GYD"
"GYD-SIN"
"SIN-AUS"
"AUS-MEX"
"MEX-GRU"
"GRU-LAS"
"LAS-LUA"
"LUA-AUH"
"AUH-LHR"
];


dist_km = [
16909.38
8017.62
1456.91
8052.37
1272.47
11621.60
2264.78
6128.98
956.83
4462.04
6941.58
15823.46
1205.03
7433.53
9782.64
13052.79
320.63
5515.99
];

nOriginalLegs = length(dist_km);

% =========================
% Payload per original leg
% ==========================

Payload_leg = Payload * ones(nOriginalLegs,1);

for i = 1:nOriginalLegs
    
    if AirportPairs(i) == "MCM-MAD" || ...
       AirportPairs(i) == "AUS-MEX"
        
        Payload_leg(i) = 0;
        
    end
    
end

%% =========================
% Split legs based on aircraft range
% ==========================

Range_max   = ADP.TLAR.RangeDes;
DesignRange = ADP.TLAR.RangeDes;

ExpandedLegs = struct([]);
idx = 1;

for i = 1:nOriginalLegs
    
    totalRange = dist_km(i) * 1e3;
    
    if totalRange <= Range_max
        
        ExpandedLegs(idx).Name  = "Leg " + i;
        ExpandedLegs(idx).Route       = AirportPairs(i);
        ExpandedLegs(idx).IsSplit     = false;
        ExpandedLegs(idx).SubIndex    = 1;
        ExpandedLegs(idx).IsRefuelLeg = false;
        ExpandedLegs(idx).Range = totalRange;
        idx = idx + 1;
        
    else
        
        nSub = ceil(totalRange / Range_max);
        subRange = totalRange / nSub;
        
        for j = 1:nSub
            ExpandedLegs(idx).Name        = "Leg " + i + "." + j;
            ExpandedLegs(idx).Route       = AirportPairs(i);
            ExpandedLegs(idx).IsSplit     = true;
            ExpandedLegs(idx).SubIndex    = j;
            ExpandedLegs(idx).IsRefuelLeg = true; % these exist because of range limit

            ExpandedLegs(idx).Range = subRange;
            idx = idx + 1;
        end
        
    end
end

nLegs      = length(ExpandedLegs);
nStops     = nLegs - 1;
TotalRange = sum([ExpandedLegs.Range]);

fprintf('\n----- Mission Setup -----\n')
fprintf('Original legs: %d\n', nOriginalLegs)
fprintf('Effective legs: %d\n', nLegs)
fprintf('Stops: %d\n', nStops)
fprintf('Design range: %.0f km\n', DesignRange/1000)
fprintf('Total mission range: %.0f km\n', TotalRange/1000)


%% =========================
% Initial mission state
% ==========================

FuelRemaining = 0;

fuelTimeline  = [];
labelTimeline = [];

TripFuel_vec    = zeros(1,nLegs);
BlockFuel_vec   = zeros(1,nLegs);   % actual fuel at start of leg
ResFuel_vec     = zeros(1,nLegs);
FuelEnd_vec     = zeros(1,nLegs);
RefuelAmount    = zeros(1,nLegs);
TakeoffMass_vec = zeros(1,nLegs);

TotalMissionTime = 0;

%% =========================
% Mission evaluation
% ==========================

for i = 1:nLegs
    
    legRange = ExpandedLegs(i).Range;
    legName  = ExpandedLegs(i).Name;
    
    % First-pass requirement estimate
    [~, TripFuel_est, ResFuel_est, ~, ~] = ...
        Unconventional.MissionAnalysis_oscar(ADP, legRange, ADP.MTOM);
    
    RequiredFuel = TripFuel_est + ResFuel_est;
    
    % Uplift only what is missing
    FuelNeeded = RequiredFuel - FuelRemaining;
    FuelUplift = max(0, FuelNeeded);
    
    % Cannot exceed fixed fuel capacity
    FuelUplift = min(FuelUplift, FuelCapacity - FuelRemaining);
    
    % Fuel at start of leg
    FuelStart = FuelRemaining + FuelUplift;
    
    % Actual takeoff mass for this leg
    CurrentWeight = OEM + Payload + FuelStart;
    
    % Re-run at actual start mass
    [~, TripFuel, ResFuel, ~, MissionTime] = ...
        Unconventional.MissionAnalysis_oscar(ADP, legRange, CurrentWeight);
    
    FuelEnd = FuelStart - TripFuel;
    
    % Store state for next leg
    FuelRemaining = FuelEnd;
    
    % Save outputs
    TripFuel_vec(i)    = TripFuel;
    BlockFuel_vec(i)   = FuelStart;
    ResFuel_vec(i)     = ResFuel;
    FuelEnd_vec(i)     = FuelEnd;
    RefuelAmount(i)    = FuelUplift;
    TakeoffMass_vec(i) = CurrentWeight;
    
    TotalMissionTime = TotalMissionTime + MissionTime;
    
    % Timeline
    fuelTimeline = [fuelTimeline, FuelStart, FuelEnd, ResFuel];
    route = ExpandedLegs(i).Route;
    
    if ExpandedLegs(i).IsSplit
        tag = " (REFUEL)";
    else
        tag = "";
    end
    
    labelTimeline = [labelTimeline, ...
        legName + " [" + route + "]" + tag + " Start", ...
        legName + " [" + route + "]" + tag + " End", ...
        legName + " [" + route + "]" + tag + " Res"];
end

%% =========================
% Derived metrics
% ==========================

TotalBurn    = sum(TripFuel_vec);
MaxReserve   = max(ResFuel_vec);
MinReserve   = min(ResFuel_vec);
TotalCarried = TotalBurn + MaxReserve;

ReserveMargin = FuelEnd_vec - ResFuel_vec;

%% =========================
% FIGURE 1: Fuel timeline
% ==========================

figure
x = 1:length(fuelTimeline);

plot(x, fuelTimeline/1e3, '-o', 'LineWidth', 1.8)
set(gca, 'XTick', x, 'XTickLabel', labelTimeline)
xtickangle(45)

ylabel('Fuel Remaining [t]')
xlabel('Mission Events')
title('Fuel Evolution (Start → End → Reserve per Leg)')
grid on

%% =========================
% FIGURE 2: Debug dashboard
% ==========================

figure
tiledlayout(2,2)

nexttile
bar([BlockFuel_vec'/1e3, FuelEnd_vec'/1e3])
legend('Start','End')
xticks(1:nLegs)
xticklabels({ExpandedLegs.Name})
xtickangle(45)
ylabel('Fuel [t]')
title('Fuel Before and After Each Leg')
grid on

nexttile
bar(RefuelAmount/1e3)
xticks(1:nLegs)
xticklabels({ExpandedLegs.Name})
xtickangle(45)
ylabel('Fuel Added [t]')
title('Required Fuel Uplift per Leg')
grid on

nexttile
plot(ResFuel_vec/1e3, '-o', 'LineWidth', 1.5)
xticks(1:nLegs)
xticklabels({ExpandedLegs.Name})
xtickangle(45)
ylabel('Reserve [t]')
title('Reserve per Leg')
grid on

nexttile
bar(ReserveMargin/1e3)
xticks(1:nLegs)
xticklabels({ExpandedLegs.Name})
xtickangle(45)
ylabel('Fuel Margin [t]')
title('End Fuel - Reserve (Check)')
grid on

%% =========================
% FIGURE 3: Engineering summary
% ==========================

figure
tiledlayout(1,3)

nexttile
bar(TripFuel_vec/1e3)
xticks(1:nLegs)
xticklabels({ExpandedLegs.Name})
xtickangle(45)
title('Fuel Burn per Leg')
grid on

nexttile
bar([ExpandedLegs.Range]/1000)
xticks(1:nLegs)
xticklabels({ExpandedLegs.Name})
xtickangle(45)
title('Effective Leg Ranges')
grid on

nexttile
axis off

text(0,0.90,sprintf('MTOM: %.1f t', ADP.MTOM/1e3))
text(0,0.80,sprintf('OEM: %.1f t', OEM/1e3))
text(0,0.70,sprintf('Payload: %.1f t', Payload/1e3))
text(0,0.60,sprintf('Fuel capacity: %.1f t', FuelCapacity/1e3))

text(0,0.45,sprintf('Design range: %.0f km', DesignRange/1000))
text(0,0.35,sprintf('Total range: %.0f km', TotalRange/1000))
text(0,0.25,sprintf('Effective legs: %d', nLegs))
text(0,0.15,sprintf('Stops: %d', nStops))

text(0.55,0.80,sprintf('Burn: %.1f t', TotalBurn/1e3))
text(0.55,0.60,sprintf('Carried: %.1f t', TotalCarried/1e3))
text(0.55,0.40,sprintf('Max res: %.1f t', MaxReserve/1e3))
text(0.55,0.20,sprintf('Min res: %.1f t', MinReserve/1e3))

title('Engineering Summary')

%% =========================
% Console
% ==========================

fprintf('\n----- Mission Summary -----\n')
fprintf('Fuel capacity: %.1f t\n', FuelCapacity/1e3)
fprintf('Burn: %.1f t\n', TotalBurn/1e3)
fprintf('Carried: %.1f t\n', TotalCarried/1e3)
fprintf('Max reserve: %.1f t\n', MaxReserve/1e3)
fprintf('Min reserve: %.1f t\n', MinReserve/1e3)
fprintf('Time: %.1f hr\n', TotalMissionTime/3600)