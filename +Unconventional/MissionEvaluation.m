%% Evaluate mission fuel evolution (Block Fuel) at MTOM only

clear
clc

%% Run sizing first
scripts.ExampleSizing

%% =========================
% Mission definition
% ==========================

dist_km = [
16909.38
8017.62
1456.91
8052.37
1272.47
11621.60
2264.78
6128.98
496.31
1369.73
1275.10
350.45
1139.27
1169.25
796.67
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

%% =========================
% Split legs based on aircraft range
% ==========================

Range_max = ADP.TLAR.RangeDes;
DesignRange = ADP.TLAR.RangeDes;

ExpandedLegs = struct([]);
idx = 1;

for i = 1:nOriginalLegs
    
    totalRange = dist_km(i) * 1e3;
    
    if totalRange <= Range_max
        
        ExpandedLegs(idx).Name  = "Leg " + i;
        ExpandedLegs(idx).Range = totalRange;
        idx = idx + 1;
        
    else
        
        nSub = ceil(totalRange / Range_max);
        subRange = totalRange / nSub;
        
        for j = 1:nSub
            ExpandedLegs(idx).Name  = "Leg " + i + "." + j;
            ExpandedLegs(idx).Range = subRange;
            idx = idx + 1;
        end
        
    end
end

nLegs = length(ExpandedLegs);
nStops = nLegs - 1;
TotalRange = sum([ExpandedLegs.Range]);

fprintf('\n----- Mission Setup -----\n')
fprintf('Original legs: %d\n', nOriginalLegs)
fprintf('Effective legs: %d\n', nLegs)
fprintf('Stops: %d\n', nStops)
fprintf('Design range: %.0f km\n', DesignRange/1000)
fprintf('Total mission range: %.0f km\n', TotalRange/1000)

%% =========================
% Mission evaluation
% ==========================

fuelTimeline = [];
labelTimeline = [];

TripFuel_vec = [];
BlockFuel_vec = [];
ResFuel_vec = [];

MissionTimeA = 0;

for i = 1:nLegs
    
    legRange = ExpandedLegs(i).Range;
    legName  = ExpandedLegs(i).Name;
    
    [BlockFuel, TripFuel, ResFuel, ~, MissionTime] = ...
        B777.MissionAnalysis(ADP, legRange, ADP.MTOM);
    
    % Store
    TripFuel_vec  = [TripFuel_vec, TripFuel];
    BlockFuel_vec = [BlockFuel_vec, BlockFuel];
    ResFuel_vec   = [ResFuel_vec, ResFuel];
    
    MissionTimeA = MissionTimeA + MissionTime;
    
    % Timeline (ONLY Start → End → Reserve)
    fuelTimeline = [fuelTimeline, ...
        BlockFuel, ...
        BlockFuel - TripFuel, ...
        ResFuel];
    
    labelTimeline = [labelTimeline, ...
        legName + " Start", ...
        legName + " End", ...
        legName + " Reserve"];
end

%% =========================
% Derived metrics
% ==========================

TotalBurn    = sum(TripFuel_vec);
MaxReserve   = max(ResFuel_vec);
MinReserve   = min(ResFuel_vec);
TotalCarried = TotalBurn + MaxReserve;

%% =========================
% FIGURE 3: Fuel timeline ONLY
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
% DEBUG DASHBOARD (UNCHANGED STRUCTURE)
% ==========================

FuelStart = BlockFuel_vec;
FuelEnd   = BlockFuel_vec - TripFuel_vec;
ReserveMargin = FuelEnd - ResFuel_vec;

RefuelAmount = zeros(1,nLegs);
for i = 2:nLegs
    RefuelAmount(i) = BlockFuel_vec(i) - ResFuel_vec(i-1);
end

figure
tiledlayout(2,2)

% Start vs End
nexttile
bar([FuelStart'/1e3, FuelEnd'/1e3])
legend('Start','End')
xticks(1:nLegs)
xticklabels({ExpandedLegs.Name})
xtickangle(45)
ylabel('Fuel [t]')
title('Fuel Before and After Each Leg')
grid on

% Required uplift (kept for insight)
nexttile
bar(RefuelAmount/1e3)
xticks(1:nLegs)
xticklabels({ExpandedLegs.Name})
xtickangle(45)
ylabel('Fuel Added [t]')
title('Required Fuel Uplift per Leg')
grid on

% Reserve
nexttile
plot(ResFuel_vec/1e3,'-o','LineWidth',1.5)
xticks(1:nLegs)
xticklabels({ExpandedLegs.Name})
xtickangle(45)
ylabel('Reserve [t]')
title('Reserve per Leg')
grid on

% Consistency
nexttile
bar(ReserveMargin/1e3)
xticks(1:nLegs)
xticklabels({ExpandedLegs.Name})
xtickangle(45)
ylabel('Fuel Margin [t]')
title('End Fuel - Reserve (Check)')
grid on

%% =========================
% ENGINEERING SUMMARY
% ==========================

figure
tiledlayout(1,3)

% Burn
nexttile
bar(TripFuel_vec/1e3)
xticks(1:nLegs)
xticklabels({ExpandedLegs.Name})
xtickangle(45)
title('Fuel Burn per Leg')
grid on

% Range
nexttile
bar([ExpandedLegs.Range]/1000)
xticks(1:nLegs)
xticklabels({ExpandedLegs.Name})
xtickangle(45)
title('Effective Leg Ranges')
grid on

% Summary
nexttile
axis off

text(0,0.85,sprintf('MTOM: %.1f t', ADP.MTOM/1e3))
text(0,0.75,sprintf('Design range: %.0f km', DesignRange/1000))
text(0,0.65,sprintf('Total range: %.0f km', TotalRange/1000))
text(0,0.55,sprintf('Original legs: %d', nOriginalLegs))
text(0,0.45,sprintf('Effective legs: %d', nLegs))
text(0,0.35,sprintf('Stops: %d', nStops))
text(0,0.25,sprintf('Time: %.1f hr', MissionTimeA/3600))

text(0.55,0.75,sprintf('Burn: %.1f t', TotalBurn/1e3))
text(0.55,0.55,sprintf('Carried: %.1f t', TotalCarried/1e3))
text(0.55,0.35,sprintf('Max res: %.1f t', MaxReserve/1e3))
text(0.55,0.15,sprintf('Min res: %.1f t', MinReserve/1e3))

title('Engineering Summary')

%% =========================
% Console
% ==========================

fprintf('\n----- Mission Summary -----\n')
fprintf('Burn: %.1f t\n', TotalBurn/1e3)
fprintf('Carried: %.1f t\n', TotalCarried/1e3)
fprintf('Max reserve: %.1f t\n', MaxReserve/1e3)
fprintf('Min reserve: %.1f t\n', MinReserve/1e3)
fprintf('Time: %.1f hr\n', MissionTimeA/3600)