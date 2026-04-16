clear; clc; close all;

%% =========================
% 1. Size aircraft
% =========================
scripts.ExampleUnconventional

MTOM = ADP.MTOM;
OEM  = ADP.OEM;
Pmax = ADP.TLAR.Payload;
Rdes = ADP.TLAR.RangeDes;

UsefulLoad = MTOM - OEM;

%% =========================
% 2. Range sweep
% =========================
N = 25;
Range_vec = linspace(0, 3*Rdes, N);

Payload_vec = zeros(1,N);
Fuel_vec    = zeros(1,N);
WTO_vec     = zeros(1,N);

%% =========================
% 3. Mission loop
% =========================
for i = 1:N

    R = Range_vec(i);

    % --- start with max payload ---
    Payload = Pmax;

    % --- solve fuel ---
    Fuel = fuel_solve(R, ADP, Payload, OEM);

    % --- enforce useful load constraint ---
    if Payload + Fuel > UsefulLoad
        Payload = UsefulLoad - Fuel;
        Payload = max(Payload,0);

        % recompute fuel with reduced payload
        Fuel = fuel_solve(R, ADP, Payload, OEM);
    end

    % --- takeoff weight ---
    WTO = OEM + Payload + Fuel;

    % --- clamp to MTOM ---
    if WTO > MTOM
        WTO = MTOM;
    end

    % --- store ---
    Payload_vec(i) = Payload;
    Fuel_vec(i)    = Fuel;
    WTO_vec(i)     = WTO;

end

%% =========================
% 4. Plot
% =========================
figure; hold on; grid on

R_km = Range_vec / 1000;
Payload_t = Payload_vec / 1000;
Fuel_t    = Fuel_vec / 1000;
WTO_t     = WTO_vec / 1000;
OEM_t     = OEM / 1000;
MTOM_t    = MTOM / 1000;

plot(R_km, Payload_t, 'b-', 'LineWidth', 2)
plot(R_km, Fuel_t, 'r-', 'LineWidth', 2)
plot(R_km, WTO_t, 'k--', 'LineWidth', 2)

yline(OEM_t, 'g--', 'LineWidth', 2)
yline(MTOM_t, 'm-', 'LineWidth', 2)

xlabel('Range [km]')
ylabel('Mass [tonnes]')

legend('Payload','Fuel','Takeoff Weight','OEM','MTOM',...
    'Location','best')

title('Full Payload–Range Mass Breakdown')

%% =========================
% 5. Fuel solver (WITH CAP)
% =========================
function Fuel = fuel_solve(R, ADP, Payload, OEM)

    if R == 0
        Fuel = 0;
        return;
    end

    % -----------------------------
    % Fuel capacity limit (KEY LINE)
    % -----------------------------
    Fuel_cap = 0.4 * ADP.MTOM;   % ~A380 scaling (~250 tonnes)- CONNECT TO 630 WING VOLUME
    % WING VOLUME CONNECTION

    % initial guess
    Fuel = 0.2 * ADP.MTOM;

    for k = 1:6

        WTO = OEM + Payload + Fuel;

        [~, TripFuel, ResFuel] = ...
            Unconventional.MissionAnalysis_oscar(ADP, R, WTO);

        Fuel_new = TripFuel + ResFuel;

        % -----------------------------
        % APPLY FUEL CAP
        % -----------------------------
        Fuel_new = min(Fuel_new, Fuel_cap);

        % convergence
        if abs(Fuel_new - Fuel) < 1e-2
            Fuel = Fuel_new;
            return;
        end

        Fuel = Fuel_new;
    end
end