

%% ==================== AIC MODEL ====================

% -------- INPUTS --------
% h_profile_ft : vector of altitudes along mission [ft]
% L_total_nm   : total distance flown per year [nautical miles]

% Example (REMOVE when integrating)
% h_profile_ft = linspace(25000, 38000, 100);
% L_total_nm   = 1e7;

% -------- CONSTANT --------
RF_per_L_AIC = 2.21e-12;   % (W/m^2) per nautical mile


%% -------- GET AIC FORCING FACTOR --------
s_AIC_profile = get_s_AIC(h_profile_ft);


%% -------- AVERAGE OVER PROFILE --------
% simple mean is fine for now (can weight by distance later if needed)
s_AIC_avg = mean(s_AIC_profile);


%% -------- COMPUTE RF --------
RF_AIC = s_AIC_avg * RF_per_L_AIC * L_total_nm;


%% -------- OUTPUT --------
fprintf('AIC forcing factor (avg): %.3f\n', s_AIC_avg);
fprintf('RF_AIC: %.3e W/m^2\n', RF_AIC);


%% ==================== FUNCTION ====================
function s = get_s_AIC(h)

    % Digitised data from Dallara (2011) graph
    h_data = [ ...
        17000 18000 20000 22000 24000 26000 28000 ...
        30000 32000 34000 36000 38000 40000 42000];
    
    s_data = [ ...
        0.02 0.05 0.05 0.10 0.25 0.50 0.80 ...
        1.10 1.60 2.10 1.80 1.50 1.20 0.90];

    s = interp1(h_data, s_data, h, 'pchip');

    % Clamp outside range manually (better than extrap)
    s(h < min(h_data)) = s_data(1);
    s(h > max(h_data)) = s_data(end);
    
    % Physical clamp
    s(s < 0) = 0;

    % Interpolation
    s = interp1(h_data, s_data, h, 'pchip', 'extrap');

    % Clamp to physical values
    s(s < 0) = 0;

end


function s = get_s_O3S(h)

    h_data = [17000 19000 21000 23000 25000 27000 29000 ...
              31000 33000 35000 37000 39000 41000];

    s_data = [0.45 0.50 0.55 0.60 0.70 0.80 0.95 ...
              1.10 1.25 1.40 1.60 1.80 2.00];

    s = interp1(h_data, s_data, h, 'pchip');

    s(h < min(h_data)) = s_data(1);
    s(h > max(h_data)) = s_data(end);

end

function s = get_s_CH4_O3L(h)

    h_data = [17000 19000 21000 23000 25000 27000 29000 ...
              31000 33000 35000 37000 39000 41000];

    s_data = [0.90 0.90 0.90 0.92 0.95 0.98 1.00 ...
              1.05 1.10 1.15 1.20 1.20 1.20];

    s = interp1(h_data, s_data, h, 'pchip');

    s(h < min(h_data)) = s_data(1);
    s(h > max(h_data)) = s_data(end);

end



%% ==================== PLOT s_AIC(h) ====================



h_plot = linspace(16000, 42000, 300);

s_AIC   = get_s_AIC(h_plot);
s_O3S   = get_s_O3S(h_plot);
s_CH4   = get_s_CH4_O3L(h_plot);

figure;
plot(s_O3S, h_plot, 'b', 'LineWidth', 2); hold on;
plot(s_CH4, h_plot, 'g', 'LineWidth', 2);
plot(s_AIC, h_plot, 'r', 'LineWidth', 2);

legend('O_{3S}', 'CH_4 & O_{3L}', 'AIC');

xlabel('Forcing factor s');
ylabel('Altitude [ft]');
title('Altitude-dependent forcing factors');
grid on;

xlim([0 2.5]);
ylim([16000 42000]);