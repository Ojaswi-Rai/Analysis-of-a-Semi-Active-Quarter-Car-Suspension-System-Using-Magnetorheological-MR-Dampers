
%% ============================================================
% STAGE 6: FINAL PERFORMANCE ANALYSIS
% MR DAMPER QUARTER-CAR SUSPENSION PROJECT
%
% Input:
%   Stage5_Results.mat
%
% Outputs:
%   Stage6_Results.mat
%   Stage6_Performance_Summary.csv
%   Stage6_Improvements.csv
%   Stage6_Controller_Usage.csv
%
% MATLAB R2016b or newer
%% ============================================================

clear;
clc;
close all;

fprintf('\n============================================\n');
fprintf('STAGE 6: FINAL PERFORMANCE ANALYSIS\n');
fprintf('============================================\n');

%% 1. LOAD STAGE 5 RESULTS

if ~isfile('Stage5_Results.mat')
    error(['Stage5_Results.mat not found. ', ...
           'Place it in the current MATLAB folder.']);
end

S = load('Stage5_Results.mat');

requiredFields = {'results','metrics','cases', ...
                  'modeNames','p','dt'};

for k = 1:length(requiredFields)
    if ~isfield(S,requiredFields{k})
        error('Missing variable in Stage5_Results.mat: %s', ...
            requiredFields{k});
    end
end

results = S.results;
stage5Metrics = S.metrics;
cases = S.cases;
modeNames = S.modeNames;
p = S.p;
dt = S.dt;

numCases = size(results,1);
numModes = size(results,2);

fprintf('\nLoaded Stage 5 results successfully.\n');
fprintf('Road tests: %d\n',numCases);
fprintf('Suspension configurations: %d\n',numModes);

%% 2. INITIALIZE PERFORMANCE ARRAYS

% Metric order:
% 1 = Body displacement (mm)
% 2 = Peak body acceleration (m/s^2)
% 3 = RMS body acceleration (m/s^2)
% 4 = Suspension travel (mm)
% 5 = Tyre deflection (mm)

metricNames = { ...
    'BodyDisplacement_mm', ...
    'PeakAcceleration', ...
    'RMSAcceleration', ...
    'SuspensionTravel_mm', ...
    'TyreDeflection_mm'};

metricValues = zeros(numCases,numModes,5);

% Controller statistics:
% 1 = Average current after disturbance (A)
% 2 = RMS current after disturbance (A)
% 3 = Maximum current (A)
% 4 = Current integral (A.s)
% 5 = Current squared integral (A^2.s)
% 6 = Mechanical dissipated energy (J)

controllerStats = zeros(numCases,numModes,6);

minimumPower = zeros(numCases,numModes);

%% 3. RECALCULATE PERFORMANCE METRICS

fprintf('\nCalculating performance metrics...\n');

for c = 1:numCases

    for m = 1:numModes

        r = results{c,m};

        if isempty(r)
            error('Missing simulation result for case %d, mode %d.', ...
                c,m);
        end

        % Determine evaluation interval

        if strcmp(cases(c).type,'random')

            idx = true(size(r.t));

        else

            idx = r.t >= cases(c).start;

        end

        % Performance metrics

        bodyDisp = max(abs(r.xs(idx)));

        peakAcc = max(abs(r.accs(idx)));

        rmsAcc = sqrt(mean(r.accs(idx).^2));

        travel = max(abs(r.xs(idx)-r.xu(idx)));

        tyre = max(abs(r.xu(idx)-r.road(idx)));

        metricValues(c,m,:) = [ ...
            bodyDisp*1000, ...
            peakAcc, ...
            rmsAcc, ...
            travel*1000, ...
            tyre*1000];

        % Damper mechanical power
        %
        % With the force convention used in Stage 5:
        % Power = Damper force x Relative velocity.
        % Positive power represents energy dissipation.

        power = r.force .* r.vrel;

        minimumPower(c,m) = min(power(idx));

        dissipatedPower = max(power(idx),0);

        dissipatedEnergy = trapz( ...
            r.t(idx),dissipatedPower);

        % Controller current statistics

        I = r.current(idx);

        avgCurrent = mean(I);

        rmsCurrent = sqrt(mean(I.^2));

        maxCurrent = max(r.current);

        currentIntegral = trapz(r.t(idx),I);

        currentSquaredIntegral = trapz(r.t(idx),I.^2);

        controllerStats(c,m,:) = [ ...
            avgCurrent, ...
            rmsCurrent, ...
            maxCurrent, ...
            currentIntegral, ...
            currentSquaredIntegral, ...
            dissipatedEnergy];

    end

end

%% 4. FINAL PERFORMANCE SUMMARY TABLE

fprintf('\n============================================\n');
fprintf('FINAL PERFORMANCE SUMMARY\n');
fprintf('============================================\n');

summaryRows = cell(0,7);

for c = 1:numCases

    fprintf('\nROAD TEST: %s\n',cases(c).name);

    fprintf('%-12s %9s %9s %9s %10s %10s\n', ...
        'System','Disp','PeakAcc','RMSAcc','Travel','Tyre');

    fprintf('%s\n',repmat('-',1,68));

    for m = 1:numModes

        v = squeeze(metricValues(c,m,:));

        fprintf('%-12s %9.2f %9.3f %9.3f %10.2f %10.2f\n', ...
            modeNames{m}, ...
            v(1),v(2),v(3),v(4),v(5));

        summaryRows(end+1,:) = { ...
            cases(c).name, ...
            modeNames{m}, ...
            v(1),v(2),v(3),v(4),v(5)};

    end

end

performanceTable = cell2table(summaryRows, ...
    'VariableNames',{'RoadTest','System', ...
    'BodyDisplacement_mm','PeakAcceleration', ...
    'RMSAcceleration','SuspensionTravel_mm', ...
    'TyreDeflection_mm'});

%% 5. PERCENTAGE IMPROVEMENT OVER PASSIVE SUSPENSION

% Positive improvement = reduction in the metric.
% Negative improvement = deterioration in the metric.

fprintf('\n============================================\n');
fprintf('IMPROVEMENT OVER PASSIVE SUSPENSION\n');
fprintf('============================================\n');

improvementValues = zeros(numCases,numModes,5);

improvementRows = cell(0,7);

for c = 1:numCases

    passiveValues = squeeze(metricValues(c,1,:));

    fprintf('\nROAD TEST: %s\n',cases(c).name);

    fprintf('%-12s %10s %10s %10s %10s\n', ...
        'System','Disp(%)','PeakAcc(%)','RMSAcc(%)','Travel(%)');

    fprintf('%s\n',repmat('-',1,59));

    for m = 1:numModes

        currentValues = squeeze(metricValues(c,m,:));

        improvement = zeros(1,5);

        for q = 1:5

            if abs(passiveValues(q)) > eps

                improvement(q) = ...
                    (passiveValues(q)-currentValues(q)) ...
                    /passiveValues(q)*100;

            end

        end

        improvementValues(c,m,:) = improvement;

        fprintf('%-12s %10.2f %10.2f %10.2f %10.2f\n', ...
            modeNames{m}, ...
            improvement(1), ...
            improvement(2), ...
            improvement(3), ...
            improvement(4));

        improvementRows(end+1,:) = { ...
            cases(c).name, ...
            modeNames{m}, ...
            improvement(1), ...
            improvement(2), ...
            improvement(3), ...
            improvement(4), ...
            improvement(5)};

    end

end

improvementTable = cell2table(improvementRows, ...
    'VariableNames',{'RoadTest','System', ...
    'DisplacementImprovement_pct', ...
    'PeakAccelerationImprovement_pct', ...
    'RMSAccelerationImprovement_pct', ...
    'TravelImprovement_pct', ...
    'TyreDeflectionImprovement_pct'});

%% 6. IDENTIFY BEST CONFIGURATION FOR EACH ROAD TEST

fprintf('\n============================================\n');
fprintf('BEST CONFIGURATION FOR EACH ROAD TEST\n');
fprintf('============================================\n');

bestRows = cell(0,5);

for c = 1:numCases

    rmsValues = squeeze(metricValues(c,:,3));

    travelValues = squeeze(metricValues(c,:,4));

    tyreValues = squeeze(metricValues(c,:,5));

    [~,bestComfort] = min(rmsValues);

    [~,bestTravel] = min(travelValues);

    [~,bestTyre] = min(tyreValues);

    fprintf('\n%s\n',cases(c).name);

    fprintf('Best ride comfort (lowest RMS acceleration): %s\n', ...
        modeNames{bestComfort});

    fprintf('Lowest suspension travel: %s\n', ...
        modeNames{bestTravel});

    fprintf('Lowest tyre deflection: %s\n', ...
        modeNames{bestTyre});

    bestRows(end+1,:) = { ...
        cases(c).name, ...
        modeNames{bestComfort}, ...
        modeNames{bestTravel}, ...
        modeNames{bestTyre}, ...
        min(rmsValues)};

end

bestTable = cell2table(bestRows, ...
    'VariableNames',{'RoadTest','BestRideComfort', ...
    'LowestSuspensionTravel','LowestTyreDeflection', ...
    'MinimumRMSAcceleration'});

%% 7. CONTROLLER CURRENT AND ENERGY ANALYSIS

fprintf('\n============================================\n');
fprintf('CONTROLLER CURRENT AND ENERGY ANALYSIS\n');
fprintf('============================================\n');

usageRows = cell(0,9);

for c = 1:numCases

    fprintf('\n%s\n',cases(c).name);

    for m = 1:numModes

        v = squeeze(controllerStats(c,m,:));

        fprintf('%-12s Avg I = %.4f A | RMS I = %.4f A | Max I = %.4f A | Dissipated = %.2f J\n', ...
            modeNames{m},v(1),v(2),v(3),v(6));

        usageRows(end+1,:) = { ...
            cases(c).name, ...
            modeNames{m}, ...
            v(1),v(2),v(3),v(4),v(5),v(6), ...
            minimumPower(c,m)};

    end

end

usageTable = cell2table(usageRows, ...
    'VariableNames',{'RoadTest','System', ...
    'AverageCurrent_A','RMSCurrent_A','MaximumCurrent_A', ...
    'CurrentIntegral_As','CurrentSquaredIntegral_A2s', ...
    'MechanicalDissipatedEnergy_J','MinimumPower_W'});

%% 8. OVERALL RMS ACCELERATION COMPARISON

figure('Name','Stage 6 RMS Acceleration','Color','w');

bar(metricValues(:,:,3));

title('RMS Body Acceleration Across Road Conditions');

xlabel('Road Test');

ylabel('RMS Acceleration (m/s^2)');

set(gca,'XTick',1:numCases);
set(gca,'XTickLabel',{cases.name});

xtickangle(25);

legend(modeNames,'Location','best');

grid on;

%% 9. OVERALL SUSPENSION TRAVEL COMPARISON

figure('Name','Stage 6 Suspension Travel','Color','w');

bar(metricValues(:,:,4));

title('Peak Suspension Travel Across Road Conditions');

xlabel('Road Test');

ylabel('Suspension Travel (mm)');

set(gca,'XTick',1:numCases);
set(gca,'XTickLabel',{cases.name});

xtickangle(25);

legend(modeNames,'Location','best');

grid on;

%% 10. OVERALL TYRE DEFLECTION COMPARISON

figure('Name','Stage 6 Tyre Deflection','Color','w');

bar(metricValues(:,:,5));

title('Peak Tyre Deflection Across Road Conditions');

xlabel('Road Test');

ylabel('Tyre Deflection (mm)');

set(gca,'XTick',1:numCases);
set(gca,'XTickLabel',{cases.name});

xtickangle(25);

legend(modeNames,'Location','best');

grid on;

%% 11. RMS ACCELERATION IMPROVEMENT

figure('Name','Stage 6 Ride Comfort Improvement','Color','w');

bar(improvementValues(:,:,3));

title('Ride Comfort Improvement Over Passive Suspension');

xlabel('Road Test');

ylabel('RMS Acceleration Improvement (%)');

set(gca,'XTick',1:numCases);
set(gca,'XTickLabel',{cases.name});

xtickangle(25);

legend(modeNames,'Location','best');

grid on;

%% 12. CONTROLLER CURRENT COMPARISON

figure('Name','Stage 6 Average Current','Color','w');

hold on;

for m = 3:4

    plot(1:numCases,controllerStats(:,m,1), ...
        '-o','LineWidth',1.5,'MarkerSize',6);

end

title('Average Controller Current Across Road Conditions');

xlabel('Road Test');

ylabel('Average Current (A)');

set(gca,'XTick',1:numCases);
set(gca,'XTickLabel',{cases.name});

xtickangle(25);

legend(modeNames(3:4),'Location','best');

grid on;

%% 13. RANDOM ROAD ACCELERATION COMPARISON

figure('Name','Stage 6 Random Road Response','Color','w');

hold on;

for m = 1:numModes

    r = results{8,m};

    plot(r.t,r.accs,'LineWidth',1.1);

end

title('Random Road C: Body Acceleration Comparison');

xlabel('Time (s)');

ylabel('Body Acceleration (m/s^2)');

legend(modeNames,'Location','best');

grid on;

%% 14. RANDOM ROAD PROFILE

figure('Name','Stage 6 Random Road Profile','Color','w');

r = results{8,1};

plot(r.t,r.road,'LineWidth',1.2);

title('Class C Inspired Random Road Profile');

xlabel('Time (s)');

ylabel('Road Displacement (m)');

grid on;

%% 15. SEMI-ACTIVE DISSIPATION VALIDATION

fprintf('\n============================================\n');
fprintf('SEMI-ACTIVE DISSIPATION VALIDATION\n');
fprintf('============================================\n');

for c = 1:numCases

    for m = 1:numModes

        fprintf('%-18s %-12s Minimum power = %.8f W\n', ...
            cases(c).name, ...
            modeNames{m}, ...
            minimumPower(c,m));

    end

end

if any(minimumPower(:) < -1e-8)

    warning(['Negative damper power detected. ', ...
        'Check force and velocity sign conventions.']);

else

    fprintf('\nAll configurations satisfy the dissipation check.\n');

end

%% 16. SAVE FINAL RESULTS

save('Stage6_Results.mat', ...
    'metricValues', ...
    'improvementValues', ...
    'controllerStats', ...
    'minimumPower', ...
    'performanceTable', ...
    'improvementTable', ...
    'usageTable', ...
    'bestTable', ...
    'cases', ...
    'modeNames', ...
    'p', ...
    'dt');

writetable(performanceTable, ...
    'Stage6_Performance_Summary.csv');

writetable(improvementTable, ...
    'Stage6_Improvements.csv');

writetable(usageTable, ...
    'Stage6_Controller_Usage.csv');

writetable(bestTable, ...
    'Stage6_Best_Configurations.csv');

%% 17. FINAL COMPLETION MESSAGE

fprintf('\n============================================\n');
fprintf('STAGE 6 COMPLETED SUCCESSFULLY\n');
fprintf('============================================\n');

fprintf('\nGenerated files:\n');

fprintf('1. Stage6_Results.mat\n');
fprintf('2. Stage6_Performance_Summary.csv\n');
fprintf('3. Stage6_Improvements.csv\n');
fprintf('4. Stage6_Controller_Usage.csv\n');
fprintf('5. Stage6_Best_Configurations.csv\n');

fprintf('\nALL SIX SIMULATION STAGES ARE COMPLETE.\n');

fprintf('============================================\n');
