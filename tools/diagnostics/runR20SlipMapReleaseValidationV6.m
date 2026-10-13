function report = runR20SlipMapReleaseValidationV6(options)
%RUNR20SLIPMAPRELEASEVALIDATIONV6 Create the frozen numerical reference.
% Applies a conservative local branch policy and cell correction to V5,
% verifies fixed release criteria, and stores the complete reference state.

arguments
    options.SourceFile (1,1) string = ""
end
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(root,fullfile(root,'vehicles'),fullfile(root,'config'));
sourceFile = options.SourceFile;
if sourceFile == ""
    sourceFile = fullfile(root,'data','reference','r20SlipMapBoundaryStudyV5.mat');
end
assert(isfile(sourceFile), 'lts:io:ReleaseSource', ...
    ['The historical V5 source is not included. Use the frozen input via ' ...
     'lts.io.loadR20Reference, or supply SourceFile to rebuild: %s'],sourceFile);
source = load(sourceFile, 'report');
v5 = source.report;
vehicle = v5.vehicle;
events = manualFsgEvents();
rules = lts.config.scoring.fsg2026(referenceTimes());

criteria.maximumLateralOptimism_percent = 1.0;
criteria.maximumLongitudinalOptimism_percent = 5.0;
criteria.minimumRelativeBoundary_mps2 = 0.5;
criteria.maximumSmallBoundaryOptimism_mps2 = 0.1;
criteria.maximumIterationDifference_percent = 1.0;
criteria.maximumEventRegression_percent = 0.5;
criteria.correctionMargin = 0.995;

folder = fullfile(root, 'results');
resultFile = lts.v2.nextResultFile(string(folder), ...
    "r20SlipMapReleaseValidationV6");
fprintf('\nR20 numerical reference release V6\n');
fprintf('Source: %s\n', sourceFile);
fprintf('Result: %s\n', resultFile);

releaseFractions = unique(sort([v5.v5Map.lateralFractions, ...
    v5.branchScan.lateralFraction.']));
releaseMap = expandFractions(v5.v5Map, releaseFractions);
[releaseMap, branchPolicy] = applyConservativeBranchPolicy( ...
    releaseMap, v5.branchScan);
[releaseMap, correctionLog] = applyLocalCorrections( ...
    releaseMap, v5.directAudit, criteria);
releaseMap.meta.releaseVersion = 'V6';
releaseMap.meta.releasePolicy = [ ...
    'Minimum feasible retry/no-retry branch at scanned low-speed nodes; ' ...
    'local cell correction against relative and absolute audit tolerances.'];
releaseMap.meta.interpolationSafetyFactor = 1.0;

releaseAudit = evaluateAudit(vehicle, releaseMap, v5.directAudit, criteria);
auditSummary = summarizeAudit(releaseAudit, criteria);
branchVerification = verifyBranchPolicy(releaseMap, v5.branchScan);
v5Case = runCase(vehicle, v5.v5Map, events, rules, "V5 boundary refined");
v6Case = runCase(vehicle, releaseMap, events, rules, "V6 numerical reference");
eventComparison = comparisonTable(v5Case, v6Case);
releaseCriteria = evaluateReleaseCriteria(auditSummary, ...
    v5.continuitySummary, branchVerification, eventComparison, criteria);
releasePassed = all(releaseCriteria.Passed);

manifest.releaseName = 'R20 Slip Map Numerical Reference V6';
manifest.created = datetime('now', 'TimeZone', 'local');
manifest.sourceFile = sourceFile;
manifest.scriptFile = fullfile(root, mfilename + ".m");
manifest.model = v5.meta.model;
manifest.grid = sprintf('%dx%d', numel(releaseMap.speedGrid_mps), ...
    numel(releaseMap.lateralFractions));
manifest.interpolationSafetyFactor = 1.0;
manifest.branchPolicy = releaseMap.meta.releasePolicy;
manifest.releasePassed = releasePassed;
manifest.vehicleDefinitionStored = true;
manifest.mapStored = true;
manifest.note = [ ...
    'Frozen means versioned and immutable by convention. Future changes ' ...
    'must use a new version and compare against this stored reference.'];

report.meta.created = manifest.created;
report.meta.sourceFile = sourceFile;
report.meta.status = ternary(releasePassed, 'RELEASED', 'NOT RELEASED');
report.meta.criteria = criteria;
report.manifest = manifest;
report.vehicle = vehicle;
report.releaseMap = releaseMap;
report.branchPolicy = branchPolicy;
report.correctionLog = correctionLog;
report.releaseAudit = releaseAudit;
report.auditSummary = auditSummary;
report.branchVerification = branchVerification;
report.v5Case = v5Case;
report.v6Case = v6Case;
report.eventComparison = eventComparison;
report.releaseCriteria = releaseCriteria;
report.releasePassed = releasePassed;

[resultFolder, stem] = fileparts(resultFile);
auditCsv = fullfile(resultFolder, stem + "_audit.csv");
auditSummaryCsv = fullfile(resultFolder, stem + "_auditSummary.csv");
branchCsv = fullfile(resultFolder, stem + "_branchPolicy.csv");
correctionCsv = fullfile(resultFolder, stem + "_corrections.csv");
comparisonCsv = fullfile(resultFolder, stem + "_comparison.csv");
criteriaCsv = fullfile(resultFolder, stem + "_releaseCriteria.csv");
manifestFile = fullfile(resultFolder, stem + "_manifest.txt");
report.files = struct('mat', resultFile, 'auditCsv', auditCsv, ...
    'auditSummaryCsv', auditSummaryCsv, 'branchPolicyCsv', branchCsv, ...
    'correctionCsv', correctionCsv, 'comparisonCsv', comparisonCsv, ...
    'releaseCriteriaCsv', criteriaCsv, 'manifest', manifestFile);
save(resultFile, 'report', '-v7.3');
writetable(releaseAudit, auditCsv);
writetable(auditSummary, auditSummaryCsv);
writetable(branchVerification, branchCsv);
writetable(correctionLog, correctionCsv);
writetable(eventComparison, comparisonCsv);
writetable(releaseCriteria, criteriaCsv);
writeManifest(manifestFile, manifest, releaseCriteria);
disp(auditSummary);
disp(branchVerification);
disp(eventComparison);
disp(releaseCriteria);
fprintf('V6 numerical reference status: %s\n', report.meta.status);
fprintf('Saved: %s\n', resultFile);
end

function map = expandFractions(source, fractions)
map = source;
speedCount = numel(source.speedGrid_mps);
directionCount = numel(source.turnDirections);
fractionCount = numel(fractions);
map.lateralFractions = fractions;
map.maximumAcceleration_mps2 = zeros(speedCount, fractionCount, directionCount);
map.maximumDeceleration_mps2 = zeros(speedCount, fractionCount, directionCount);
map.boundaryStates.acceleration = cell(speedCount, fractionCount, directionCount);
map.boundaryStates.deceleration = cell(speedCount, fractionCount, directionCount);
for speedIndex = 1:speedCount
    for directionIndex = 1:directionCount
        map.maximumAcceleration_mps2(speedIndex,:,directionIndex) = interp1( ...
            source.lateralFractions, ...
            squeeze(source.maximumAcceleration_mps2(speedIndex,:,directionIndex)), ...
            fractions, 'linear');
        map.maximumDeceleration_mps2(speedIndex,:,directionIndex) = interp1( ...
            source.lateralFractions, ...
            squeeze(source.maximumDeceleration_mps2(speedIndex,:,directionIndex)), ...
            fractions, 'linear');
    end
end
map.meta.boundaryStatesStored = false;
end

function [map, policy] = applyConservativeBranchPolicy(map, scan)
rowCount = height(scan);
speed_mps = scan.speed_mps;
lateralFraction = scan.lateralFraction;
retryBoundary_mps2 = scan.brakeRetry20_mps2;
noRetryBoundary_mps2 = scan.brakeNoRetry20_mps2;
selectedBoundary_mps2 = min(retryBoundary_mps2, noRetryBoundary_mps2);
originalMapBoundary_mps2 = NaN(rowCount,1);
releasedMapBoundary_mps2 = NaN(rowCount,1);
for row = 1:rowCount
    speedIndex = find(abs(map.speedGrid_mps - speed_mps(row)) < 1e-9, 1);
    fractionIndex = find(abs(map.lateralFractions - lateralFraction(row)) < 1e-12, 1);
    assert(~isempty(speedIndex) && ~isempty(fractionIndex), ...
        'Branch policy point is absent from release grid.');
    originalMapBoundary_mps2(row) = max( ...
        map.maximumDeceleration_mps2(speedIndex,fractionIndex,:));
    for directionIndex = 1:numel(map.turnDirections)
        map.maximumDeceleration_mps2(speedIndex,fractionIndex,directionIndex) = min( ...
            map.maximumDeceleration_mps2(speedIndex,fractionIndex,directionIndex), ...
            selectedBoundary_mps2(row));
    end
    releasedMapBoundary_mps2(row) = max( ...
        map.maximumDeceleration_mps2(speedIndex,fractionIndex,:));
end
policy = table(speed_mps, lateralFraction, retryBoundary_mps2, ...
    noRetryBoundary_mps2, selectedBoundary_mps2, ...
    originalMapBoundary_mps2, releasedMapBoundary_mps2);
end

function [map, logTable] = applyLocalCorrections(map, audit, criteria)
quantity = strings(0,1); speed_mps = zeros(0,1); lateralFraction = zeros(0,1);
directBoundary_mps2 = zeros(0,1); before_mps2 = zeros(0,1);
allowed_mps2 = zeros(0,1); appliedScale = zeros(0,1);
for pass = 1:3
    model = lts.model.dualTrackSlip.create(map.meta.sourceVehicleDefinition, map);
    for row = 1:height(audit)
        speed = audit.speed_mps(row);
        ay = audit.lateralFraction(row) * audit.directLateral_mps2(row);
        limits = model.accelerationLimits(speed, ay);
        pairs = {"acceleration", audit.directAcceleration_mps2(row), ...
            limits.maximumAcceleration_mps2; ...
            "deceleration", audit.directDeceleration_mps2(row), ...
            limits.maximumDeceleration_mps2};
        for pairIndex = 1:size(pairs,1)
            name = pairs{pairIndex,1}; direct = pairs{pairIndex,2}; current = pairs{pairIndex,3};
            allowed = allowedBoundary(direct, criteria) * criteria.correctionMargin;
            if current <= allowed + 1e-12 || current <= 0, continue; end
            scale = allowed / current;
            fraction = queryFraction(map, speed, ay);
            [speedIndices, fractionIndices] = cellCorners(map, speed, fraction);
            field = "maximumAcceleration_mps2";
            if name == "deceleration", field = "maximumDeceleration_mps2"; end
            values = map.(field);
            values(speedIndices,fractionIndices,:) = min( ...
                values(speedIndices,fractionIndices,:), ...
                scale * values(speedIndices,fractionIndices,:));
            map.(field) = values;
            quantity(end+1,1) = name; speed_mps(end+1,1) = speed; %#ok<AGROW>
            lateralFraction(end+1,1) = audit.lateralFraction(row); %#ok<AGROW>
            directBoundary_mps2(end+1,1) = direct; before_mps2(end+1,1) = current; %#ok<AGROW>
            allowed_mps2(end+1,1) = allowed; appliedScale(end+1,1) = scale; %#ok<AGROW>
            model = lts.model.dualTrackSlip.create(map.meta.sourceVehicleDefinition, map);
        end
    end
end
logTable = table(quantity, speed_mps, lateralFraction, directBoundary_mps2, ...
    before_mps2, allowed_mps2, appliedScale);
end

function audit = evaluateAudit(vehicle, map, sourceAudit, criteria)
audit = sourceAudit;
model = lts.model.dualTrackSlip.create(vehicle, map);
for row = 1:height(audit)
    speed = audit.speed_mps(row);
    ay = audit.lateralFraction(row) * audit.directLateral_mps2(row);
    limits = model.accelerationLimits(speed, ay);
    audit.mapLateral_mps2(row) = interp1(map.speedGrid_mps, ...
        map.maximumLateralAcceleration_mps2(:,2), speed, 'linear');
    audit.mapAcceleration_mps2(row) = limits.maximumAcceleration_mps2;
    audit.mapDeceleration_mps2(row) = limits.maximumDeceleration_mps2;
    audit.lateralMapOverDirect(row) = safeRatio( ...
        audit.mapLateral_mps2(row), audit.directLateral_mps2(row));
    audit.accelerationMapOverDirect(row) = safeRatio( ...
        audit.mapAcceleration_mps2(row), audit.directAcceleration_mps2(row));
    audit.decelerationMapOverDirect(row) = safeRatio( ...
        audit.mapDeceleration_mps2(row), audit.directDeceleration_mps2(row));
end
audit.accelerationAllowed_mps2 = arrayfun( ...
    @(value) allowedBoundary(value, criteria), audit.directAcceleration_mps2);
audit.decelerationAllowed_mps2 = arrayfun( ...
    @(value) allowedBoundary(value, criteria), audit.directDeceleration_mps2);
audit.accelerationPass = audit.mapAcceleration_mps2 <= audit.accelerationAllowed_mps2 + 1e-9;
audit.decelerationPass = audit.mapDeceleration_mps2 <= audit.decelerationAllowed_mps2 + 1e-9;
end

function summary = summarizeAudit(audit, criteria)
metric = ["lateral optimism"; "acceleration relative optimism"; ...
    "deceleration relative optimism"; "acceleration small-boundary absolute optimism"; ...
    "deceleration small-boundary absolute optimism"];
measured = [100 * max(audit.lateralMapOverDirect - 1); ...
    relativeMaximum(audit.directAcceleration_mps2, audit.mapAcceleration_mps2, criteria); ...
    relativeMaximum(audit.directDeceleration_mps2, audit.mapDeceleration_mps2, criteria); ...
    absoluteMaximum(audit.directAcceleration_mps2, audit.mapAcceleration_mps2, criteria); ...
    absoluteMaximum(audit.directDeceleration_mps2, audit.mapDeceleration_mps2, criteria)];
limit = [criteria.maximumLateralOptimism_percent; ...
    criteria.maximumLongitudinalOptimism_percent; ...
    criteria.maximumLongitudinalOptimism_percent; ...
    criteria.maximumSmallBoundaryOptimism_mps2; ...
    criteria.maximumSmallBoundaryOptimism_mps2];
unit = ["percent"; "percent"; "percent"; "m/s^2"; "m/s^2"];
passed = measured <= limit + 1e-9;
summary = table(metric, measured, limit, unit, passed);
end

function value = relativeMaximum(direct, mapped, criteria)
mask = direct >= criteria.minimumRelativeBoundary_mps2;
value = max([0; 100 * (mapped(mask) ./ direct(mask) - 1)]);
end

function value = absoluteMaximum(direct, mapped, criteria)
mask = direct < criteria.minimumRelativeBoundary_mps2;
value = max([0; mapped(mask) - direct(mask)]);
end

function allowed = allowedBoundary(direct, criteria)
if direct >= criteria.minimumRelativeBoundary_mps2
    allowed = direct * (1 + criteria.maximumLongitudinalOptimism_percent / 100);
else
    allowed = direct + criteria.maximumSmallBoundaryOptimism_mps2;
end
end

function verification = verifyBranchPolicy(map, scan)
rowCount = height(scan); releasedBoundary_mps2 = NaN(rowCount,1);
selectedBoundary_mps2 = min(scan.brakeRetry20_mps2, scan.brakeNoRetry20_mps2);
for row = 1:rowCount
    speedIndex = find(abs(map.speedGrid_mps - scan.speed_mps(row)) < 1e-9, 1);
    fractionIndex = find(abs(map.lateralFractions - scan.lateralFraction(row)) < 1e-12, 1);
    releasedBoundary_mps2(row) = max( ...
        map.maximumDeceleration_mps2(speedIndex,fractionIndex,:));
end
passed = releasedBoundary_mps2 <= selectedBoundary_mps2 + 1e-9;
verification = table(scan.speed_mps, scan.lateralFraction, ...
    selectedBoundary_mps2, releasedBoundary_mps2, passed, ...
    'VariableNames', {'speed_mps','lateralFraction','selectedBoundary_mps2', ...
    'releasedBoundary_mps2','passed'});
end

function output = evaluateReleaseCriteria(auditSummary, continuity, branch, events, criteria)
Criterion = ["audit limits"; "iteration convergence"; ...
    "conservative branch policy"; "event regression"];
Measured = [double(all(auditSummary.passed)); ...
    continuity.maximumDifference_percent(continuity.metric == "14 vs 20 iterations"); ...
    double(all(branch.passed)); ...
    max(abs([events.AccelerationChange_percent(2), events.SkidpadChange_percent(2), ...
    events.AutocrossChange_percent(2), events.EnduranceChange_percent(2)]))];
Limit = [1; criteria.maximumIterationDifference_percent; 1; ...
    criteria.maximumEventRegression_percent];
Unit = ["boolean"; "percent"; "boolean"; "percent"];
Passed = [Measured(1) == 1; Measured(2) <= Limit(2); ...
    Measured(3) == 1; Measured(4) <= Limit(4)];
output = table(Criterion, Measured, Limit, Unit, Passed);
end

function fraction = queryFraction(map, speed, ay)
lat = interp1(map.speedGrid_mps, map.maximumLateralAcceleration_mps2(:,2), ...
    speed, 'linear');
fraction = min(1, max(0, abs(ay) / lat));
end

function [speedIndices, fractionIndices] = cellCorners(map, speed, fraction)
speedIndices = bracket(map.speedGrid_mps, speed);
fractionIndices = bracket(map.lateralFractions, fraction);
end

function indices = bracket(nodes, value)
upper = find(nodes >= value, 1, 'first');
if isempty(upper), upper = numel(nodes); end
lower = max(1, upper - 1);
if lower == upper && upper < numel(nodes), upper = upper + 1; end
indices = unique([lower, upper]);
end

function item = runCase(vehicle, map, events, rules, name)
model = lts.model.dualTrackSlip.create(vehicle, map);
eventResults = lts.events.runSet(events, model, IncludeDriverless=false, CoupleEnergy=false);
scoring = lts.scoring.scoreEvents(eventResults, rules, IncludeDriverless=false);
item = struct('name', name, 'eventResults', eventResults, 'scoring', scoring);
end

function output = comparisonTable(baseCase, releaseCase)
caseName = string({baseCase.name; releaseCase.name}); cases = [baseCase, releaseCase];
acceleration_s = metric(cases, 'acceleration', false);
skidpad_s = metric(cases, 'skidpad', false);
autocross_s = metric(cases, 'autocross', false);
endurance_s = metric(cases, 'endurance', false);
enduranceEnergy_kWh = metric(cases, 'endurance', true);
dynamicPoints = arrayfun(@(item) item.scoring.totalPoints, cases).';
reference = [acceleration_s(1), skidpad_s(1), autocross_s(1), endurance_s(1)];
changes = 100 * ([acceleration_s, skidpad_s, autocross_s, endurance_s] ./ reference - 1);
output = table(caseName, acceleration_s, skidpad_s, autocross_s, endurance_s, ...
    enduranceEnergy_kWh, dynamicPoints, changes(:,1), changes(:,2), changes(:,3), ...
    changes(:,4), 'VariableNames', {'CaseName','Acceleration_s','Skidpad_s', ...
    'Autocross_s','Endurance_s','EnduranceEnergy_kWh','DynamicPoints', ...
    'AccelerationChange_percent','SkidpadChange_percent', ...
    'AutocrossChange_percent','EnduranceChange_percent'});
end

function writeManifest(file, manifest, criteria)
fid = fopen(file, 'w'); cleanup = onCleanup(@() fclose(fid));
fprintf(fid, 'Release: %s\n', manifest.releaseName);
fprintf(fid, 'Created: %s\n', char(manifest.created));
fprintf(fid, 'Source: %s\n', manifest.sourceFile);
fprintf(fid, 'Script: %s\n', manifest.scriptFile);
fprintf(fid, 'Model: %s\n', manifest.model);
fprintf(fid, 'Grid: %s\n', manifest.grid);
fprintf(fid, 'Interpolation safety factor: %.3f\n', manifest.interpolationSafetyFactor);
fprintf(fid, 'Branch policy: %s\n', manifest.branchPolicy);
fprintf(fid, 'Status: %s\n\n', ternary(manifest.releasePassed, 'RELEASED', 'NOT RELEASED'));
for row = 1:height(criteria)
    fprintf(fid, '%s: %s (measured %.6g, limit %.6g %s)\n', ...
        criteria.Criterion(row), ternary(criteria.Passed(row), 'PASS', 'FAIL'), ...
        criteria.Measured(row), criteria.Limit(row), criteria.Unit(row));
end
fprintf(fid, '\n%s\n', manifest.note);
end

function result = ternary(condition, yesValue, noValue)
if condition, result = yesValue; else, result = noValue; end
end

function ratio = safeRatio(mapped, direct)
if direct <= 1e-9
    ratio = double(mapped <= 1e-9);
    if mapped > 1e-9
        ratio = Inf;
    end
else
    ratio = mapped / direct;
end
end

function values = metric(cases, eventId, useEnergy)
values = NaN(numel(cases),1);
for index = 1:numel(cases)
    result = cases(index).eventResults(find(strcmp({cases(index).eventResults.id}, eventId),1));
    if useEnergy, values(index) = result.energy.netBatteryEnergy_kWh;
    else, values(index) = result.time_s; end
end
end

function events = manualFsgEvents()
files = lts.io.trackCatalog();
events(1) = lts.events.createDefinition(Id="acceleration", Name="Acceleration", ...
    Discipline="acceleration", TrackFile=files.acceleration, IsClosed=false, ...
    InitialSpeed_mps=0, FinalSpeed_mps=100);
events(2) = lts.events.createDefinition(Id="skidpad", Name="Skidpad", ...
    Discipline="skidpad", TrackFile=files.skidpad, IsClosed=true, ...
    TimingWindows_m=[52,105;157,210], TimingAggregation="meanWindows");
events(3) = lts.events.createDefinition(Id="autocross", Name="Autocross", ...
    Discipline="autocross", TrackFile=files.autocross, IsClosed=false, ...
    InitialSpeed_mps=0, FinalSpeed_mps=100);
events(4) = lts.events.createDefinition(Id="endurance", Name="Endurance", ...
    Discipline="endurance", TrackFile=files.endurance, IsClosed=true, TimeMultiplier=18);
end

function references = referenceTimes()
references.acceleration.bestTime_s = 3.369;
references.skidpad.bestTime_s = 4.678;
references.autocross.bestTime_s = 72.925;
references.endurance.bestTime_s = 1325.67;
references.efficiency.minimumFactor_s2kWh = 8725793.17;
end
