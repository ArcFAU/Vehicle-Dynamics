function report = runFsg2026SlipEnvelopeTireComparison(options)
%RUNFSG2026SLIPENVELOPETIRECOMPARISON Compare physics and tire choices.
%   Runs the same TAU 2026 vehicle and FSG proxy events with:
%     1) dual-track envelope + fitted LC0
%     2) dual-track envelope + explicit R20 fallback approximation
%     3) slip-consistent dual-track map + fitted LC0
%     4) slip-consistent dual-track map + R20 TMeasy
%   Existing files are never overwritten. The R20 envelope case is clearly
%   classified as an approximation; only the slip case evaluates TMeasy.
arguments
    options.SpeedRows (1,1) double {mustBeInteger,mustBeGreaterThanOrEqual(options.SpeedRows,5)} = 7
    options.LateralFractions (1,:) double = [0,.25,.5,.75,1]
    options.BoundaryIterations (1,1) double {mustBeInteger,mustBePositive} = 10
    options.InterpolationSafetyFactor (1,1) double {mustBePositive,mustBeLessThanOrEqual(options.InterpolationSafetyFactor,1)} = .8
    options.ShowProgress (1,1) logical = true
end

root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(root,fullfile(root,'vehicles'),fullfile(root,'config'));

base = FAUmaxTauV2("fourMotorAWD");
base.powertrain.maximumPower_W = 80e3;
base.energy.maximumBatteryDischargePower_W = 80e3;
base.controls.driveAllocation.mode = 'fixed';
base.controls.brakeAllocation.mode = 'fixed';
base.controls.regenAllocation.mode = 'fixed';

lc0 = lts.tire.selectModel(base, "fittedCombinedSlip");
r20 = lts.tire.selectModel(base, "tmeasyAdtm");
r20.tireModel.allowFastEnvelopeApproximation = true;
events = manualFsgEvents();
rules = lts.config.scoring.fsg2026(referenceTimes());

drive = lts.powertrain.maximumDriveForce(base.powertrain, 0);
speedGrid_mps = linspace(3, drive.maximumVehicleSpeed_mps, options.SpeedRows);
fprintf('\nFSG 2026 | envelope/slip x LC0/R20 comparison\n');
fprintf('Map grid: %d speeds x %d lateral fractions x 2 directions.\n', ...
    numel(speedGrid_mps), numel(options.LateralFractions));

maps.lc0 = buildMap(lc0, speedGrid_mps, options, "LC0 slip map");
maps.r20 = buildMap(r20, speedGrid_mps, options, "R20 TMeasy slip map");

cases(1) = runCase("Envelope LC0", "envelope", "LC0 fitted", ...
    lc0, [], events, rules);
cases(2) = runCase("Envelope R20 approximation", "envelope", ...
    "R20 symmetric fallback", r20, [], events, rules);
cases(3) = runCase("Slip LC0", "slip", "LC0 fitted", ...
    lc0, maps.lc0, events, rules);
cases(4) = runCase("Slip R20 TMeasy", "slip", "R20 TMeasy", ...
    r20, maps.r20, events, rules);

eventComparison = eventTable(cases);
summary = summaryTable(cases);
audit = auditOperatingPoints(lc0, r20);

report.meta.created = datetime('now', 'TimeZone', 'local');
report.meta.vehicle = base.meta;
report.meta.mapSettings = struct('speedGrid_mps', speedGrid_mps, ...
    'lateralFractions', options.LateralFractions, ...
    'boundaryIterations', options.BoundaryIterations, ...
    'interpolationSafetyFactor', options.InterpolationSafetyFactor);
report.meta.caveats = { ...
    'R20 envelope uses the explicit load-sensitive symmetric fallback, not TMeasy peak extraction.', ...
    'LC0 is a retained development fit and does not represent the selected R20 tire.', ...
    'R20 TMeasy lateral and longitudinal parameters originate from different Hoosier sizes.', ...
    'Autocross and endurance tracks are project proxies rather than measured FSG 2026 tracks.', ...
    'Energy is post-processed and not coupled back into event speed.'};
report.cases = cases;
report.maps = maps;
report.eventComparison = eventComparison;
report.summary = summary;
report.audit = audit;

folder = fullfile(root, 'results');
matFile = lts.v2.nextResultFile(string(folder), ...
    "fsg2026SlipEnvelopeTireComparison");
[resultFolder, stem] = fileparts(matFile);
eventsCsv = fullfile(resultFolder, stem + "_events.csv");
summaryCsv = fullfile(resultFolder, stem + "_summary.csv");
auditCsv = fullfile(resultFolder, stem + "_audit.csv");
report.files = struct('mat', matFile, 'eventsCsv', eventsCsv, ...
    'summaryCsv', summaryCsv, 'auditCsv', auditCsv);
save(matFile, 'report', '-v7.3');
writetable(eventComparison, eventsCsv);
writetable(summary, summaryCsv);
writetable(audit, auditCsv);
disp(summary);
disp(eventComparison);
fprintf('Saved: %s\n', matFile);
end

function map = buildMap(vehicle, speedGrid_mps, options, label)
map = lts.model.dualTrackSlip.buildPerformanceMap(vehicle, ...
    speedGrid_mps=speedGrid_mps, ...
    lateralFractions=options.LateralFractions, ...
    boundaryIterations=options.BoundaryIterations, ...
    interpolationSafetyFactor=options.InterpolationSafetyFactor, ...
    showProgress=options.ShowProgress, progressLabel=label, ...
    enableSolverRetries=true);
end

function output = runCase(name, physics, tire, vehicle, map, events, rules)
if physics == "envelope"
    model = lts.model.dualTrackEnvelope.create(vehicle);
    approximation = string(model.peakEnvelope.classification);
else
    model = lts.model.dualTrackSlip.create(vehicle, map);
    approximation = "slip-consistent";
end
eventResults = lts.events.runSet(events, model, ...
    IncludeDriverless=false, CoupleEnergy=false);
scoring = lts.scoring.scoreEvents(eventResults, rules, ...
    IncludeDriverless=false);
output.name = char(name);
output.physics = char(physics);
output.tire = char(tire);
output.classification = char(approximation);
output.vehicle = vehicle;
output.eventResults = eventResults;
output.scoring = scoring;
end

function tableOut = eventTable(cases)
rowCount = numel(cases) * 5;
configuration = strings(rowCount, 1);
physics = strings(rowCount, 1);
tire = strings(rowCount, 1);
classification = strings(rowCount, 1);
event = strings(rowCount, 1);
time_s = NaN(rowCount, 1);
points = NaN(rowCount, 1);
energy_kWh = NaN(rowCount, 1);
row = 0;
for caseIndex = 1:numel(cases)
    one = cases(caseIndex);
    for resultIndex = 1:numel(one.eventResults)
        row = row + 1;
        result = one.eventResults(resultIndex);
        scoreIndex = find(strcmp({one.scoring.events.eventId}, result.id), 1);
        configuration(row) = string(one.name);
        physics(row) = string(one.physics);
        tire(row) = string(one.tire);
        classification(row) = string(one.classification);
        event(row) = string(result.name);
        time_s(row) = result.time_s;
        points(row) = one.scoring.events(scoreIndex).points;
        energy_kWh(row) = result.energy.netBatteryEnergy_kWh;
    end
    row = row + 1;
    scoreIndex = find(strcmp({one.scoring.events.eventId}, 'efficiency'), 1);
    enduranceIndex = find(strcmp({one.eventResults.id}, 'endurance'), 1);
    configuration(row) = string(one.name);
    physics(row) = string(one.physics);
    tire(row) = string(one.tire);
    classification(row) = string(one.classification);
    event(row) = "Efficiency";
    time_s(row) = one.eventResults(enduranceIndex).time_s;
    points(row) = one.scoring.events(scoreIndex).points;
    energy_kWh(row) = one.eventResults(enduranceIndex).energy.netBatteryEnergy_kWh;
end
tableOut = table(configuration, physics, tire, classification, event, ...
    time_s, points, energy_kWh, 'VariableNames', {'Configuration', ...
    'Physics', 'Tire', 'Classification', 'Event', 'Time_s', 'Points', ...
    'NetBatteryEnergy_kWh'});
end

function tableOut = summaryTable(cases)
configuration = string({cases.name}).';
physics = string({cases.physics}).';
tire = string({cases.tire}).';
classification = string({cases.classification}).';
dynamicPoints = arrayfun(@(item) item.scoring.totalPoints, cases).';
acceleration_s = metric(cases, 'acceleration', 'time_s');
skidpad_s = metric(cases, 'skidpad', 'time_s');
autocross_s = metric(cases, 'autocross', 'time_s');
endurance_s = metric(cases, 'endurance', 'time_s');
enduranceEnergy_kWh = metric(cases, 'endurance', 'energy');
tableOut = table(configuration, physics, tire, classification, ...
    acceleration_s, skidpad_s, autocross_s, endurance_s, ...
    enduranceEnergy_kWh, dynamicPoints);
end

function values = metric(cases, eventId, field)
values = NaN(numel(cases), 1);
for index = 1:numel(cases)
    resultIndex = find(strcmp({cases(index).eventResults.id}, eventId), 1);
    result = cases(index).eventResults(resultIndex);
    if strcmp(field, 'energy')
        values(index) = result.energy.netBatteryEnergy_kWh;
    else
        values(index) = result.(field);
    end
end
end

function audit = auditOperatingPoints(lc0, r20)
speed_mps = [10; 15; 20; 25; 30];
ax_mps2 = [8; 0; 4; -8; 0];
ay_mps2 = [0; 10; 8; 6; -12];
names = ["Launch"; "Pure corner"; "Combined drive"; ...
    "Combined brake"; "High-speed corner"];
tires = {lc0, r20};
tireNames = ["LC0 fitted", "R20 TMeasy"];
rows = numel(speed_mps) * 2;
operatingPoint = strings(rows, 1);
tire = strings(rows, 1);
speed = zeros(rows, 1);
ax = zeros(rows, 1);
ay = zeros(rows, 1);
feasible = false(rows, 1);
physicallyValid = false(rows, 1);
residualInf = NaN(rows, 1);
bodySlip_deg = NaN(rows, 1);
steering_deg = NaN(rows, 1);
maximumAbsSlipRatio = NaN(rows, 1);
maximumAbsSlipAngle_deg = NaN(rows, 1);
row = 0;
for tireIndex = 1:2
    for pointIndex = 1:numel(speed_mps)
        row = row + 1;
        point = lts.model.dualTrackSlip.operatingPoint(tires{tireIndex}, ...
            speed_mps(pointIndex), ax_mps2(pointIndex), ay_mps2(pointIndex));
        operatingPoint(row) = names(pointIndex);
        tire(row) = tireNames(tireIndex);
        speed(row) = speed_mps(pointIndex);
        ax(row) = ax_mps2(pointIndex);
        ay(row) = ay_mps2(pointIndex);
        feasible(row) = point.feasible;
        physicallyValid(row) = point.physicallyValid;
        residualInf(row) = norm(point.solver.normalizedResidual, Inf);
        bodySlip_deg(row) = rad2deg(point.bodySlipAngle_rad);
        steering_deg(row) = rad2deg(point.steeringAngle_rad);
        maximumAbsSlipRatio(row) = max(abs(point.slipRatios));
        maximumAbsSlipAngle_deg(row) = max(abs(rad2deg(point.slipAngles_rad)));
    end
end
audit = table(operatingPoint, tire, speed, ax, ay, feasible, ...
    physicallyValid, residualInf, bodySlip_deg, steering_deg, ...
    maximumAbsSlipRatio, maximumAbsSlipAngle_deg, 'VariableNames', ...
    {'OperatingPoint', 'Tire', 'Speed_mps', 'Ax_mps2', 'Ay_mps2', ...
    'Feasible', 'PhysicallyValid', 'ResidualInf', 'BodySlip_deg', ...
    'Steering_deg', 'MaximumAbsSlipRatio', 'MaximumAbsSlipAngle_deg'});
end

function events = manualFsgEvents()
files = lts.io.trackCatalog();
events(1) = lts.events.createDefinition(Id="acceleration", ...
    Name="Acceleration", Discipline="acceleration", ...
    TrackFile=files.acceleration, IsClosed=false, ...
    InitialSpeed_mps=0, FinalSpeed_mps=100);
events(2) = lts.events.createDefinition(Id="skidpad", Name="Skidpad", ...
    Discipline="skidpad", TrackFile=files.skidpad, IsClosed=true, ...
    TimingWindows_m=[52,105;157,210], TimingAggregation="meanWindows");
events(3) = lts.events.createDefinition(Id="autocross", Name="Autocross", ...
    Discipline="autocross", TrackFile=files.autocross, IsClosed=false, ...
    InitialSpeed_mps=0, FinalSpeed_mps=100);
events(4) = lts.events.createDefinition(Id="endurance", Name="Endurance", ...
    Discipline="endurance", TrackFile=files.endurance, IsClosed=true, ...
    TimeMultiplier=18);
end

function references = referenceTimes()
references.acceleration.bestTime_s = 3.369;
references.skidpad.bestTime_s = 4.678;
references.autocross.bestTime_s = 72.925;
references.endurance.bestTime_s = 1325.67;
references.efficiency.minimumFactor_s2kWh = 8725793.17;
end
