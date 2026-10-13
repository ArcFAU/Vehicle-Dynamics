function comparison = runTau2026LegacyComparison()
%RUNTAU2026LEGACYCOMPARISON Compare legacy and nextGen with aligned inputs.
%   The historical SVN files remain read-only. The corrected legacy vehicle
%   is constructed in memory so that Tau.mat is never overwritten.

root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(root,fullfile(root,'vehicles'),fullfile(root,'config'));
legacyRoot = 'C:\Octanes\SVN\LaptimeSim\RundenzeitSimulation';
legacySimulation = fullfile(legacyRoot, 'Simulation');
legacyVehicleFile = fullfile(legacyRoot, 'Fahrzeuge', 'Tau.mat');
assert(isfile(legacyVehicleFile), 'Tau legacy vehicle not found.');
addpath(fullfile(root, 'vehicles'), legacySimulation);

loaded = load(legacyVehicleFile, 'C');
legacyOriginal = loaded.C;
[legacyCorrected, nextGenVehicle, changes] = correctedVehicles(legacyOriginal);

fprintf('\nTAU 2026 legacy/nextGen comparison\n');
fprintf('Legacy files are read-only; corrected inputs exist only in memory.\n');
oldFolder = pwd;
cleanup = onCleanup(@() cd(oldFolder));
cd(legacySimulation);
legacyOriginalResult = FSG(legacyOriginal, legacyOriginal, ...
    legacyOriginal, legacyOriginal, 12, 2);
legacyCorrectedResult = FSG(legacyCorrected, legacyCorrected, ...
    legacyCorrected, legacyCorrected, 12, 2);
cd(oldFolder);

events = manualFsgEvents();
rules = lts.config.scoring.fsg2026(referenceTimes());
model = lts.v2.buildDirectModel(Vehicle=nextGenVehicle, ...
    Model="dualTrack", Aero=true, LoadTransfer=true, Energy=true, ...
    YawInertia=false, ChassisTorsion=false);
nextGenEvents = lts.events.runSet(events, model, ...
    IncludeDriverless=false, CoupleEnergy=false);
nextGenScoring = lts.scoring.scoreEvents(nextGenEvents, rules, ...
    IncludeDriverless=false);

event = ["Acceleration"; "Skidpad"; "Autocross"; "Endurance"; "Energy"];
unit = ["s"; "s"; "s"; "s"; "kWh"];
legacyOriginalValues = legacyOriginalResult(:);
legacyCorrectedValues = legacyCorrectedResult(:);
nextGenValues = extractNextGen(nextGenEvents);
comparisonTable = table(event, unit, legacyOriginalValues, ...
    legacyCorrectedValues, nextGenValues, ...
    100 * (nextGenValues - legacyCorrectedValues) ...
    ./ legacyCorrectedValues, 'VariableNames', {'Event', 'Unit', ...
    'LegacyOriginal', 'LegacyCorrected2026', 'NextGenDualTrack2026', ...
    'NextGenVsCorrectedLegacy_percent'});

comparison.meta.created = datetime('now', 'TimeZone', 'local');
comparison.meta.legacyVehicleFile = legacyVehicleFile;
comparison.meta.legacyModel = 'ESSim: slip-based single-track model';
comparison.meta.nextGenModel = model.name;
comparison.meta.trackIdentity = verifyTrackIdentity(root, legacyRoot);
comparison.meta.caveats = { ...
    'Legacy ESSim applies traction at the rear axle and cannot represent four independent motors.', ...
    'Legacy rolling resistance is represented as one equivalent total force.', ...
    'The active LC0 fit is a development tire and not a validated R20 data set.', ...
    'Autocross and endurance tracks are project proxies, not FSG 2026 measured tracks.', ...
    'Energy algorithms are not equivalent and energy values are diagnostic only.'};
comparison.inputChanges = changes;
comparison.legacyOriginalVehicle = legacyOriginal;
comparison.legacyCorrectedVehicle = legacyCorrected;
comparison.nextGenVehicle = nextGenVehicle;
comparison.results = comparisonTable;
comparison.nextGenEventResults = nextGenEvents;
comparison.nextGenScoring = nextGenScoring;

resultFolder = fullfile(root, 'results');
matFile = lts.v2.nextResultFile(string(resultFolder), ...
    "tau2026LegacyComparison");
[folder, stem] = fileparts(matFile);
resultCsv = fullfile(folder, stem + "_events.csv");
inputCsv = fullfile(folder, stem + "_inputs.csv");
comparison.files = struct('mat', matFile, 'eventsCsv', resultCsv, ...
    'inputsCsv', inputCsv);
save(matFile, 'comparison');
writetable(comparisonTable, resultCsv);
writetable(changes, inputCsv);
disp(comparisonTable);
fprintf('Saved: %s\n', matFile);
end

function [legacy, vehicle, changes] = correctedVehicles(original)
vehicle = FAUmaxTauV2("fourMotorAWD");
% Explicit 2026 baseline. All values remain legal and within the currently
% declared TAU concept; no artificial 100 kW case is used.
vehicle.mass.carWithoutDriver_kg = 170;
vehicle.mass.driver_kg = 80;
vehicle.mass.total_kg = 250;
vehicle.geometry.wheelbase_m = 1.530;
vehicle.geometry.frontMassFraction = 0.470;
vehicle.geometry.cgHeight_m = 0.290;
vehicle.geometry.frontTrack_m = 1.240;
vehicle.geometry.rearTrack_m = 1.200;
vehicle.aerodynamics.referenceArea_m2 = 1.0;
vehicle.aerodynamics.totalLiftCoefficient = -5.4;
vehicle.aerodynamics.frontDownforceFraction = 0.50;
vehicle.aerodynamics.liftCoefficientFront = -2.7;
vehicle.aerodynamics.liftCoefficientRear = -2.7;
vehicle.aerodynamics.dragCoefficient = 1.7;
vehicle.powertrain.maximumPower_W = 80e3;
vehicle.powertrain.maximumMotorPower_W = 35e3;
vehicle.powertrain.maximumMotorTorque_Nm = 21;
vehicle.powertrain.gearRatio = 12.421;
vehicle.powertrain.effectiveTireRadius_m = 0.202;
vehicle.brakes.frontForceFraction = 0.70;

legacy = original;
legacy.m = vehicle.mass.total_kg;
legacy.l = vehicle.geometry.wheelbase_m;
legacy.lh = vehicle.geometry.frontMassFraction * legacy.l;
legacy.lv = legacy.l - legacy.lh;
legacy.h = vehicle.geometry.cgHeight_m;
legacy.wv = vehicle.geometry.frontTrack_m;
legacy.wh = vehicle.geometry.rearTrack_m;
legacy.A = vehicle.aerodynamics.referenceArea_m2;
legacy.Ca = -vehicle.aerodynamics.totalLiftCoefficient;
legacy.Cw = vehicle.aerodynamics.dragCoefficient;
legacy.av = vehicle.aerodynamics.frontDownforceFraction;
legacy.P = vehicle.powertrain.maximumPower_W ...
    * vehicle.powertrain.gearboxEfficiency;
legacy.ratio = vehicle.powertrain.gearRatio;
legacy.primaer = 1;
legacy.tk = 1;
legacy.rtire = vehicle.powertrain.effectiveTireRadius_m;
legacy.torque = [0, vehicle.powertrain.maximumMotorTorque_Nm; ...
    vehicle.powertrain.maximumMotorSpeed_rpm, ...
    vehicle.powertrain.maximumMotorTorque_Nm];
legacy.vmax = vehicle.powertrain.maximumMotorSpeed_rpm * 2 * pi / 60 ...
    / vehicle.powertrain.gearRatio * vehicle.powertrain.effectiveTireRadius_m;
legacy.bal = vehicle.brakes.frontForceFraction;
legacy.Frollv = vehicle.energy.rollingResistanceCoefficient ...
    * vehicle.mass.total_kg * 9.81;
legacy.Frollh = 0;
legacy.laengs = vehicle.tireModel.fittedCombinedSlip.longitudinalScale;
legacy.quer = vehicle.tireModel.fittedCombinedSlip.lateralScale;

input = ["Total mass"; "Wheelbase"; "Front mass fraction"; "CG height"; ...
    "Front track"; "Rear track"; "Reference area"; "Downforce coefficient"; ...
    "Drag coefficient"; "Power at model input"; "Gear ratio"; ...
    "Tire radius"; "Brake front fraction"; "Rolling resistance total"];
unit = ["kg"; "m"; "-"; "m"; "m"; "m"; "m^2"; "-"; "-"; "W"; ...
    "-"; "m"; "-"; "N"];
oldValue = [original.m; original.lv + original.lh; ...
    original.lh / (original.lv + original.lh); original.h; original.wv; ...
    original.wh; original.A; original.Ca; original.Cw; original.P; ...
    original.ratio; original.rtire; original.bal; ...
    original.Frollv + original.Frollh];
newValue = [legacy.m; legacy.l; legacy.lh / legacy.l; legacy.h; legacy.wv; ...
    legacy.wh; legacy.A; legacy.Ca; legacy.Cw; legacy.P; legacy.ratio; ...
    legacy.rtire; legacy.bal; legacy.Frollv + legacy.Frollh];
changes = table(input, unit, oldValue, newValue, newValue - oldValue, ...
    'VariableNames', {'Input', 'Unit', 'LegacyOriginal', ...
    'Corrected2026', 'Delta'});
end

function values = extractNextGen(results)
ids = {'acceleration', 'skidpad', 'autocross', 'endurance'};
values = NaN(5, 1);
for index = 1:4
    resultIndex = find(strcmp({results.id}, ids{index}), 1);
    values(index) = results(resultIndex).time_s;
end
enduranceIndex = find(strcmp({results.id}, 'endurance'), 1);
values(5) = results(enduranceIndex).energy.netBatteryEnergy_kWh;
end

function events = manualFsgEvents()
files = lts.io.trackCatalog();
events(1) = lts.events.createDefinition(Id="acceleration", ...
    Name="Acceleration", Discipline="acceleration", ...
    TrackFile=files.acceleration, IsClosed=false, ...
    InitialSpeed_mps=0, FinalSpeed_mps=100);
events(2) = lts.events.createDefinition(Id="skidpad", Name="Skidpad", ...
    Discipline="skidpad", TrackFile=files.skidpad, IsClosed=true, ...
    TimingWindows_m=[52, 105; 157, 210], TimingAggregation="meanWindows");
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

function identity = verifyTrackIdentity(root, legacyRoot)
names = {'A_Accel.mat', 'A_FSGautox.mat', 'A_FSGend_3.mat'};
identity = struct();
for index = 1:numel(names)
    current = fullfile(root, 'data', 'tracks', names{index});
    legacy = fullfile(legacyRoot, 'Strecken', names{index});
    currentData = load(current, 'A');
    legacyData = load(legacy, 'A');
    field = matlab.lang.makeValidName(erase(names{index}, '.mat'));
    identity.(field) = isequaln(currentData.A, legacyData.A);
    assert(identity.(field), 'Track mismatch: %s', names{index});
end
end
