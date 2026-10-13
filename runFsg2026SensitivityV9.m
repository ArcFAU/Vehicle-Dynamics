function report = runFsg2026SensitivityV9(options)
%RUNFSG2026SENSITIVITYV9 FSG sensitivity with event-specific driver masses.
% The yaw-inertia gate is diagnostic. A failed gate never changes lap times.

arguments
    options.ShowProgress (1,1) logical = true
    options.ValidationOnly (1,1) logical = false
    options.IncludeYawInertia (1,1) logical = true
    options.TireBasis (1,1) string ...
        {mustBeMember(options.TireBasis,["teamQuadratic","tmeasy050","tmeasy060"])} ...
        = "teamQuadratic"
    options.DriverMass_kg (1,1) struct = struct( ...
        'acceleration',80,'skidpad',80,'autocross',80,'endurance',80)
end

root = fileparts(mfilename('fullpath'));
stored.report = lts.io.loadR20Reference();
assert(stored.report.releasePassed,'Numerical reference is not released.');
base = lts.control.applyIdealAllocation(stored.report.vehicle);
base.controls.targetYawMoment_Nm = 0;
switch options.TireBasis
    case "teamQuadratic"
        envelope = lts.tire.createQuadraticPeakEnvelope();
    case "tmeasy050"
        envelope = lts.tire.fitTmeasyQuadraticEnvelope(base,MuScale=0.50);
    case "tmeasy060"
        envelope = lts.tire.fitTmeasyQuadraticEnvelope(base,MuScale=0.60);
end
base.tireEnvelope.peakEnvelope = envelope;
base.mass.driver_kg = 80;
base.mass.carWithoutDriver_kg = base.mass.total_kg-80;
% Remove only the artificial regenerative-deceleration ceiling in V9.
base.energy.maximumRegenDeceleration_mps2 = 100;
driverMass = validateDriverMass(options.DriverMass_kg);
settings = studySettings();
events = manualFsgEvents();
rules = lts.config.scoring.fsg2026(referenceTimes());
cache = fullfile(root,'results','fsg2026SensitivityV9_cache_modular_energy_v1', ...
    char(options.TireBasis),'unboundedRegenDecel',driverKey(driverMass));
if ~isfolder(cache), mkdir(cache); end

baseline = cachedCase(fullfile(cache,'baseline.mat'),base,events,rules,driverMass);
if options.IncludeYawInertia
    [yawGate,yawGateSummary] = validateYawGate(baseline,settings,envelope);
else
    yawGate = table();
    yawGateSummary = table();
end
% The current solver has no track-position argument in its boundary API.
% Even a feasible point audit cannot certify a coupled lap. In particular,
% the released baseline has infeasible high-yaw-demand states. V9 therefore
% never assigns a lap-time or points change to yaw inertia.
yawTreatment = "postProcessingOnly";
if options.ValidationOnly
    report = struct('settings',settings,'driverMass_kg',driverMass, ...
        'yawGate',yawGate,'yawGateSummary',yawGateSummary, ...
        'yawTreatment',yawTreatment,'cacheDirectory',string(cache));
    disp(yawGateSummary);
    return
end

aeroRows = struct([]);
for ci = 1:numel(settings.ClA_m2)
    for di = 1:numel(settings.CdA_m2)
        candidate = withAero(base,settings.ClA_m2(ci), ...
            settings.CdA_m2(di),base.aerodynamics.frontDownforceFraction);
        key = sprintf('aero_cl%02d_cd%02d.mat',ci,di);
        sim = cachedCase(fullfile(cache,key),candidate,events,rules,driverMass);
        aeroRows = appendRows(aeroRows,caseRow("aero",key,sim,settings));
        progress(options.ShowProgress,"aero",numel(aeroRows), ...
            numel(settings.ClA_m2)*numel(settings.CdA_m2));
    end
end
aero = struct2table(aeroRows);

coupledRows = struct([]);
for mi = 1:numel(settings.FrontMassFractions)
    for ai = 1:numel(settings.AeroBalances)
        for ri = 1:numel(settings.FrontRollFractions)
            candidate = base;
            candidate.geometry.frontMassFraction = settings.FrontMassFractions(mi);
            candidate.suspension.frontRollMomentFraction = ...
                settings.FrontRollFractions(ri);
            candidate = withAero(candidate,settings.ReferenceClA_m2, ...
                settings.ReferenceCdA_m2,settings.AeroBalances(ai));
            key = sprintf('coupled_m%02d_a%02d_r%02d.mat',mi,ai,ri);
            sim = cachedCase(fullfile(cache,key),candidate,events,rules,driverMass);
            coupledRows = appendRows(coupledRows, ...
                caseRow("coupled",key,sim,settings));
            progress(options.ShowProgress,"coupled",numel(coupledRows), ...
                numel(settings.FrontMassFractions) ...
                *numel(settings.AeroBalances) ...
                *numel(settings.FrontRollFractions));
        end
    end
end
coupled = struct2table(coupledRows);

factorRows = struct([]);
factorNames = ["ReferenceTotalMass_kg","CgHeight_m", ...
    "SystemPower_kW","MotorTorque_Nm", ...
    "RegenPowerLimit_kW","BatteryChargePower_kW", ...
    "RegenMotorTorqueFraction"];
factorValues = {settings.ReferenceTotalMass_kg,settings.CgHeight_m, ...
    settings.SystemPower_kW,settings.MotorTorque_Nm, ...
    settings.RegenPowerLimit_kW,settings.BatteryChargePower_kW, ...
    settings.RegenMotorTorqueFraction};
for fi = 1:numel(factorNames)
    for vi = 1:numel(factorValues{fi})
        candidate = applyFactor(base,factorNames(fi),factorValues{fi}(vi));
        key = sprintf('factor_f%02d_v%02d.mat',fi,vi);
        sim = cachedCase(fullfile(cache,key),candidate,events,rules,driverMass);
        row = caseRow("oneFactor",key,sim,settings);
        row.Factor = factorNames(fi);
        row.FactorValue = factorValues{fi}(vi);
        factorRows = appendRows(factorRows,row);
        progress(options.ShowProgress,"oneFactor",numel(factorRows), ...
            sum(cellfun(@numel,factorValues)));
    end
end
oneFactor = struct2table(factorRows);

bestIndex = find(coupled.DynamicPoints==max(coupled.DynamicPoints),1);
bestCaseId = char(coupled.CaseId(bestIndex));
bestStored = load(fullfile(cache,bestCaseId),'simulation');
best = bestStored.simulation;
if options.IncludeYawInertia
    yawAudit = yawAuditTable({baseline,best},["Baseline","Best coupled"], ...
        settings.YawInertiaSensitivity_kgm2);
else
    yawAudit = table();
end

report.meta.created = datetime('now','TimeZone','local');
report.meta.tireBasis = options.TireBasis;
report.meta.model = 'dualTrackEnvelope, event-specific driver masses';
report.meta.energyKernel = 'modular-energy-v1';
report.meta.yawTreatment = yawTreatment;
report.meta.caveats = { ...
    'Yaw inertia changes moment demand only; lap times and points are unchanged.'; ...
    'The yaw gate is a point audit, not a converged yaw-coupled lap.'; ...
    'Roll moment fraction is varied directly, not mapped to spring hardware.'; ...
    'Driver mass changes total mass; its inertia contribution is not modeled.'; ...
    'Regen has no per-wheel tire-allocation check; total braking work is the aggregate bound.'; ...
    'SoC-dependent motor and battery charging limits remain active.'; ...
    'Constant aero coefficients and proxy tracks require experimental validation.'; ...
    'Historical overall cutoff is a comparison, not a rank guarantee.'};
report.settings = settings;
report.driverMass_kg = driverMass;
report.baseline = caseRow("baseline","baseline.mat",baseline,settings);
report.aero = aero;
report.coupled = coupled;
report.oneFactor = oneFactor;
report.bestCoupled = coupled(bestIndex,:);
report.yawGate = yawGate;
report.yawGateSummary = yawGateSummary;
report.yawAudit = yawAudit;

matFile = lts.v2.nextResultFile(string(fullfile(root,'results')), ...
    "fsg2026SensitivityV9_modular_energy_v1");
[folder,stem] = fileparts(matFile);
report.files.mat = matFile;
report.files.aeroCsv = fullfile(folder,stem+"_aero.csv");
report.files.coupledCsv = fullfile(folder,stem+"_coupled.csv");
report.files.oneFactorCsv = fullfile(folder,stem+"_oneFactor.csv");
report.files.yawGateCsv = fullfile(folder,stem+"_yawGate.csv");
report.files.yawGateSummaryCsv = fullfile(folder,stem+"_yawGateSummary.csv");
report.files.yawAuditCsv = fullfile(folder,stem+"_yawInertiaAudit.csv");
save(matFile,'report','-v7.3');
writetable(aero,report.files.aeroCsv);
writetable(coupled,report.files.coupledCsv);
writetable(oneFactor,report.files.oneFactorCsv);
if options.IncludeYawInertia
    writetable(yawGate,report.files.yawGateCsv);
    writetable(yawGateSummary,report.files.yawGateSummaryCsv);
    writetable(yawAudit,report.files.yawAuditCsv);
end
fprintf('Saved V9 results: %s\n',matFile);
end

function s = studySettings()
s.ClA_m2 = [0,3,4.5,5.4,6.3,7.2];
s.CdA_m2 = [0.8,1.2,1.7,2.0,2.5];
s.ReferenceClA_m2 = 5.4;
s.ReferenceCdA_m2 = 1.7;
s.FrontMassFractions = [0.47,0.485,0.50,0.51];
s.AeroBalances = [0.45,0.50,0.55];
s.FrontRollFractions = [0.50,0.55,0.60,0.65,0.70,0.75];
s.ReferenceTotalMass_kg = [220,230,240,250,260,270];
s.CgHeight_m = [0.25,0.26,0.27,0.28,0.29,0.30];
s.SystemPower_kW = [60,65,70,75,80,90,100];
s.MotorTorque_Nm = [17,21,25,26.25];
s.RegenPowerLimit_kW = [-15,-30,-45,-60,-90,-120];
s.BatteryChargePower_kW = [30,45,60,90,120];
s.RegenMotorTorqueFraction = [0.25,0.50,0.75,1,1.25];
s.YawInertiaValidation_kgm2 = [0,60,90.5,130];
s.YawInertiaSensitivity_kgm2 = [60,75,90,100];
s.OverallTop10Cutoff_points = 628.83;
s.Reserve_points = 20;
end

function masses = validateDriverMass(masses)
names = {'acceleration','skidpad','autocross','endurance'};
assert(isequal(sort(fieldnames(masses)),sort(names.')), ...
    'DriverMass_kg must contain acceleration, skidpad, autocross, endurance.');
for i = 1:numel(names)
    value = masses.(names{i});
    validateattributes(value,{'numeric'}, ...
        {'scalar','real','finite','positive'});
end
end

function key = driverKey(masses)
key = sprintf('driver_%g_%g_%g_%g',masses.acceleration, ...
    masses.skidpad,masses.autocross,masses.endurance);
key = regexprep(key,'[^A-Za-z0-9_]','p');
end

function vehicle = withAero(vehicle,clA,cdA,frontBalance)
area = vehicle.aerodynamics.referenceArea_m2;
vehicle.aerodynamics.totalLiftCoefficient = -clA/area;
vehicle.aerodynamics.dragCoefficient = cdA/area;
vehicle.aerodynamics.frontDownforceFraction = frontBalance;
vehicle.aerodynamics.liftCoefficientFront = ...
    vehicle.aerodynamics.totalLiftCoefficient*frontBalance;
vehicle.aerodynamics.liftCoefficientRear = ...
    vehicle.aerodynamics.totalLiftCoefficient*(1-frontBalance);
end

function vehicle = applyFactor(vehicle,name,value)
switch name
    case "ReferenceTotalMass_kg"
        vehicle.mass.carWithoutDriver_kg = value-80;
        vehicle.mass.total_kg = value;
    case "CgHeight_m"
        vehicle.geometry.cgHeight_m = value;
    case "SystemPower_kW"
        vehicle.powertrain.maximumPower_W = value*1000;
    case "MotorTorque_Nm"
        vehicle.powertrain.maximumMotorTorque_Nm = value;
    case "RegenPowerLimit_kW"
        vehicle.energy.maximumMechanicalRegenPower_W = abs(value)*1000;
    case "BatteryChargePower_kW"
        vehicle.energy.maximumBatteryChargePower_W = value*1000;
    case "RegenMotorTorqueFraction"
        vehicle.energy.maximumRegenMotorTorque_Nm = 18*value;
end
end

function sim = cachedCase(file,vehicle,events,rules,driverMass)
version = 'modular-energy-v1-event-mass-post-yaw';
if isfile(file)
    stored = load(file,'simulation');
    assert(strcmp(stored.simulation.modelVersion,version) ...
        && isequaln(stored.simulation.vehicle,vehicle) ...
        && isequaln(stored.simulation.driverMass_kg,driverMass), ...
        'V9 checkpoint identity mismatch: %s',file);
    sim = stored.simulation;
    return
end
eventResults = struct([]);
for i = 1:numel(events)
    eventVehicle = vehicle;
    eventVehicle.mass.driver_kg = driverMass.(events(i).id);
    eventVehicle.mass.total_kg = eventVehicle.mass.carWithoutDriver_kg ...
        + eventVehicle.mass.driver_kg;
    model = lts.model.dualTrackEnvelope.create(eventVehicle);
    result = lts.events.runSet(events(i),model, ...
        IncludeDriverless=false,CoupleEnergy=false);
    eventResults = appendRows(eventResults,result);
end
scoring = lts.scoring.scoreEvents(eventResults,rules, ...
    IncludeDriverless=false);
assert(scoring.isComplete,'V9 scoring is incomplete: %s',file);
sim = struct('vehicle',vehicle,'driverMass_kg',driverMass, ...
    'eventResults',eventResults,'scoring',scoring, ...
    'dynamicPoints',scoring.totalPoints,'modelVersion',version);
simulation = sim;
temporary = string(tempname(fileparts(file)))+".mat";
cleanup = onCleanup(@()deleteIfPresent(temporary));
save(temporary,'simulation','-v7.3');
[ok,message] = movefile(temporary,file,'f');
assert(ok,'%s',message);
clear cleanup
end

function deleteIfPresent(file)
if isfile(file), delete(file); end
end

function rows = appendRows(rows,newRows)
if isempty(rows), rows = newRows;
else, rows = [rows;newRows]; end
end

function row = caseRow(phase,key,sim,settings)
v = sim.vehicle;
row.Phase = phase;
row.CaseId = string(key);
row.ClA_m2 = -v.aerodynamics.totalLiftCoefficient ...
    *v.aerodynamics.referenceArea_m2;
row.CdA_m2 = v.aerodynamics.dragCoefficient ...
    *v.aerodynamics.referenceArea_m2;
row.FrontAeroBalance = v.aerodynamics.frontDownforceFraction;
row.FrontMassFraction = v.geometry.frontMassFraction;
row.CgFromRearAxle_m = v.geometry.frontMassFraction ...
    *v.geometry.wheelbase_m;
row.FrontRollMomentFraction = v.suspension.frontRollMomentFraction;
row.ReferenceTotalMass_kg = v.mass.carWithoutDriver_kg+80;
row.CgHeight_m = v.geometry.cgHeight_m;
row.SystemPower_kW = v.powertrain.maximumPower_W/1000;
row.MotorTorque_Nm = v.powertrain.maximumMotorTorque_Nm;
row.RegenPowerLimit_kW = -v.energy.maximumMechanicalRegenPower_W/1000;
row.BatteryChargePower_kW = v.energy.maximumBatteryChargePower_W/1000;
row.RegenMotorTorque_Nm = v.energy.maximumRegenMotorTorque_Nm;
row.RegenMotorTorqueFraction = v.energy.maximumRegenMotorTorque_Nm/18;
row.AccelerationDriver_kg = sim.driverMass_kg.acceleration;
row.SkidpadDriver_kg = sim.driverMass_kg.skidpad;
row.AutocrossDriver_kg = sim.driverMass_kg.autocross;
row.EnduranceDriver_kg = sim.driverMass_kg.endurance;
row.AccelerationMass_kg = v.mass.carWithoutDriver_kg ...
    +sim.driverMass_kg.acceleration;
row.EnduranceMass_kg = v.mass.carWithoutDriver_kg ...
    +sim.driverMass_kg.endurance;
row.Acceleration_s = eventTime(sim,'acceleration');
row.Skidpad_s = eventTime(sim,'skidpad');
row.Autocross_s = eventTime(sim,'autocross');
row.Endurance_s = eventTime(sim,'endurance');
enduranceEnergy = eventById(sim,'endurance').energy;
row.EnduranceEnergy_kWh = enduranceEnergy.netBatteryEnergy_kWh;
row.EnduranceRecoveredEnergy_kWh = enduranceEnergy.recoveredEnergy_kWh;
row.EnduranceFrictionBrakeEnergy_kWh = ...
    enduranceEnergy.frictionBrakeEnergy_kWh;
row.Acceleration_points = eventScore(sim,'acceleration');
row.Skidpad_points = eventScore(sim,'skidpad');
row.Autocross_points = eventScore(sim,'autocross');
row.Endurance_points = eventScore(sim,'endurance');
row.Efficiency_points = eventScore(sim,'efficiency');
row.DynamicPoints = sim.dynamicPoints;
row.RequiredStaticForTop10_points = ...
    settings.OverallTop10Cutoff_points-sim.dynamicPoints;
row.RequiredStaticWithReserve_points = ...
    row.RequiredStaticForTop10_points+settings.Reserve_points;
end

function [details,summary] = validateYawGate(sim,settings,envelope)
eventIds = {'autocross','endurance'};
rows = struct([]);
summaryRows = struct([]);
for ei = 1:numel(eventIds)
    event = eventById(sim,eventIds{ei});
    vehicle = eventVehicle(sim,eventIds{ei});
    lap = event.lapResult;
    demand = lts.analysis.yawDemand(lap,vehicle);
    [~,order] = sort(abs(demand.yawAcceleration_radps2),'descend');
    selected = order(1:min(20,numel(order)));
    selected = selected(selected<numel(lap.speed_mps) ...
        & lap.speed_mps(selected)>0);
    zeroFeasible = false(numel(selected),1);
    for pi = 1:numel(selected)
        index = selected(pi);
        vehicle.controls.targetYawMoment_Nm = 0;
        state = lts.model.dualTrackEnvelope.calculateState(vehicle, ...
            lap.speed_mps(index),lap.segmentAcceleration_mps2(index), ...
            lap.lateralAcceleration_mps2(index),envelope);
        zeroFeasible(pi) = state.feasible;
    end
    for ii = 1:numel(settings.YawInertiaValidation_kgm2)
        inertia = settings.YawInertiaValidation_kgm2(ii);
        targetFeasible = false(numel(selected),1);
        for pi = 1:numel(selected)
            index = selected(pi);
            vehicle.controls.targetYawMoment_Nm = ...
                inertia*demand.yawAcceleration_radps2(index);
            state = lts.model.dualTrackEnvelope.calculateState(vehicle, ...
                lap.speed_mps(index),lap.segmentAcceleration_mps2(index), ...
                lap.lateralAcceleration_mps2(index),envelope);
            targetFeasible(pi) = state.feasible;
            row.Event = string(eventIds{ei});
            row.Index = index;
            row.YawInertia_kgm2 = inertia;
            row.TargetYawMoment_Nm = vehicle.controls.targetYawMoment_Nm;
            row.ZeroMomentFeasible = zeroFeasible(pi);
            row.TargetMomentFeasible = targetFeasible(pi);
            row.Speed_mps = lap.speed_mps(index);
            row.Ax_mps2 = lap.segmentAcceleration_mps2(index);
            row.Ay_mps2 = lap.lateralAcceleration_mps2(index);
            rows = appendRows(rows,row);
        end
        item.Event = string(eventIds{ei});
        item.YawInertia_kgm2 = inertia;
        item.CheckedPoints = numel(selected);
        item.ZeroMomentFeasibleCount = sum(zeroFeasible);
        item.TargetMomentFeasibleCount = sum(targetFeasible);
        item.CouplingReleased = false;
        summaryRows = appendRows(summaryRows,item);
    end
end
details = struct2table(rows);
summary = struct2table(summaryRows);
end

function out = yawAuditTable(sims,names,inertias)
rows = struct([]);
for si = 1:numel(sims)
    for eventId = ["autocross","endurance"]
        event = eventById(sims{si},char(eventId));
        vehicle = eventVehicle(sims{si},char(eventId));
        demand = lts.analysis.yawDemand(event.lapResult,vehicle);
        accelerations = sort(abs(demand.yawAcceleration_radps2(:)));
        index95 = max(1,ceil(0.95*numel(accelerations)));
        for ii = 1:numel(inertias)
            row.Configuration = names(si);
            row.Event = eventId;
            row.YawInertia_kgm2 = inertias(ii);
            row.PeakInertialYawMoment_Nm = ...
                inertias(ii)*accelerations(end);
            row.P95InertialYawMoment_Nm = ...
                inertias(ii)*accelerations(index95);
            row.LapTime_s = event.time_s;
            row.PointsEffectAvailable = false;
            rows = appendRows(rows,row);
        end
    end
end
out = struct2table(rows);
end

function vehicle = eventVehicle(sim,id)
vehicle = sim.vehicle;
vehicle.mass.driver_kg = sim.driverMass_kg.(id);
vehicle.mass.total_kg = vehicle.mass.carWithoutDriver_kg ...
    +vehicle.mass.driver_kg;
end

function result = eventById(sim,id)
index = find(strcmp({sim.eventResults.id},id),1);
assert(~isempty(index),'Missing event: %s.',id);
result = sim.eventResults(index);
end

function time_s = eventTime(sim,id)
event = eventById(sim,id);
time_s = event.time_s;
end

function points = eventScore(sim,id)
scores = sim.scoring.events;
index = find(strcmp({scores.eventId},id),1);
assert(~isempty(index),'Missing event score: %s.',id);
points = scores(index).points;
end

function progress(enabled,phase,index,total)
if enabled, fprintf('V9 %s %d/%d\n',phase,index,total); end
end

function references = referenceTimes()
references.acceleration.bestTime_s = 3.369;
references.skidpad.bestTime_s = 4.678;
references.autocross.bestTime_s = 72.925;
references.endurance.bestTime_s = 1325.67;
references.efficiency.minimumFactor_s2kWh = 8725793.17;
end

function events = manualFsgEvents()
files = lts.io.trackCatalog();
events(1) = lts.events.createDefinition(Id="acceleration", ...
    Name="Acceleration",Discipline="acceleration", ...
    TrackFile=files.acceleration,IsClosed=false, ...
    InitialSpeed_mps=0,FinalSpeed_mps=100);
events(2) = lts.events.createDefinition(Id="skidpad", ...
    Name="Skidpad",Discipline="skidpad",TrackFile=files.skidpad, ...
    IsClosed=true,TimingWindows_m=[52,105;157,210], ...
    TimingAggregation="meanWindows");
events(3) = lts.events.createDefinition(Id="autocross", ...
    Name="Autocross",Discipline="autocross", ...
    TrackFile=files.autocross,IsClosed=false, ...
    InitialSpeed_mps=0,FinalSpeed_mps=100);
events(4) = lts.events.createDefinition(Id="endurance", ...
    Name="Endurance",Discipline="endurance",TrackFile=files.endurance, ...
    IsClosed=true,TimeMultiplier=18);
end
